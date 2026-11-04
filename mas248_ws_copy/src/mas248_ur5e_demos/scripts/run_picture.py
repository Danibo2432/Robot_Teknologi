"""Plan (and optionally simulate) image contour strokes with the UR5e mock.

The image_to_targets command must be run first. This program loads its JSON,
plans a joint-space move to each approach pose, then Cartesian draw/retreat
segments. It preflights every segment before executing any trajectory.
"""
import argparse
import json
import math
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from .common import cartesian, execute_trajectory, inverse_kinematics, make_pose, move_group


def _pose(data, z=None):
    pose = make_pose(data['x'], data['y'], data['z'] if z is None else z)
    pose.orientation.x = float(data.get('qx', 1.0))
    pose.orientation.y = float(data.get('qy', 0.0))
    pose.orientation.z = float(data.get('qz', 0.0))
    pose.orientation.w = float(data.get('qw', 0.0))
    return pose


def _last_joints(trajectory):
    points = trajectory.joint_trajectory.points
    if not points:
        raise RuntimeError('Planned trajectory has no joint points')
    return [float(v) for v in points[-1].positions]


def _check_cartesian(result, label, threshold):
    if result.error_code.val != 1:
        raise RuntimeError(f'{label}: Cartesian planner error {result.error_code.val}')
    if result.fraction < threshold:
        raise RuntimeError(f'{label}: only {result.fraction:.1%} of path planned '
                           f'(required {threshold:.1%}); refusing partial path')
    if not result.solution.joint_trajectory.points:
        raise RuntimeError(f'{label}: empty Cartesian trajectory')


def _load_limits():
    config_dir = Path(get_package_share_directory('mas248_ur5e_moveit_config')) / 'config'
    with (config_dir / 'drawing_limits.yaml').open(encoding='utf-8') as stream:
        settings = yaml.safe_load(stream)
    area, motion = settings['work_area'], settings['motion']
    for axis in ('x', 'y', 'z'):
        if float(area['min'][axis]) >= float(area['max'][axis]):
            raise ValueError(f'Work-area min must be below max for {axis}')
    for key in ('velocity_scaling_factor', 'acceleration_scaling_factor'):
        if not 0.0 < float(motion[key]) <= 1.0:
            raise ValueError(f'{key} must be in (0, 1]')
    if float(motion['fallback_max_joint_speed_rad_s']) <= 0.0:
        raise ValueError('fallback_max_joint_speed_rad_s must be > 0')
    dimensions_path = config_dir / 'workcell_dimensions.yaml'
    with dimensions_path.open(encoding='utf-8') as stream:
        dimensions = yaml.safe_load(stream)
    paper = dimensions['paper']
    pedestal = dimensions['pedestal']
    base_x = float(pedestal['world_origin_xyz_m'][0]) + float(pedestal['robot_mount_xyz_m'][0])
    base_y = float(pedestal['world_origin_xyz_m'][1]) + float(pedestal['robot_mount_xyz_m'][1])
    for key in ('size_x_m', 'size_y_m', 'thickness_m', 'drawing_width_m'):
        if float(paper[key]) <= 0.0:
            raise ValueError(f'paper.{key} must be > 0')
    return area, motion, paper, (base_x, base_y)

def _validate_drawing_points(strokes, area, paper):
    if area.get('frame', 'world') != 'world':
        raise ValueError('Work-area frame must be world')
    for stroke_no, stroke in enumerate(strokes, 1):
        for point_no, point in enumerate(stroke.get('poses', []), 1):
            for axis in ('x', 'y', 'z'):
                value = float(point[axis])
                low, high = float(area['min'][axis]), float(area['max'][axis])
                if not low <= value <= high:
                    raise ValueError(
                        f'Stroke {stroke_no}, point {point_no} outside work area: '
                        f'{axis}={value:.4f} m; allowed {low:.4f}..{high:.4f} m')
            half_x = float(paper['size_x_m']) / 2.0
            half_y = float(paper['size_y_m']) / 2.0
            paper_x0 = float(paper['center_x_m']) - half_x
            paper_x1 = float(paper['center_x_m']) + half_x
            paper_y0 = float(paper['center_y_m']) - half_y
            paper_y1 = float(paper['center_y_m']) + half_y
            if not paper_x0 <= float(point['x']) <= paper_x1 or not paper_y0 <= float(point['y']) <= paper_y1:
                raise ValueError(f'Stroke {stroke_no}, point {point_no} falls outside the paper sheet')
            z_error = abs(float(point['z']) - float(paper['surface_z_m']))
            if z_error > float(paper['target_z_tolerance_m']):
                raise ValueError(f'Stroke {stroke_no}, point {point_no} Z does not match paper surface')

def plan_picture(node, targets, *, approach_m, draw_lift_m, fraction, max_step,
                 work_area, paper, base_xy, velocity_scaling, acceleration_scaling):
    strokes = targets.get('strokes', [])
    if not strokes:
        raise ValueError('Targets JSON contains no strokes')
    _validate_drawing_points(strokes, work_area, paper)

    # Stage with the shoulder facing the paper. This avoids asking OMPL to
    # sweep the arm across the pedestal after the base has been raised.
    base_x, base_y = base_xy
    paper_heading = math.atan2(float(paper['center_y_m']) - base_y,
                               float(paper['center_x_m']) - base_x)
    start_plan = move_group(node, joints=[paper_heading, -1.57, 1.57, -1.57, -1.57, 0.0],
                            planning_time=30.0,
                            velocity_scaling=velocity_scaling,
                            acceleration_scaling=acceleration_scaling)
    current = _last_joints(start_plan.planned_trajectory)
    plans = [('Move to start', start_plan.planned_trajectory)]

    for index, stroke in enumerate(strokes, 1):
        points = stroke.get('poses', [])
        if len(points) < 2:
            node.get_logger().warning(f'Skipping stroke {index}: fewer than two points')
            continue
        first, last = points[0], points[-1]
        approach = _pose(first, first['z'] + approach_m)
        retreat = _pose(last, last['z'] + approach_m)

        # MoveGroup pose goals use OMPL joint-space planning.
        approach_joints = inverse_kinematics(node, approach, current)
        approach_plan = move_group(node, joints=approach_joints, start_joint_positions=current,
                                   planning_time=30.0,
                                   velocity_scaling=velocity_scaling,
                                   acceleration_scaling=acceleration_scaling)
        current = _last_joints(approach_plan.planned_trajectory)
        plans.append((f'Stroke {index} approach', approach_plan.planned_trajectory))

        raised_start = _pose(first, first['z'] + draw_lift_m)
        drawing = [_pose(point, point['z'] + draw_lift_m) for point in points]
        draw_result = cartesian(node, [raised_start] + drawing, max_step,
                                start_joint_positions=current,
                                velocity_scaling=velocity_scaling,
                                acceleration_scaling=acceleration_scaling)
        _check_cartesian(draw_result, f'Stroke {index} draw', fraction)
        current = _last_joints(draw_result.solution)
        plans.append((f'Stroke {index} draw', draw_result.solution))

        raised_end = _pose(last, last['z'] + approach_m)
        retreat_result = cartesian(node, [raised_end], max_step,
                                   start_joint_positions=current,
                                   velocity_scaling=velocity_scaling,
                                   acceleration_scaling=acceleration_scaling)
        _check_cartesian(retreat_result, f'Stroke {index} retreat', fraction)
        current = _last_joints(retreat_result.solution)
        plans.append((f'Stroke {index} retreat', retreat_result.solution))

    if len(plans) == 1:
        raise ValueError('No drawable strokes found')
    return plans


def get_parser():
    parser = argparse.ArgumentParser(
        description='Plan image contour strokes with MoveIt; execution is mock-only.')
    parser.add_argument('--targets', default='/tmp/mas248_targets.json',
                        help='JSON written by image_to_targets')
    parser.add_argument('--approach-mm', type=float, default=50.0,
                        help='TCP height above drawing plane for approach/retreat')
    parser.add_argument('--draw-lift-mm', type=float, default=2.0,
                        help='TCP height above the contour while drawing')
    parser.add_argument('--cartesian-fraction', type=float, default=0.98,
                        help='Minimum planned fraction for every Cartesian segment')
    parser.add_argument('--max-step-mm', type=float, default=5.0,
                        help='Maximum Cartesian interpolation step')
    parser.add_argument('--execute', action='store_true',
                        help='Execute only on the supplied mock controller')
    return parser


def main():
    args = get_parser().parse_args()
    if args.approach_mm <= args.draw_lift_mm or args.draw_lift_mm < 0:
        raise SystemExit('Require approach-mm > draw-lift-mm >= 0')
    if not 0.0 < args.cartesian_fraction <= 1.0 or args.max_step_mm <= 0:
        raise SystemExit('Require cartesian-fraction in (0,1] and max-step-mm > 0')
    target_file = Path(args.targets).expanduser()
    if not target_file.is_file():
        raise SystemExit(f'Target file not found: {target_file}; run image_to_targets first')
    with target_file.open(encoding='utf-8') as stream:
        targets = json.load(stream)

    rclpy.init()
    node = Node('mas248_run_picture')
    try:
        work_area, motion, paper, base_xy = _load_limits()
        node.get_logger().info('Configured drawing work area: ' + str(work_area))
        node.get_logger().info('Configured paper sheet: ' + str(paper))
        plans = plan_picture(node, targets,
            approach_m=args.approach_mm / 1000.0,
            draw_lift_m=args.draw_lift_mm / 1000.0,
            fraction=args.cartesian_fraction,
            max_step=args.max_step_mm / 1000.0,
            work_area=work_area,
            paper=paper,
            base_xy=base_xy,
            velocity_scaling=float(motion['velocity_scaling_factor']),
            acceleration_scaling=float(motion['acceleration_scaling_factor']))
        node.get_logger().info(f'Preflight complete: {len(plans)} trajectory segments planned')
        if args.execute:
            node.get_logger().warning('Executing on MoveIt controller; use the supplied mock only')
            for label, trajectory in plans:
                node.get_logger().info('Executing ' + label)
                execute_trajectory(node, trajectory,
                    max_joint_speed_rad_s=float(motion['fallback_max_joint_speed_rad_s']))
        else:
            node.get_logger().info('Plan only: no motion executed. Add --execute for mock simulation.')
    except Exception as exc:
        node.get_logger().error(str(exc))
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
