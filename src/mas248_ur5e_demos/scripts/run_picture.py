"""MAS248: draw the picture from /tmp/mas248_targets.json with MoveIt 2.

SAFE WORKFLOW (same code for mock and real robot):
  Phase 1  PLAN everything first, as one chain of segments, without moving:
           home -> above stroke 1 -> stroke 1 -> ... -> stroke N -> lift -> home.
           Each segment starts where the previous one ends.
  Phase 2  CHECK every segment. By default NOTHING is executed if any segment fails.
           The planned drawing is shown in RViz (Marker /mas248/planned_drawing,
           and MotionPlanning 'Planned Path').
  Phase 3  EXECUTE the stored segments, optionally asking for Enter before each one.

Recommended options for a first real-robot test (after Lecture 10 setup):
  ros2 run mas248_ur5e_demos run_picture --dry-run
  ros2 run mas248_ur5e_demos run_picture --max-strokes 1 --z-offset 0.03 --speed 0.5 --confirm

Physical dimensions (pedestal, spindle, table, drawing height) must be measured
and set in the workcell model and in image_to_targets. They are NOT checked here.
"""
import argparse
import copy
import csv
import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.time import Time
from geometry_msgs.msg import Pose, Point
from visualization_msgs.msg import Marker
from moveit_msgs.msg import DisplayTrajectory
from tf2_ros import Buffer, TransformListener

from .common import move_group, cartesian, execute_trajectory, JOINTS, WORLD, TIP

HOME = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]


# ---------------------------------------------------------------- helpers
def to_pose(v, dz=0.0):
    """{x,y,z,qx,qy,qz,qw} or [x,y,z,qx,qy,qz,qw] in world (metres) -> Pose."""
    if isinstance(v, dict):
        v = [v['x'], v['y'], v['z'], v['qx'], v['qy'], v['qz'], v['qw']]
    p = Pose()
    p.position.x, p.position.y, p.position.z = float(v[0]), float(v[1]), float(v[2]) + dz
    p.orientation.x, p.orientation.y = float(v[3]), float(v[4])
    p.orientation.z, p.orientation.w = float(v[5]), float(v[6])
    return p


def shifted(pose, dz):
    p = copy.deepcopy(pose)
    p.position.z += dz
    return p


def dist(a, b):
    return math.dist((a.position.x, a.position.y, a.position.z),
                     (b.position.x, b.position.y, b.position.z))


def max_joint_step(traj):
    """Largest change of any joint between two neighbouring trajectory points [rad]."""
    pts = traj.joint_trajectory.points
    worst = 0.0
    for a, b in zip(pts, pts[1:]):
        worst = max(worst, max(abs(x - y) for x, y in zip(a.positions, b.positions)))
    return worst


def end_joints(traj, fallback):
    """Joint positions (in JOINTS order) at the end of a trajectory."""
    jt = traj.joint_trajectory
    if not jt.points:
        return list(fallback)
    named = dict(zip(jt.joint_names, jt.points[-1].positions))
    return [float(named[j]) for j in JOINTS]


def duration(traj):
    pts = traj.joint_trajectory.points
    if not pts:
        return 0.0
    t = pts[-1].time_from_start
    return t.sec + t.nanosec * 1e-9


def slow_down(traj, factor):
    """Slow a time-parameterized trajectory by 'factor' (0 < factor <= 1)."""
    if False:
        return traj
    out = copy.deepcopy(traj)
    out.joint_trajectory.header.stamp.sec = 0
    out.joint_trajectory.header.stamp.nanosec = 0
    for p in out.joint_trajectory.points:
        t = (p.time_from_start.sec + p.time_from_start.nanosec * 1e-9) / factor
        p.time_from_start.sec = int(t)
        p.time_from_start.nanosec = int(round((t - int(t)) * 1e9)) % 1000000000
        p.velocities = [v * factor for v in p.velocities]
        p.accelerations = [a * factor * factor for a in p.accelerations]
    return out


# ---------------------------------------------------------------- TCP trace
class TcpTracer:
    """Separate node + thread: samples TF world->tcp and draws the pen-down path in RViz."""

    def __init__(self, pen_down_z, rate_hz=20.0):
        self.node = Node('mas248_tcp_trace')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self.node)
        self.pub = self.node.create_publisher(Marker, '/mas248/tcp_trace', 10)
        self.pen_down_z = pen_down_z
        self.last = None
        self.segments = []
        self.samples = []
        self.node.create_timer(1.0 / rate_hz, self.sample)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        threading.Thread(target=self.executor.spin, daemon=True).start()

    def sample(self):
        try:
            t = self.buffer.lookup_transform(WORLD, TIP, Time())
        except Exception:
            return
        p = Point(x=t.transform.translation.x, y=t.transform.translation.y,
                  z=t.transform.translation.z)
        down = p.z < self.pen_down_z
        self.samples.append((time.time(), p.x, p.y, p.z, int(down)))
        if down and self.last is not None and self.last.z < self.pen_down_z:
            if math.dist((p.x, p.y), (self.last.x, self.last.y)) > 0.0005:
                self.segments.append((self.last, p))
                self.pub.publish(line_marker('mas248_tcp_trace', self.segments, (1.0, 0.5, 0.0)))
        self.last = p

    def save(self, path):
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t', 'x', 'y', 'z', 'pen_down'])
            w.writerows(self.samples)

    def stop(self):
        self.executor.shutdown()
        self.node.destroy_node()


def line_marker(ns, segments, rgb, width=0.003):
    m = Marker()
    m.header.frame_id = WORLD
    m.ns = ns
    m.id = 0
    m.type = Marker.LINE_LIST
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = width
    m.color.r, m.color.g, m.color.b = rgb
    m.color.a = 1.0
    for a, b in segments:
        m.points.extend([a, b])
    return m


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description='Plan, check and draw image contours with MoveIt 2.')
    ap.add_argument('--json', default='/tmp/mas248_targets.json', help='target file from image_to_targets')
    ap.add_argument('--lift', type=float, default=0.05, help='pen-up height above the drawing [m]')
    ap.add_argument('--home-lift', type=float, default=0.15, help='lift before planning to home [m]')
    ap.add_argument('--z-offset', type=float, default=0.0,
                    help='raise ALL drawing targets by this much [m], e.g. 0.03 for a first real test')
    ap.add_argument('--min-fraction', type=float, default=0.99, help='minimum Cartesian completion')
    ap.add_argument('--max-joint-step', type=float, default=0.35,
                    help='refuse Cartesian paths where a joint jumps more than this [rad]')
    ap.add_argument('--max-strokes', type=int, default=0, help='draw only the first N strokes (0 = all)')
    ap.add_argument('--speed', type=float, default=1.0,
                    help='extra slow-down factor 0 < speed <= 1 on top of MoveIt scaling (0.5 = half speed)')
    ap.add_argument('--dry-run', action='store_true', help='plan and visualise only, never move the robot')
    ap.add_argument('--confirm', action='store_true', help='ask for Enter before every segment')
    ap.add_argument('--skip-failed', action='store_true',
                    help='skip strokes that fail the checks instead of refusing to run at all')
    args = ap.parse_args()
    if not 0.0 < args.speed <= 1.0:
        ap.error('--speed must be in (0, 1]')

    with open(args.json) as f:
        data = json.load(f)
    strokes = [s for s in data['strokes'] if len(s['poses']) >= 2]
    if args.max_strokes > 0:
        strokes = strokes[:args.max_strokes]
    if not strokes:
        raise SystemExit('No strokes in target file')
    draw_z = min(to_pose(p).position.z for s in strokes for p in s['poses']) + args.z_offset

    rclpy.init()
    node = Node('mas248_run_picture')
    log = node.get_logger()
    preview_pub = node.create_publisher(Marker, '/mas248/planned_drawing', 10)
    display_pub = node.create_publisher(DisplayTrajectory, '/display_planned_path', 10)
    tracer = None
    plan = []        # list of (label, trajectory, kind)
    report = []      # list of (label, status, fraction, joint_step, seconds)

    def plan_cartesian(waypoints, start, label):
        res = cartesian(node, waypoints, jump_threshold=0.0, start_joint_positions=start)
        step = max_joint_step(res.solution)
        if res.fraction < args.min_fraction:
            raise RuntimeError(f'{label}: only {res.fraction:.0%} planned')
        if step > args.max_joint_step:
            raise RuntimeError(f'{label}: joint flip of {step:.2f} rad')
        report.append((label, 'OK', res.fraction, step, duration(res.solution) / args.speed))
        return res.solution

    def plan_joints(target, start, label):
        res = move_group(node, joints=target, execute=False, planning_time=30.0,
                         start_joint_positions=start)
        traj = res.planned_trajectory
        report.append((label, 'OK', 1.0, max_joint_step(traj), duration(traj) / args.speed))
        return traj

    try:
        # ---------------- Phase 1: plan the whole chain without moving
        log.info(f'PHASE 1: planning {len(strokes)} strokes, drawing height z = {draw_z:.3f} m')
        traj = plan_joints(HOME, None, 'home')                       # from current state
        plan.append(('home', traj, 'joint'))
        joints = end_joints(traj, HOME)

        dz = args.z_offset
        first_up = to_pose(strokes[0]['poses'][0], dz + args.lift)
        traj = plan_cartesian([first_up], joints, 'approach')
        plan.append(('approach', traj, 'cartesian'))
        joints = end_joints(traj, joints)

        preview = []
        last_up = first_up
        for i, stroke in enumerate(strokes, 1):
            pts = [to_pose(v, dz) for v in stroke['poses']]
            if dist(pts[0], pts[-1]) > 0.001:
                pts.append(copy.deepcopy(pts[0]))                    # close the contour
            up = shifted(pts[0], args.lift)
            label = f'stroke {i}/{len(strokes)} ({len(pts)} pts)'
            try:
                traj = plan_cartesian([up] + pts + [up], joints, label)
            except Exception as exc:
                report.append((label, f'FAILED: {exc}', 0.0, 0.0, 0.0))
                if not args.skip_failed:
                    raise RuntimeError(f'{exc}. Nothing will be executed (use --skip-failed to skip).')
                log.warn(f'{exc}; stroke skipped')
                continue
            plan.append((label, traj, 'cartesian'))
            joints = end_joints(traj, joints)
            last_up = up
            preview += [(Point(x=a.position.x, y=a.position.y, z=a.position.z),
                         Point(x=b.position.x, y=b.position.y, z=b.position.z))
                        for a, b in zip(pts, pts[1:])]

        lift = shifted(last_up, args.home_lift)
        traj = plan_cartesian([lift], joints, 'lift')
        plan.append(('lift', traj, 'cartesian'))
        joints = end_joints(traj, joints)
        traj = plan_joints(HOME, joints, 'return home')
        plan.append(('return home', traj, 'joint'))

        # ---------------- Phase 2: show the plan
        disp = DisplayTrajectory()
        first = next((t for _, t, _ in plan if t.joint_trajectory.points), None)
        if first is not None:
            disp.trajectory_start.joint_state.name = list(first.joint_trajectory.joint_names)
            disp.trajectory_start.joint_state.position = list(first.joint_trajectory.points[0].positions)
        disp.trajectory = [t for _, t, _ in plan if t.joint_trajectory.points]
        for _ in range(3):                                           # give RViz time to connect
            preview_pub.publish(line_marker('mas248_planned_drawing', preview, (0.2, 0.4, 1.0), 0.002))
            display_pub.publish(disp)
            time.sleep(0.3)
        total = sum(r[4] for r in report)
        log.info(f'PHASE 2: all {len(plan)} segments planned and checked, '
                 f'estimated execution time {total:.0f} s (speed factor {args.speed})')

        if args.dry_run:
            log.warn('DRY RUN: nothing executed. Check /mas248/planned_drawing and Planned Path in RViz.')
            return

        # ---------------- Phase 3: execute the stored, checked segments
        log.info('PHASE 3: executing')
        tracer = TcpTracer(pen_down_z=draw_z + 0.003)
        for label, traj, _ in plan:
            if not traj.joint_trajectory.points:
                continue                                             # already there
            if args.confirm:
                answer = input(f'Execute "{label}" ({duration(traj) / args.speed:.1f} s)? '
                               f'Enter = yes, q + Enter = stop: ')
                if answer.strip().lower() == 'q':
                    log.warn(f'Stopped by user before "{label}". Robot is left where it is.')
                    return
            execute_trajectory(node, slow_down(traj, args.speed))
            log.info(f'Executed: {label}')

    finally:
        if tracer is not None:
            tracer.save('/tmp/mas248_tcp_trace.csv')
            tracer.stop()
        log.info('===== SUMMARY =====')
        for label, status, frac, step, secs in report:
            log.info(f'{label}: {status}, fraction {frac:.3f}, max joint step {step:.3f} rad, {secs:.1f} s')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()