#!/usr/bin/env python3
"""MAS248 Lecture 10: draw image contours on the REAL UR5e.

DEFAULT = PLAN-ONLY PREFLIGHT. The robot never moves unless --execute is given
together with --acknowledge-hardware and --acknowledge-clearance, and physical
execution is only allowed with the instructor present.

Run from ~/mas248_ws (as on slide 22):
  PYTHONPATH="$HOME/mas248_ws/src/mas248_ur5e_demos:$PYTHONPATH" \
  python3 src/mas248_ur5e_real/scripts/run_picture_real.py \
      --targets /tmp/mas248_l10_targets_lower.json

It reuses the planning, checks and TCP trace from run_picture.py (Lecture 9):
  Phase 1  plan the whole chain without moving:
           home -> above stroke 1 -> strokes -> lift -> home
  Phase 2  check every segment (fraction, joint jumps, workspace), show the plan
  Phase 3  only with --execute: system checks, then execute segment by segment,
           asking for Enter before each one (unless --no-confirm).

IMPORTANT: Ctrl+C stops THIS program, not a motion that is already running.
To stop the robot, use the stop button or E-stop on the teach pendant.
"""
import argparse
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Bool
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from moveit_msgs.msg import DisplayTrajectory
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from geometry_msgs.msg import Point

from scripts.common import move_group, cartesian, execute_trajectory, JOINTS
from scripts.run_picture import (HOME, to_pose, shifted, dist, max_joint_step, end_joints,
                                 duration, slow_down, TcpTracer, line_marker)

REAL_CONTROLLER = 'scaled_joint_trajectory_controller'


# ---------------------------------------------------------------- system checks
def wait_for_message(node, msg_type, topic, timeout=3.0):
    box = []
    sub = node.create_subscription(msg_type, topic, lambda m: box.append(m), 10)
    end = time.time() + timeout
    while not box and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    return box[0] if box else None


def controller_state(node, name, timeout=5.0):
    try:
        from controller_manager_msgs.srv import ListControllers
    except ImportError:
        return 'unknown (controller_manager_msgs missing)'
    cli = node.create_client(ListControllers, '/controller_manager/list_controllers')
    if not cli.wait_for_service(timeout_sec=timeout):
        return 'no controller_manager'
    fut = cli.call_async(ListControllers.Request())
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    if not fut.done() or fut.result() is None:
        return 'no answer'
    for c in fut.result().controller:
        if c.name == name:
            return c.state
    return 'not loaded'


def system_checks(node, need_execution):
    """Return a list of (check, ok, detail). Execution needs ALL of them OK."""
    checks = []
    js = wait_for_message(node, JointState, '/joint_states')
    checks.append(('/joint_states received', js is not None,
                   'robot state is known' if js else 'no joint states: driver running?'))
    mg = ActionClient(node, MoveGroup, '/move_action').wait_for_server(timeout_sec=5.0)
    checks.append(('MoveIt /move_action', mg, 'available' if mg else 'start moveit_real.launch.py'))
    if need_execution:
        ex = ActionClient(node, ExecuteTrajectory, '/execute_trajectory').wait_for_server(timeout_sec=5.0)
        checks.append(('MoveIt /execute_trajectory', ex,
                       'available' if ex else 'launch MoveIt with enable_execution:=true'))
        state = controller_state(node, REAL_CONTROLLER)
        checks.append((REAL_CONTROLLER, state == 'active', state))
        prog = wait_for_message(node, Bool, '/io_and_status_controller/robot_program_running')
        running = prog is not None and prog.data
        checks.append(('External Control program running', running,
                       'true' if running else 'press Play on the pendant AFTER the driver runs'))
    return checks


# ---------------------------------------------------------------- target checks
def target_checks(strokes, z_offset, min_z, max_radius):
    """Simple geometric sanity checks on the target file before planning."""
    pts = [to_pose(p, z_offset).position for s in strokes for p in s['poses']]
    lowest = min(p.z for p in pts)
    widest = max(math.hypot(p.x, p.y) for p in pts)
    problems = []
    if min_z is not None and lowest < min_z:
        problems.append(f'lowest target z = {lowest:.3f} m is below --min-z {min_z:.3f} m')
    if widest > max_radius:
        problems.append(f'a target is {widest:.3f} m from the base axis, above --max-radius {max_radius:.2f} m')
    return lowest, widest, problems


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description='Plan-only preflight and supervised drawing on the real UR5e.')
    ap.add_argument('--targets', default='/tmp/mas248_l10_targets_lower.json', help='target JSON')
    ap.add_argument('--lift', type=float, default=0.05, help='pen-up height above the drawing [m]')
    ap.add_argument('--home-lift', type=float, default=0.15, help='lift before planning to home [m]')
    ap.add_argument('--z-offset', type=float, default=0.0, help='raise all targets by this much [m]')
    ap.add_argument('--min-z', type=float, default=None,
                    help='refuse targets below this world z [m]; set it from the measured table height')
    ap.add_argument('--max-radius', type=float, default=0.80,
                    help='refuse targets further than this from the base axis [m]')
    ap.add_argument('--min-fraction', type=float, default=0.99)
    ap.add_argument('--max-joint-step', type=float, default=0.35, help='[rad]')
    ap.add_argument('--max-strokes', type=int, default=0, help='first N strokes only (0 = all)')
    ap.add_argument('--speed', type=float, default=0.5,
                    help='extra slow-down 0 < speed <= 1 on top of MoveIt scaling (default 0.5)')
    ap.add_argument('--no-home', action='store_true',
                    help='start from the current pose instead of planning to home first')
    ap.add_argument('--execute', action='store_true', help='move the REAL robot (instructor present)')
    ap.add_argument('--acknowledge-hardware', action='store_true',
                    help='I confirm this moves the physical UR5e')
    ap.add_argument('--acknowledge-clearance', action='store_true',
                    help='I confirm the workspace is clear and the E-stop is within reach')
    ap.add_argument('--no-confirm', action='store_true', help='do not ask for Enter before each segment')
    args = ap.parse_args()

    if not 0.0 < args.speed <= 1.0:
        ap.error('--speed must be in (0, 1]')
    if args.execute and not (args.acknowledge_hardware and args.acknowledge_clearance):
        ap.error('--execute requires --acknowledge-hardware and --acknowledge-clearance')

    with open(args.targets) as f:
        data = json.load(f)
    strokes = [s for s in data['strokes'] if len(s['poses']) >= 2]
    if args.max_strokes > 0:
        strokes = strokes[:args.max_strokes]
    if not strokes:
        sys.exit('No strokes in target file')

    lowest, widest, problems = target_checks(strokes, args.z_offset, args.min_z, args.max_radius)
    print(f'Targets: {len(strokes)} strokes, lowest z = {lowest:.3f} m, widest reach = {widest:.3f} m')
    if args.min_z is None:
        print('WARNING: --min-z not set; the table height is not checked.')
    if problems:
        sys.exit('Target check FAILED: ' + '; '.join(problems))

    rclpy.init()
    node = Node('mas248_run_picture_real')
    log = node.get_logger()
    preview_pub = node.create_publisher(Marker, '/mas248/planned_drawing', 10)
    display_pub = node.create_publisher(DisplayTrajectory, '/display_planned_path', 10)
    tracer = None
    plan, report = [], []
    passed = False

    def plan_cartesian(waypoints, start, label):
        res = cartesian(node, waypoints, jump_threshold=0.0, start_joint_positions=start)
        step = max_joint_step(res.solution)
        if res.fraction < args.min_fraction:
            raise RuntimeError(f'{label}: only {res.fraction:.0%} planned')
        if step > args.max_joint_step:
            raise RuntimeError(f'{label}: joint flip of {step:.2f} rad')
        report.append((label, res.fraction, step, duration(res.solution) / args.speed))
        return res.solution

    def plan_joints(target, start, label):
        traj = move_group(node, joints=target, execute=False, planning_time=30.0,
                          start_joint_positions=start).planned_trajectory
        report.append((label, 1.0, max_joint_step(traj), duration(traj) / args.speed))
        return traj

    try:
        # ---------------- system checks
        checks = system_checks(node, need_execution=args.execute)
        for name, ok, detail in checks:
            print(f'  [{"OK" if ok else "FAIL"}] {name}: {detail}')
        if not all(ok for _, ok, _ in checks):
            raise RuntimeError('System check failed; nothing planned or executed')

        # ---------------- Phase 1: plan the whole chain
        log.info(f'PLANNING {len(strokes)} strokes (robot does not move)')
        joints = None
        if not args.no_home:
            traj = plan_joints(HOME, None, 'home')
            plan.append(('home', traj))
            joints = end_joints(traj, HOME)

        dz = args.z_offset
        first_up = to_pose(strokes[0]['poses'][0], dz + args.lift)
        traj = plan_cartesian([first_up], joints, 'approach')
        plan.append(('approach', traj))
        joints = end_joints(traj, joints or HOME)

        preview, last_up = [], first_up
        for i, stroke in enumerate(strokes, 1):
            pts = [to_pose(v, dz) for v in stroke['poses']]
            if dist(pts[0], pts[-1]) > 0.001:
                pts.append(to_pose(stroke['poses'][0], dz))
            up = shifted(pts[0], args.lift)
            label = f'stroke {i}/{len(strokes)} ({len(pts)} pts)'
            traj = plan_cartesian([up] + pts + [up], joints, label)    # any failure stops everything
            plan.append((label, traj))
            joints = end_joints(traj, joints)
            last_up = up
            preview += [(Point(x=a.position.x, y=a.position.y, z=a.position.z),
                         Point(x=b.position.x, y=b.position.y, z=b.position.z))
                        for a, b in zip(pts, pts[1:])]

        traj = plan_cartesian([shifted(last_up, args.home_lift)], joints, 'lift')
        plan.append(('lift', traj))
        joints = end_joints(traj, joints)
        traj = plan_joints(HOME, joints, 'return home')
        plan.append(('return home', traj))

        # ---------------- Phase 2: show the plan
        disp = DisplayTrajectory()
        moving = [t for _, t in plan if t.joint_trajectory.points]
        if moving:
            disp.trajectory_start.joint_state.name = list(moving[0].joint_trajectory.joint_names)
            disp.trajectory_start.joint_state.position = list(moving[0].joint_trajectory.points[0].positions)
        disp.trajectory = moving
        for _ in range(3):
            preview_pub.publish(line_marker('mas248_planned_drawing', preview, (0.2, 0.4, 1.0), 0.002))
            display_pub.publish(disp)
            time.sleep(0.3)
        print('Planning OK\nCollisions OK\nTrajectory valid\nPreflight check passed!')
        passed = True

        if not args.execute:
            log.warn('PLAN-ONLY: the robot did not move. Physical execution needs the instructor.')
            return

        # ---------------- Phase 3: supervised execution
        log.warn('EXECUTING ON THE REAL UR5e. Keep the E-stop within reach.')
        draw_z = lowest
        tracer = TcpTracer(pen_down_z=draw_z + 0.003)
        for label, traj in plan:
            if not traj.joint_trajectory.points:
                continue
            if not args.no_confirm:
                answer = input(f'Execute "{label}" ({duration(traj) / args.speed:.1f} s)? '
                               f'Enter = yes, q + Enter = stop: ')
                if answer.strip().lower() == 'q':
                    log.warn(f'Stopped by operator before "{label}". Robot left where it is.')
                    return
            try:
                execute_trajectory(node, slow_down(traj, args.speed))
            except Exception as exc:
                log.error(f'Execution of "{label}" FAILED: {exc}. Stopping. '
                          'Check the pendant for a protective stop before doing anything else.')
                return
            log.info(f'Executed: {label}')

    except Exception as exc:
        log.error(str(exc))
    finally:
        if tracer is not None:
            tracer.save('/tmp/mas248_tcp_trace_real.csv')
            tracer.stop()
        print('===== SUMMARY =====')
        for label, frac, step, secs in report:
            print(f'{label}: fraction {frac:.3f}, max joint step {step:.3f} rad, {secs:.1f} s')
        print(f'Total estimated time: {sum(r[3] for r in report):.0f} s, speed factor {args.speed}')
        if not passed:
            print('Preflight check FAILED: do not execute.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
