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
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
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
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


def wait_for_message(node, msg_type, topic, timeout=3.0, qos=10):
    box = []
    sub = node.create_subscription(msg_type, topic, lambda m: box.append(m), qos)
    end = time.time() + timeout
    while not box and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    return box[0] if box else None


def controller_state(node, name, timeout=15.0):
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
        # Published only when it changes, with transient-local QoS (latched).
        prog = wait_for_message(node, Bool, '/io_and_status_controller/robot_program_running', qos=LATCHED)
        running = prog is not None and prog.data
        checks.append(('External Control program running', running,
                       'true' if running else 'press Play on the pendant AFTER the driver runs'))
    return checks


# ---------------------------------------------------------------- IK seeded from home
def compute_ik(node, pose, seed, timeout=10.0):
    """Ask MoveIt for joint angles that put the TCP at 'pose', starting the search
    from 'seed'. Returns six joint values or None. Seeding from home keeps the arm
    in a home-like configuration instead of a random IK branch."""
    from moveit_msgs.srv import GetPositionIK
    from builtin_interfaces.msg import Duration
    cli = node.create_client(GetPositionIK, '/compute_ik')
    if not cli.wait_for_service(timeout_sec=timeout):
        return None
    req = GetPositionIK.Request()
    ik = req.ik_request
    ik.group_name = 'manipulator'
    ik.ik_link_name = 'tcp'
    ik.avoid_collisions = True
    ik.pose_stamped.header.frame_id = 'world'
    ik.pose_stamped.pose = pose
    ik.robot_state.joint_state.name = list(JOINTS)
    ik.robot_state.joint_state.position = [float(v) for v in seed]
    ik.timeout = Duration(sec=1)
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    res = fut.result() if fut.done() else None
    if res is None or res.error_code.val != 1:
        return None
    named = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
    return [float(named[j]) for j in JOINTS]


# ---------------------------------------------------------------- FK of a joint pose
def compute_fk(node, joints, link='tcp', timeout=10.0):
    """Pose of 'link' in world for the given joint angles, from MoveIt."""
    from moveit_msgs.srv import GetPositionFK
    cli = node.create_client(GetPositionFK, '/compute_fk')
    if not cli.wait_for_service(timeout_sec=timeout):
        raise RuntimeError('/compute_fk not available')
    req = GetPositionFK.Request()
    req.header.frame_id = 'world'
    req.fk_link_names = [link]
    req.robot_state.joint_state.name = list(JOINTS)
    req.robot_state.joint_state.position = [float(v) for v in joints]
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    res = fut.result() if fut.done() else None
    if res is None or res.error_code.val != 1 or not res.pose_stamped:
        raise RuntimeError('FK of the drawing pose failed')
    return res.pose_stamped[0].pose


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
    ap.add_argument('--home', default='0.012,-1.572,-1.571,-1.571,1.571,-3.142',
                    help='parking pose at start and end [rad]: pan, lift, elbow, wrist1, wrist2, wrist3')
    ap.add_argument('--drawing-pose', default='0.422,-2.734,-1.265,-2.285,1.774,-3.142',
                    help='taught pose above the sheet, pen down [rad]. The robot always passes '
                         'through it on the way to and from the drawing, which fixes the arm shape.')
    ap.add_argument('--no-home', action='store_true',
                    help='skip the parking pose: go from the current pose straight to the drawing pose')
    ap.add_argument('--execute', action='store_true', help='move the REAL robot (instructor present)')
    ap.add_argument('--acknowledge-hardware', action='store_true',
                    help='I confirm this moves the physical UR5e')
    ap.add_argument('--acknowledge-clearance', action='store_true',
                    help='I confirm the workspace is clear and the E-stop is within reach')
    ap.add_argument('--no-confirm', action='store_true', help='do not ask for Enter before each segment')
    args = ap.parse_args()

    home = [float(v) for v in args.home.split(',')]
    drawing_pose = [float(v) for v in args.drawing_pose.split(',')]
    if len(home) != 6 or len(drawing_pose) != 6:
        ap.error('--home and --drawing-pose need six comma-separated joint angles')
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

    def check_valid(q):
        from moveit_msgs.srv import GetStateValidity
        cli = node.create_client(GetStateValidity, '/check_state_validity')
        if not cli.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('/check_state_validity not available')
        req = GetStateValidity.Request()
        req.group_name = 'manipulator'
        req.robot_state.joint_state.name = list(JOINTS)
        req.robot_state.joint_state.position = [float(v) for v in q]
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
        return fut.done() and fut.result() is not None and fut.result().valid

    def joint_line(start, goal, label, vmax=0.3, amax=0.3):
        """Deterministic move: every joint goes straight from start to goal at the same
        time, with a smooth (cosine) speed profile. Intermediate states are checked
        for collisions before anything is executed. No random planner involved."""
        from moveit_msgs.msg import RobotTrajectory
        from trajectory_msgs.msg import JointTrajectoryPoint
        from builtin_interfaces.msg import Duration
        start, goal = [float(v) for v in start], [float(v) for v in goal]
        dq = [g - s0 for s0, g in zip(start, goal)]
        span = max(abs(v) for v in dq)
        traj = RobotTrajectory()
        traj.joint_trajectory.joint_names = list(JOINTS)
        if span < 1e-4:
            report.append((label, 1.0, 0.0, 0.0))
            return traj
        T = max(math.pi / 2 * span / vmax, math.pi * math.sqrt(span / (2 * amax)), 0.5)
        n = max(10, int(math.ceil(span / 0.01)))
        for k in range(n + 1):
            t = T * k / n
            s_ = (1 - math.cos(math.pi * t / T)) / 2
            sd = math.pi / (2 * T) * math.sin(math.pi * t / T)
            sdd = (math.pi / T) ** 2 / 2 * math.cos(math.pi * t / T)
            q = [s0 + s_ * d for s0, d in zip(start, dq)]
            if k % 5 == 0 or k == n:
                if not check_valid(q):
                    raise RuntimeError(f'{label}: the straight joint path is in collision at '
                                       f'{100 * k // n} %. Move the robot or choose another --home.')
            pt = JointTrajectoryPoint()
            pt.positions = q
            pt.velocities = [sd * d for d in dq]
            pt.accelerations = [sdd * d for d in dq]
            pt.time_from_start = Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))
            traj.joint_trajectory.points.append(pt)
        report.append((label, 1.0, max_joint_step(traj), T / args.speed))
        return traj

    def current_joints():
        js = wait_for_message(node, JointState, '/joint_states')
        if js is None:
            raise RuntimeError('No /joint_states received')
        named = dict(zip(js.name, js.position))
        return [float(named[j]) for j in JOINTS]

    try:
        # ---------------- system checks
        checks = system_checks(node, need_execution=args.execute)
        for name, ok, detail in checks:
            print(f'  [{"OK" if ok else "FAIL"}] {name}: {detail}')
        if not all(ok for _, ok, _ in checks):
            raise RuntimeError('System check failed; nothing planned or executed')

        # ---------------- pen orientation = the one in the taught drawing pose
        # Every target gets exactly the TCP orientation of --drawing-pose (pen down,
        # flange towards the base), so the wrist never has to turn while drawing.
        o = compute_fk(node, drawing_pose).orientation
        pen_down = 1 - 2 * (o.x * o.x + o.y * o.y)      # world z of the TCP z axis
        if pen_down > -0.99:
            raise RuntimeError('The pen does not point down in --drawing-pose; choose another pose')
        for stroke in strokes:
            for pz in stroke['poses']:
                if isinstance(pz, dict):
                    pz['qx'], pz['qy'], pz['qz'], pz['qw'] = o.x, o.y, o.z, o.w
                else:
                    pz[3:7] = [o.x, o.y, o.z, o.w]
        print(f'Pen orientation taken from --drawing-pose: q = ({o.x:.3f}, {o.y:.3f}, {o.z:.3f}, {o.w:.3f})')

        # ---------------- Phase 1: plan the whole chain
        log.info(f'PLANNING {len(strokes)} strokes (robot does not move)')
        joints = current_joints()
        if not args.no_home:
            traj = joint_line(joints, home, 'to home')
            plan.append(('to home', traj))
            joints = list(home)
        traj = joint_line(joints, drawing_pose, 'to drawing pose')
        plan.append(('to drawing pose', traj))
        joints = list(drawing_pose)

        dz = args.z_offset
        first_up = to_pose(strokes[0]['poses'][0], dz + args.lift)
        # Straight line for the pen tip from the drawing pose to above stroke 1.
        traj = plan_cartesian([first_up], joints, 'approach')
        plan.append(('approach', traj))
        joints = end_joints(traj, joints)

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
        traj = joint_line(joints, drawing_pose, 'back to drawing pose')
        plan.append(('back to drawing pose', traj))
        if not args.no_home:
            traj = joint_line(drawing_pose, home, 'back to home')
            plan.append(('back to home', traj))

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
