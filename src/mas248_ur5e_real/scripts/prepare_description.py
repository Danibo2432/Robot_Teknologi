#!/usr/bin/env python3
"""MAS248 Lecture 10: generate the REAL-robot workcell description at build time.

Called by CMakeLists.txt. Reads the Lecture 8 workcell Xacro (never modified) and
writes a derived copy into build/ where the UR macro:
  * generates the ros2_control tag for the real UR driver (not mock hardware),
  * takes calibration, limits and network settings from xacro arguments, so that
    ur_control.launch.py and moveit_real.launch.py can pass them in.
It also copies the official UR5e parameter files into build/config/ur5e/.

Only macro parameters that the INSTALLED ur_description version supports are
passed, so the script follows the UR version on each computer.
"""
import argparse
import re
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

XACRO_NS = 'http://www.ros.org/wiki/xacro'
ET.register_namespace('xacro', XACRO_NS)
X = '{%s}' % XACRO_NS

# Top-level xacro arguments of the derived file: name -> default.
# Names match ur_control.launch.py (Humble) and _real_common.render_description().
ARGS = {
    'name': 'ur5e', 'ur_type': 'ur5e', 'tf_prefix': '',
    'robot_ip': '0.0.0.0',
    'joint_limit_params': '$(find ur_description)/config/ur5e/joint_limits.yaml',
    'kinematics_params': '$(find ur_description)/config/ur5e/default_kinematics.yaml',
    'physical_params': '$(find ur_description)/config/ur5e/physical_parameters.yaml',
    'visual_params': '$(find ur_description)/config/ur5e/visual_parameters.yaml',
    'safety_limits': 'true', 'safety_pos_margin': '0.15', 'safety_k_position': '20',
    'use_fake_hardware': 'false', 'fake_sensor_commands': 'false', 'mock_control': 'false',
    'headless_mode': 'false', 'verify_robot_model': 'true', 'use_currents_as_efforts': 'true',
    'script_filename': '', 'input_recipe_filename': '', 'output_recipe_filename': '',
    'reverse_ip': '0.0.0.0', 'reverse_port': '50001', 'script_sender_port': '50002',
    'script_command_port': '50004', 'trajectory_port': '50003',
    'use_tool_communication': 'false', 'tool_voltage': '0', 'tool_parity': '0',
    'tool_baud_rate': '115200', 'tool_stop_bits': '1', 'tool_rx_idle_chars': '1.5',
    'tool_tx_idle_chars': '3.5', 'tool_device_name': '/tmp/ttyUR', 'tool_tcp_port': '54321',
}

# ur_robot macro parameter -> value in the derived file.
MACRO = {
    'name': '$(arg name)', 'tf_prefix': '$(arg tf_prefix)', 'ur_type': '$(arg ur_type)',
    'joint_limits_parameters_file': '$(arg joint_limit_params)',
    'kinematics_parameters_file': '$(arg kinematics_params)',
    'physical_parameters_file': '$(arg physical_params)',
    'visual_parameters_file': '$(arg visual_params)',
    'generate_ros2_control_tag': 'true',
    'safety_limits': '$(arg safety_limits)', 'safety_pos_margin': '$(arg safety_pos_margin)',
    'safety_k_position': '$(arg safety_k_position)',
    'use_fake_hardware': '$(arg use_fake_hardware)',
    'fake_sensor_commands': '$(arg fake_sensor_commands)',
    'headless_mode': '$(arg headless_mode)', 'robot_ip': '$(arg robot_ip)',
    'script_filename': '$(arg script_filename)',
    'input_recipe_filename': '$(arg input_recipe_filename)',
    'output_recipe_filename': '$(arg output_recipe_filename)',
    'reverse_ip': '$(arg reverse_ip)', 'reverse_port': '$(arg reverse_port)',
    'script_sender_port': '$(arg script_sender_port)',
    'script_command_port': '$(arg script_command_port)',
    'trajectory_port': '$(arg trajectory_port)',
    'use_tool_communication': '$(arg use_tool_communication)',
    'tool_voltage': '$(arg tool_voltage)', 'tool_parity': '$(arg tool_parity)',
    'tool_baud_rate': '$(arg tool_baud_rate)', 'tool_stop_bits': '$(arg tool_stop_bits)',
    'tool_rx_idle_chars': '$(arg tool_rx_idle_chars)',
    'tool_tx_idle_chars': '$(arg tool_tx_idle_chars)',
    'tool_device_name': '$(arg tool_device_name)', 'tool_tcp_port': '$(arg tool_tcp_port)',
    'verify_robot_model': '$(arg verify_robot_model)',
    'use_currents_as_efforts': '$(arg use_currents_as_efforts)',
}
KEEP = ('parent',)            # taken unchanged from the Lecture 8 workcell


def share(pkg, override=None):
    if override:
        return Path(override)
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory(pkg))


def macro_params(ur_description):
    """Parameter names of the installed ur_robot macro."""
    text = (ur_description / 'urdf' / 'ur_macro.xacro').read_text(encoding='utf-8')
    m = re.search(r'<xacro:macro\s+name="ur_robot"\s+params="([^"]*)"', text, re.S)
    if not m:
        raise RuntimeError('Cannot find the ur_robot macro in ur_description')
    return {p.split(':=')[0].lstrip('*') for p in m.group(1).split()}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--source', required=True)
    ap.add_argument('--output', required=True)
    ap.add_argument('--config-output', required=True)
    ap.add_argument('--ur-description-share', default=None, help='testing only')
    ap.add_argument('--ur-driver-share', default=None, help='testing only')
    a = ap.parse_args()

    ur_desc = share('ur_description', a.ur_description_share)
    supported = macro_params(ur_desc)

    tree = ET.parse(a.source)
    root = tree.getroot()
    robots = root.findall(X + 'ur_robot')
    if len(robots) != 1:
        raise RuntimeError(f'Expected one xacro:ur_robot in {a.source}, found {len(robots)}')
    robot = robots[0]

    # Replace the workcell's own top-level args with the full real-robot set.
    for el in root.findall(X + 'arg'):
        root.remove(el)
    for i, (name, default) in enumerate(ARGS.items()):
        arg = ET.Element(X + 'arg', {'name': name, 'default': default})
        arg.tail = '\n  '
        root.insert(i, arg)

    # Rewrite the ur_robot call: keep parent and origin, pass everything else as args.
    kept = {k: robot.get(k) for k in KEEP if robot.get(k) is not None}
    robot.attrib.clear()
    robot.attrib.update(kept)
    skipped = []
    for param, value in MACRO.items():
        if param in supported:
            robot.set(param, value)
        else:
            skipped.append(param)

    out = Path(a.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    tree.write(out, encoding='utf-8', xml_declaration=True)

    # Official UR5e parameter files, referenced by the driver as config/ur5e/*.yaml.
    cfg = Path(a.config_output)
    cfg.mkdir(parents=True, exist_ok=True)
    for f in ('joint_limits.yaml', 'physical_parameters.yaml', 'visual_parameters.yaml',
              'default_kinematics.yaml'):
        shutil.copy(ur_desc / 'config' / 'ur5e' / f, cfg / f)
    try:
        rate = share('ur_robot_driver', a.ur_driver_share) / 'config' / 'ur5e_update_rate.yaml'
        if rate.is_file():
            shutil.copy(rate, cfg.parent / 'ur5e_update_rate.yaml')
    except Exception:
        pass   # the copy in the package source is used instead

    print(f'Generated {out} (parameters not supported by this UR version, skipped: '
          f'{", ".join(skipped) or "none"})')


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print(f'prepare_description failed: {exc}', file=sys.stderr)
        sys.exit(1)
