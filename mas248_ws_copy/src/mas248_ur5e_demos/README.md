## Paper sheet dimensions

The mock paper sheet is configured in `mas248_ur5e_moveit_config/config/workcell_dimensions.yaml`.
Its size and surface position are used by the URDF, shown in RViz, and checked
against every drawing point. The runner reads the paper centre, surface height,
and drawing width from this file when it creates image targets. Keep the sheet
inside the configured `drawing_limits.yaml` work area.

## Workcell dimensions

Edit `mas248_ur5e_moveit_config/config/workcell_dimensions.yaml` for the robot
mount position, measured pedestal footprint, and marker-tip/TCP offset. Lengths
are metres and rotations are radians. The TCP should be placed at the marker
contact point. The mock pedestal uses a 0.40 m square placeholder footprint and 0.772 m height. Replace the footprint with measured width and depth when available. Rebuild after edits.

# MAS248 UR5e drawing simulation (ROS 2 Humble)

This workspace launches a mock UR5e workcell, converts an image into drawing waypoints, displays those waypoints in RViz, and plans or simulates the motion with MoveIt 2. It is mock hardware only.

## Dependencies on Ubuntu 22.04

Install ROS 2 Humble, MoveIt 2, and the Universal Robots description/driver packages. Then install colcon and resolve this workspace's ROS dependencies:

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep
rosdep update
source /opt/ros/humble/setup.bash
cd ~/mas248_ws_copy
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
```

Run `sudo rosdep init` once beforehand if rosdep has not been initialized on this computer.

## Build

```bash
cd ~/mas248_ws_copy
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select mas248_ur5e_moveit_config mas248_ur5e_demos
source install/setup.bash
```

## Start the mock system

Use separate terminals. In each terminal, run:

```bash
source /opt/ros/humble/setup.bash
source ~/mas248_ws_copy/install/setup.bash
```

Terminal 1:

```bash
ros2 launch mas248_ur5e_moveit_config ros2_control.launch.py
```

Terminal 2:

```bash
ros2 launch mas248_ur5e_moveit_config moveit.launch.py
```

Wait for MoveIt/RViz to finish loading.

## Convert and display the sample picture

Terminal 3:

```bash
ros2 run mas248_ur5e_demos image_to_targets \
  ~/mas248_ws_copy/src/mas248_ur5e_demos/images/pumpkin_outline.jpg \
  --width-m 0.18 --center-x -0.43 --center-y -0.06 --center-z 0.462 \
  --tcp-offset-mm 2 --max-contours 4 --max-points 40 --open-stages --rviz
```

Review the six image-processing stages in the browser. RViz displays the intended target paths on `/mas248/drawing_targets`. The publisher also samples the actual `world -> tcp` transform and publishes the executed trace as `nav_msgs/msg/Path` on `/mas248/tcp_path`; the configured RViz display is named **Actual TCP Trace**. The actual trace grows while the mock robot moves and is retained for late subscribers. The target JSON is `/tmp/mas248_targets.json`.

To use your own picture, replace the image path. It should be high contrast (dark drawing on a light background). The converter extracts contour boundaries, not pen-stroke centrelines, so inspect the detected contours and points before planning. `--width-m` is the full raster image width in metres. `--center-x`, `--center-y`, and `--center-z` set the centre in the mock world. These coordinates are examples for this workcell, not calibration for a real robot.

## Plan and simulate the drawing

Terminal 4:

```bash
ros2 run mas248_ur5e_demos run_picture
```

This plans a start move, approach, drawing, and retreat for every stroke. It does not execute robot motion. Confirm that the paths look right in RViz and the command reports a successful preflight. To animate the motion on the mock controller, run:

```bash
ros2 run mas248_ur5e_demos run_picture --execute
```

Do not use this mock configuration or its example coordinates to control a physical robot.

## Stop

Press `Ctrl+C` in Terminals 3, 2, and 1. The image-target marker publisher (started by `--rviz`) runs until interrupted.
