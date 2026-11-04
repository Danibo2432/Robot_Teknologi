"""Display image-derived drawing paths and their image frame in RViz."""
import argparse
import json
import math
from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped, TransformStamped
from nav_msgs.msg import Path as PathMsg
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy

COLORS = ((0.9, 0.25, 0.2), (0.15, 0.45, 0.9), (0.2, 0.7, 0.3),
          (0.7, 0.25, 0.8), (0.9, 0.55, 0.1))

class TargetViewer(Node):
    def __init__(self, targets, axis_every=10, axis_length_mm=18.0):
        super().__init__('mas248_show_targets_rviz')
        self.publisher = self.create_publisher(MarkerArray, '/mas248/drawing_targets', 10)
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        path_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_publisher = self.create_publisher(PathMsg, '/mas248/tcp_path', path_qos)
        self.tcp_path = PathMsg()
        self.tcp_path.header.frame_id = 'world'
        self._reported_tf_wait = False
        self.targets = targets
        limits_path = Path(get_package_share_directory('mas248_ur5e_moveit_config')) / 'config' / 'drawing_limits.yaml'
        with limits_path.open(encoding='utf-8') as stream:
            self.settings = yaml.safe_load(stream)
        self.work_area = self.settings['work_area']
        dimensions_path = Path(get_package_share_directory('mas248_ur5e_moveit_config')) / 'config' / 'workcell_dimensions.yaml'
        with dimensions_path.open(encoding='utf-8') as stream:
            self.paper = yaml.safe_load(stream)['paper']
        self.axis_every = max(1, int(axis_every))
        self.axis_length = float(axis_length_mm) / 1000.0
        self._publish_frame()
        self.markers = self._make_markers()
        self.timer = self.create_timer(1.0, self._publish)
        self.path_timer = self.create_timer(0.05, self._publish_tcp_path)
        self.get_logger().info('Publishing target markers on /mas248/drawing_targets and live TCP trace on /mas248/tcp_path')

    def _publish_frame(self):
        frame = self.targets.get('frame_to_world', {})
        xyz = frame.get('xyz', [0.0, 0.0, 0.0])
        q = frame.get('quaternion_xyzw', [0.0, 0.0, 0.0, 1.0])
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame.get('parent', 'world')
        msg.child_frame_id = frame.get('child', 'image_center')
        msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = map(float, xyz)
        msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = map(float, q)
        self.tf_broadcaster.sendTransform(msg)

    @staticmethod
    def _point(pose):
        point = Point()
        point.x, point.y, point.z = float(pose['x']), float(pose['y']), float(pose['z'])
        return point

    def _make_markers(self):
        array = MarkerArray()
        frame = self.targets.get('frame_to_world', {}).get('parent', 'world')
        stamp = self.get_clock().now().to_msg()
        for index, stroke in enumerate(self.targets.get('strokes', [])):
            poses = stroke.get('poses', [])
            if not poses:
                continue
            color = COLORS[index % len(COLORS)]
            line = Marker()
            line.header.frame_id, line.header.stamp = frame, stamp
            line.ns, line.id, line.type, line.action = 'drawing_strokes', index, Marker.LINE_STRIP, Marker.ADD
            line.pose.orientation.w = 1.0
            line.scale.x = 0.002
            line.color.r, line.color.g, line.color.b, line.color.a = *color, 1.0
            line.points = [self._point(pose) for pose in poses]
            array.markers.append(line)

            dots = Marker()
            dots.header.frame_id, dots.header.stamp = frame, stamp
            dots.ns, dots.id, dots.type, dots.action = 'drawing_waypoints', index, Marker.SPHERE_LIST, Marker.ADD
            dots.pose.orientation.w = 1.0
            dots.scale.x = dots.scale.y = dots.scale.z = 0.006
            dots.color.r, dots.color.g, dots.color.b, dots.color.a = *color, 1.0
            dots.points = [self._point(pose) for pose in poses]
            array.markers.append(dots)

            for point_index in range(0, len(poses), self.axis_every):
                pose = poses[point_index]
                arrow = Marker()
                arrow.header.frame_id, arrow.header.stamp = frame, stamp
                arrow.ns, arrow.id, arrow.type, arrow.action = 'tcp_axes', index * 10000 + point_index, Marker.ARROW, Marker.ADD
                arrow.pose.position = self._point(pose)
                arrow.pose.orientation.x = float(pose.get('qx', 1.0))
                arrow.pose.orientation.y = float(pose.get('qy', 0.0))
                arrow.pose.orientation.z = float(pose.get('qz', 0.0))
                arrow.pose.orientation.w = float(pose.get('qw', 0.0))
                arrow.scale.x, arrow.scale.y, arrow.scale.z = self.axis_length, 0.0015, 0.0025
                arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 0.1, 0.1, 0.1, 1.0
                array.markers.append(arrow)
        # Outline the configured work volume in RViz.
        area = self.work_area
        low, high = area['min'], area['max']
        xyz = ((low['x'], low['y'], low['z']), (high['x'], low['y'], low['z']),
               (high['x'], high['y'], low['z']), (low['x'], high['y'], low['z']),
               (low['x'], low['y'], high['z']), (high['x'], low['y'], high['z']),
               (high['x'], high['y'], high['z']), (low['x'], high['y'], high['z']))
        corners = [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in xyz]
        bounds = Marker()
        bounds.header.frame_id, bounds.header.stamp = area.get('frame', 'world'), stamp
        bounds.ns, bounds.id = 'work_area', 0
        bounds.type, bounds.action = Marker.LINE_LIST, Marker.ADD
        bounds.pose.orientation.w = 1.0
        bounds.scale.x = 0.004
        bounds.color.r, bounds.color.g, bounds.color.b, bounds.color.a = 1.0, 0.75, 0.05, 0.9
        for first, second in ((0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)):
            bounds.points.extend((corners[first], corners[second]))
        array.markers.append(bounds)

        # A translucent sheet marker makes the configured paper dimensions easy to see.
        paper = self.paper
        sheet = Marker()
        sheet.header.frame_id = 'world'
        sheet.header.stamp = stamp
        sheet.ns, sheet.id = 'paper_sheet', 0
        sheet.type, sheet.action = Marker.CUBE, Marker.ADD
        sheet.pose.position.x = float(paper['center_x_m'])
        sheet.pose.position.y = float(paper['center_y_m'])
        sheet.pose.position.z = float(paper['surface_z_m']) - float(paper['thickness_m']) / 2.0
        sheet.pose.orientation.w = 1.0
        sheet.scale.x = float(paper['size_x_m'])
        sheet.scale.y = float(paper['size_y_m'])
        sheet.scale.z = float(paper['thickness_m'])
        sheet.color.r, sheet.color.g, sheet.color.b, sheet.color.a = 0.85, 0.78, 0.48, 0.55
        array.markers.append(sheet)
        return array

    def _publish(self):
        self.publisher.publish(self.markers)

    def _publish_tcp_path(self):
        try:
            transform = self.tf_buffer.lookup_transform('world', 'tcp', Time())
        except TransformException as exc:
            if not self._reported_tf_wait:
                self.get_logger().warning(f'Waiting for world -> tcp transform: {exc}')
                self._reported_tf_wait = True
            return
        pose = PoseStamped()
        pose.header.frame_id = 'world'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        moved = True
        if self.tcp_path.poses:
            previous = self.tcp_path.poses[-1].pose.position
            dx = pose.pose.position.x - previous.x
            dy = pose.pose.position.y - previous.y
            dz = pose.pose.position.z - previous.z
            moved = dx*dx + dy*dy + dz*dz >= 1.0e-7
        if moved:
            if not self.tcp_path.poses:
                self.get_logger().info('Recording actual TCP motion to /mas248/tcp_path')
            self.tcp_path.poses.append(pose)
            if len(self.tcp_path.poses) > 10000:
                self.tcp_path.poses = self.tcp_path.poses[-10000:]
        self.tcp_path.header.stamp = pose.header.stamp
        self.path_publisher.publish(self.tcp_path)

def main():
    parser = argparse.ArgumentParser(description='Display drawing targets in RViz')
    parser.add_argument('--json', default='/tmp/mas248_targets.json')
    parser.add_argument('--axis-every', type=int, default=10)
    parser.add_argument('--axis-length-mm', type=float, default=18.0)
    parser.add_argument('--axis-lift-mm', type=float, default=0.0,
                        help='Compatibility option; target heights are already included in JSON')
    args = parser.parse_args()
    target_file = Path(args.json).expanduser()
    if not target_file.is_file():
        raise SystemExit(f'Target JSON does not exist: {target_file}')
    with target_file.open(encoding='utf-8') as stream:
        targets = json.load(stream)
    rclpy.init()
    node = TargetViewer(targets, args.axis_every, args.axis_length_mm)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
