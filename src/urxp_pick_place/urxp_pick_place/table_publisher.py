"""
Publishes a static, textured table mesh in RViz for MoveIt scene context —
the real-hardware equivalent of the Fuel "Table" model used in Gazebo.
Robot (link_base) is at the back-center of the table; table extends in +X direction.

meshes/table.dae is authored true-to-scale in meters, centered on its own
origin: X (wide axis) = 1.21m, Y (depth axis) = 0.60m, Z = 0.02m thick,
top surface at local z=+0.01. Its wide axis is mesh-local X, but our
convention (matching pick_place_server's place positions) is depth along
+X and width along Y — so the mesh is yawed 90 degrees to match, same as
the flat-box marker this replaces used to sit.
"""

import math

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration


class TablePublisher(Node):
    def __init__(self):
        super().__init__('table_publisher')

        self.declare_parameter('table_depth_m', 0.60)
        self.declare_parameter('table_thickness_m', 0.02)
        self.declare_parameter('mesh_resource', 'package://urxp_pick_place/meshes/table.dae')

        self.depth = self.get_parameter('table_depth_m').value
        self.thickness = self.get_parameter('table_thickness_m').value
        self.mesh_resource = self.get_parameter('mesh_resource').value

        self.pub = self.create_publisher(Marker, '/urxp/table', 10)
        # Republish at 1 Hz so any RViz subscriber (any QoS) picks it up.
        self.create_timer(1.0, self._publish)
        self._publish()
        self.get_logger().info('Table mesh publishing on /urxp/table')

    def _publish(self):
        m = Marker()
        m.header.stamp.sec = 0  # 0 = use latest available TF, avoids timing jitter
        m.header.stamp.nanosec = 0
        m.header.frame_id = 'link_base'
        m.ns = 'table'
        m.id = 0
        m.type = Marker.MESH_RESOURCE
        m.mesh_resource = self.mesh_resource
        m.mesh_use_embedded_materials = True  # use table.dae's own wood texture
        m.action = Marker.ADD

        m.pose.position.x = self.depth / 2.0 - 0.06
        m.pose.position.y = 0.0
        m.pose.position.z = -self.thickness / 2.0  # top surface flush with z=0
        # Yaw 90 degrees: mesh-local X (1.21m wide) -> world Y, mesh-local Y (0.60m deep) -> world X.
        m.pose.orientation.z = math.sin(math.pi / 4.0)
        m.pose.orientation.w = math.cos(math.pi / 4.0)

        m.scale.x = 1.0  # mesh is already true-to-scale in meters
        m.scale.y = 1.0
        m.scale.z = 1.0

        # Fallback tint if mesh_use_embedded_materials ever fails to load.
        m.color.r = 0.85
        m.color.g = 0.70
        m.color.b = 0.48
        m.color.a = 1.0

        m.lifetime = Duration(sec=0, nanosec=0)  # never expire
        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = TablePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
