"""
pose_adapter — Normalizes vehicle pose from Gazebo into a stable ROS 2 topic.

Subscribes: /terrain_mapping/vehicle/pose/raw  (geometry_msgs/PoseStamped, from ros_gz_bridge)
Publishes:  /terrain_mapping/vehicle/pose      (geometry_msgs/PoseStamped, normalized)

Frame convention:
  - Gazebo uses ENU, Z-up.  ROS planning convention is also ENU, Z-up for map-frame.
  - This adapter sets the ROS frame_id to 'map' (world-fixed) and child_frame_id to 'base_link'.
  - No coordinate transformation is needed since both Gazebo Harmonic and ROS use ENU.
  - If your Gazebo world uses a different convention, add the transform here.

The adapter also optionally publishes a TF transform for downstream nodes.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster


class PoseAdapter(Node):
    """Adapts raw Gazebo pose into a normalized ROS pose topic."""

    def __init__(self):
        super().__init__('pose_adapter')

        # Configurable frame IDs
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('publish_tf', True)

        self._frame_id = self.get_parameter('frame_id').value
        self._child_frame_id = self.get_parameter('child_frame_id').value
        self._publish_tf = self.get_parameter('publish_tf').value

        self._subscription = self.create_subscription(
            PoseStamped,
            'vehicle/pose/raw',       # relative — remapped by launch
            self._pose_callback,
            10,
        )

        self._publisher = self.create_publisher(
            PoseStamped,
            'vehicle/pose',            # relative — remapped by launch
            10,
        )

        if self._publish_tf:
            self._tf_broadcaster = TransformBroadcaster(self)
        else:
            self._tf_broadcaster = None

        self.get_logger().info(
            f'PoseAdapter started: frame_id={self._frame_id}, '
            f'child_frame_id={self._child_frame_id}, publish_tf={self._publish_tf}'
        )

    def _pose_callback(self, msg: PoseStamped):
        """Re-publish pose with normalized frame_id."""
        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._frame_id
        out.pose = msg.pose

        self._publisher.publish(out)

        if self._tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = self._frame_id
            t.child_frame_id = self._child_frame_id
            t.transform.translation.x = msg.pose.position.x
            t.transform.translation.y = msg.pose.position.y
            t.transform.translation.z = msg.pose.position.z
            t.transform.rotation.x = msg.pose.orientation.x
            t.transform.rotation.y = msg.pose.orientation.y
            t.transform.rotation.z = msg.pose.orientation.z
            t.transform.rotation.w = msg.pose.orientation.w
            self._tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = PoseAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
