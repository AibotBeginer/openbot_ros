#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Header
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class DiabloMotionController(Node):
    def __init__(self):
        super().__init__('diablo_motion_controller')

        # Publishers
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriber to toggle motion state
        self.motion_cmd_sub = self.create_subscription(
            String,
            '/diablo/MotionCmd',
            self.toggle_motion,
            10
        )

        # Timer for periodic updates at 10 Hz
        self.timer = self.create_timer(0.1, self.update_motion)

        # Motion state
        self.is_sitting = True  # Start in sitting position
        self.motion_in_progress = False  # True when a motion is ongoing

        # Initial and target positions for joints and base_link
        self.joint_angles = {
            'Rev6': -1.43,
            'Rev7': 2.87,
            'Rev3': 1.43,
            'Rev4': -2.87
        }
        self.base_z = 0.0  # Initial height of base_link
        self.target_angles = self.joint_angles.copy()  # Initialize targets
        self.target_base_z = self.base_z

        # Standing position
        self.standing_angles = {
            'Rev6': 0.0,
            'Rev7': 0.0,
            'Rev3': 0.0,
            'Rev4': 0.0
        }
        self.standing_base_z = 0.5  # Height for standing

        # Sitting position
        self.sitting_angles = {
            'Rev6': -1.43,
            'Rev7': 2.87,
            'Rev3': 1.43,
            'Rev4': -2.87
        }
        self.sitting_base_z = 0.0  # Height for sitting

        # Motion duration
        self.motion_duration = 5.0  # Duration of motion in seconds
        self.start_time = None  # Initialize start time

        self.get_logger().info("Diablo Motion Controller node initialized.")

    def toggle_motion(self, msg):
        """Toggle the motion state between standing and sitting."""
        if not self.motion_in_progress:
            if self.is_sitting:
                self.get_logger().info("Switching to Stand-Up Motion.")
                self.target_angles = self.standing_angles
                self.target_base_z = self.standing_base_z
            else:
                self.get_logger().info("Switching to Sit-Down Motion.")
                self.target_angles = self.sitting_angles
                self.target_base_z = self.sitting_base_z

            self.is_sitting = not self.is_sitting  # Toggle state
            self.motion_in_progress = True
            self.start_time = self.get_clock().now()
            self.get_logger().info("Motion started.")

    def interpolate(self, start, end, t, duration):
        """Linear interpolation function."""
        if t >= duration:
            return end
        return start + (end - start) * (t / duration)

    def update_motion(self):
        """Update joint states and base_link transform."""
        if self.motion_in_progress:
            now = self.get_clock().now()
            elapsed_time = (now.nanosec * 1e-9 + now.sec) - (self.start_time.nanosec * 1e-9 + self.start_time.sec)

            # Interpolate joint positions
            for joint, target in self.target_angles.items():
                current = self.joint_angles[joint]
                self.joint_angles[joint] = self.interpolate(current, target, elapsed_time, self.motion_duration)

            # Interpolate base_link height
            self.base_z = self.interpolate(self.base_z, self.target_base_z, elapsed_time, self.motion_duration)

            # Stop updating after the motion is complete
            if elapsed_time >= self.motion_duration:
                self.motion_in_progress = False
                self.get_logger().info("Motion completed.")

        # Always publish joint states and transform at consistent frequency
        self.publish_joint_states()
        # self.publish_base_link_transform()

    def publish_joint_states(self):
        """Publish the joint states."""
        joint_state_msg = JointState()
        joint_state_msg.header = Header()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ['Rev6', 'Rev7', 'Rev3', 'Rev4']
        joint_state_msg.position = [
            self.joint_angles['Rev6'],
            self.joint_angles['Rev7'],
            self.joint_angles['Rev3'],
            self.joint_angles['Rev4']
        ]
        self.joint_pub.publish(joint_state_msg)
        self.get_logger().debug(f"Published joint states: {joint_state_msg}")

    def publish_base_link_transform(self):
        """Publish the base_link transform."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_footprint'  # Assuming world is the fixed frame
        t.child_frame_id = 'base_link'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = self.base_z
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().debug("Published base_link transform.")


def main(args=None):
    rclpy.init(args=args)
    node = DiabloMotionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
