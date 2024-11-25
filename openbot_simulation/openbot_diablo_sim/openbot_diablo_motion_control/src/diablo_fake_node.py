#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from std_msgs.msg import String, Header
import threading
from motion_msgs.msg import MotionCtrl  # Import the message type


class JointStateSimulatorNode(Node):
    def __init__(self):
        super().__init__('joint_state_simulator')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        
        
        self.timer_tf = self.create_timer(0.01, self.publish_base_link_transform)  # Timer will be created dynamically
        self.current_transition_factor = 1.0
        self.target_transition_factor = 1.0
        self.transition_step = 0
        self.in_transition = False
        self.timer = self.create_timer(0.1, self.transition_step_callback)  # Timer will be created dynamically
        self.get_logger().info("Started joint_state_simulator.")
        
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriber to toggle motion state
        self.motion_cmd_sub = self.create_subscription(
            MotionCtrl,
            '/diablo/MotionCmd',
            self.toggle_transition,
            10
        )
        self.sitting_base_z = -0.228  # Height for sitting
        
        def sit_down_wrapper():
            self.stand_up(duration=3.0)

        # Create a timer to call the sit_down method after 1 second
        timer = threading.Timer(2.0, sit_down_wrapper)
        timer.start()
        
    def toggle_transition(self, msg):
        """Handle motion command to toggle between sit and stand states."""
        self.get_logger().warning(f"Received motion command: {msg}")
        if msg.mode.stand_mode:
            self.stand_up(duration=3.0)
        else:
            self.sit_down(duration=2.0)

    @property
    def base_z(self):
        """
        Calculate the vertical height (z) of the base_link relative to the transition factor.

        Returns:
            float: The interpolated base_link z-height.
        """
        # Interpolate between sitting and standing heights
        interpolated_base_z = self.sitting_base_z * self.current_transition_factor
        self.get_logger().debug(f"Calculated base_link z: {interpolated_base_z}")
        return interpolated_base_z

        
    def compute_joint_state(self, transition_factor):
      """
      Compute joint state positions based on a transition factor.

      Args:
          transition_factor (float): A value between 0 and 1 indicating the interpolation factor.

      Returns:
          list: An array representing joint_state_msg.positio
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
.
      """
      # Ensure transition_factor is within the valid range
      if not (0 <= transition_factor <= 1):
          raise ValueError("transition_factor must be between 0 and 1 inclusive.")
      
      # Define fixed and transition positions
      fixed_positions = [0.0, 0.0, 0.0, 0.0]  # Rev1, Rev2, Rev5, Rev8
      start_positions = [0, 0, 0, 0]          # Initial state for Rev3, Rev4, Rev6, Rev7
      end_positions = [1.35, -2.65, -1.35, 2.65]  # Final state for Rev3, Rev4, Rev6, Rev7
      
      # Interpolate positions for the transitioning joints
      transition_positions = [
          start + transition_factor * (end - start)
          for start, end in zip(start_positions, end_positions)
      ]
      
      # Construct the final joint state array
      joint_state_msg_position = fixed_positions[:2] + transition_positions[:2] + \
                                [fixed_positions[2]] + transition_positions[2:] + \
                                [fixed_positions[3]]
      
      return joint_state_msg_position

    def simulate_joint_state(self, transition_factor):
        """Simulate the robot's joint state based on the transition factor."""
        transition_factor = max(0.0, min(1.0, transition_factor))  # Clamp to [0, 1]
        self.get_logger().debug(f"Simulating joint state with transition factor: {transition_factor}")

        # Constants for sit and stand states
        sit_leg_length = 5  #  (10 cm)
        stand_leg_length = 25  #  (14 cm)

        # Interpolate leg length
        leg_length = sit_leg_length + (stand_leg_length - sit_leg_length) * transition_factor
        leg_length_in_dm = leg_length // 10 # Already in decimeters
        self.get_logger().debug(f"Interpolated leg length: {leg_length} dm")

        # Calculate angles
        try:
            numerator = (3.92 - (leg_length_in_dm * leg_length_in_dm))
            denominator = 3.92
            acos_input = numerator / denominator
            self.get_logger().debug(f"Numerator for acos: {numerator}, Denominator: {denominator}, acos Input: {acos_input}")

            if not -1 <= acos_input <= 1:
                self.get_logger().error(f"acos Input out of range: {acos_input}. Skipping angle calculation.")
                return  # Skip if invalid

            l_angleB = (3.14 - math.acos(acos_input)) / 2
            r_angleB = l_angleB  # Symmetric legs
            self.get_logger().debug(f"Calculated left angle B: {l_angleB}, right angle B: {r_angleB}")
        except ValueError as e:
            self.get_logger().error(f"Error in angle calculation: {e}")
            return

        # Interpolate joint angle compensation
        sit_joint_angle_compensate = 1.0
        stand_joint_angle_compensate = 1.9
        joint_angle_compensate = sit_joint_angle_compensate + \
                                 (stand_joint_angle_compensate - sit_joint_angle_compensate) * transition_factor
        self.get_logger().debug(f"Joint angle compensation: {joint_angle_compensate}")

        # Create joint state message
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ["Rev1", "Rev2", "Rev3", "Rev4", "Rev5", "Rev6", "Rev7", "Rev8"]
        joint_state_msg.position = [
            0.0,                                # Rev1 fixed
            0.0,                                # Rev2 fixed
            -l_angleB - joint_angle_compensate, # Rev3
            2 * l_angleB,                       # Rev4
            0.0,                                # Rev5 fixed
            r_angleB + joint_angle_compensate,  # Rev6
            -(2 * r_angleB),                    # Rev7
            0.0                                 # Rev8 fixed
        ]
        joint_state_msg.position = self.compute_joint_state(transition_factor)

        # Publish the joint state
        self.publisher.publish(joint_state_msg)
        self.publish_base_link_transform()
        self.get_logger().debug(f"Published joint state: {joint_state_msg.position}")
        
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

    def start_transition(self, target_factor, duration):
        """Start a transition to the given target factor over the specified duration."""
        if duration <= 0:
            self.get_logger().error("Invalid duration for transition. Must be > 0.")
            return

        # if self.timer:
        #     self.timer.cancel()
        #     self.timer = None

        self.target_transition_factor = target_factor
        step_count = duration * 10  # 10 Hz updates
        self.transition_step = (target_factor - self.current_transition_factor) / step_count
        self.in_transition = True

        # Create a timer to handle the transition
        # self.timer = self.create_timer(0.1, self.transition_step_callback)
        self.get_logger().info(f"Started transition to {target_factor} over {duration} seconds.")

    def transition_step_callback(self):
        """Update the transition factor and simulate joint states."""
        if self.in_transition:
          self.current_transition_factor += self.transition_step
          if (self.transition_step > 0 and self.current_transition_factor >= self.target_transition_factor) or \
            (self.transition_step < 0 and self.current_transition_factor <= self.target_transition_factor):
              self.current_transition_factor = self.target_transition_factor
              self.in_transition = False
              self.get_logger().info(f"Transition to {self.target_transition_factor} completed.")
          
        self.simulate_joint_state(self.current_transition_factor)

        # Stop the timer if we reach the target

    def sit_down(self, duration):
        """Start transitioning to the sit-down state."""
        self.get_logger().info(f"Transitioning to sit-down state over {duration} seconds.")
        self.start_transition(1.0, duration)

    def stand_up(self, duration):
        """Start transitioning to the stand-up state."""
        self.get_logger().info(f"Transitioning to stand-up state over {duration} seconds.")
        self.start_transition(0.0, duration)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateSimulatorNode()

    try:  
        # Set logging level to DEBUG for more information
        # node.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        # Example transitions
        # node.get_logger().info('Starting initial sit-down transition...')
        # node.stand_up(duration=3.0)  # Transition to sit-down state over 5 seconds

        # Use rclpy.spin to keep the node running
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('Node stopped by user.')

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
