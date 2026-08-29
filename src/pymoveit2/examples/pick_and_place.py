#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pymoveit2 import MoveIt2
from pymoveit2.robots import panda
import math

import time

class PickAndPlace(Node):
    def __init__(self):
        super().__init__("pick_and_place")

        # Parameters
        self.declare_parameter("target_color", "R")
        self.target_color = self.get_parameter("target_color").value.upper()

        self.declare_parameter("approach_offset", 0.31)
        self.approach_offset = float(self.get_parameter("approach_offset").value)

        # Target Storage
        self.target_coords = None  

        # Arm MoveIt2 interface
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=panda.joint_names(),
            base_link_name=panda.base_link_name(),
            end_effector_name=panda.end_effector_name(),
            group_name=panda.MOVE_GROUP_ARM,
        )
        self.moveit2.max_velocity = 0.1
        self.moveit2.max_acceleration = 0.1

        # Gripper MoveIt2 interface (Treated as a standard group to avoid Action errors)
        self.gripper = MoveIt2(
            node=self,
            joint_names=panda.gripper_joint_names(),
            base_link_name=panda.base_link_name(),
            end_effector_name=panda.end_effector_name(),
            group_name=panda.MOVE_GROUP_GRIPPER,
        )
        self.gripper.max_velocity = 0.5
        self.gripper.max_acceleration = 0.5

        # Subscriber
        self.sub = self.create_subscription(
            String, "/color_coordinates", self.coords_callback, 10
        )

        # Predefined joint positions (in radians)
        # Change the 4th number (index 3) to match our safe -1.5 angle
        self.start_joints = [0.0, 0.0, 0.0, -1.5, 0.0, 0.0, math.radians(-125.0)]
        self.home_joints  = [0.0, 0.0, 0.0, math.radians(-90.0), 0.0, math.radians(92.0), math.radians(50.0)]
        self.drop_joints  = [math.radians(-155.0), math.radians(30.0), math.radians(-20.0),
                             math.radians(-124.0), math.radians(44.0), math.radians(163.0), math.radians(7.0)]
        # Define safe gripper limits (slightly inside the 0.0 to 0.04 absolute limits)
        # Define safe gripper limits
        self.gripper_open = [0.035, 0.035]
        self.gripper_closed = [0.002, 0.002]

    def coords_callback(self, msg):
        # Once we lock on, ignore all future camera messages
        if self.target_coords is not None:
            return  

        try:
            color_id, x, y, z = msg.data.split(",")
            color_id = color_id.strip().upper()

            if color_id == self.target_color:
                # Lock coordinates
                self.target_coords = [float(x), float(y), float(z)]
                self.get_logger().info(
                    f"Target {self.target_color} locked at: "
                    f"[{self.target_coords[0]:.3f}, {self.target_coords[1]:.3f}, {self.target_coords[2]:.3f}]"
                )
        except Exception as e:
            self.get_logger().error(f"Error parsing /color_coordinates: {e}")

    def execute_sequence(self):
        pick_position = [self.target_coords[0], self.target_coords[1], self.target_coords[2] - 0.60]
        quat_xyzw = [0.0, 1.0, 0.0, 0.0]

        # 1. Move to home joint configuration
        self.get_logger().info("Moving to Home...")
        self.moveit2.move_to_configuration(self.home_joints)
        self.moveit2.wait_until_executed()

        # 2. Move above target
        self.get_logger().info("Moving above target...")
        self.moveit2.move_to_pose(position=pick_position, quat_xyzw=quat_xyzw)
        self.moveit2.wait_until_executed()

        # 3. Open gripper
        self.gripper.move_to_configuration(self.gripper_open)
        self.gripper.wait_until_executed()

        # 4. Move down to approach object
        self.get_logger().info("Approaching target...")
        approach_position = [pick_position[0], pick_position[1], pick_position[2] - self.approach_offset]
        self.moveit2.move_to_pose(position=approach_position, quat_xyzw=quat_xyzw, cartesian=True)
        self.moveit2.wait_until_executed()

        # 5. Close gripper
        # 5. Close gripper
        self.get_logger().info("Closing gripper...")
        self.gripper.move_to_configuration(self.gripper_closed)
        
        # DELETE THIS LINE:
        # self.gripper.wait_until_executed() 
        
        # ADD THIS LINE INSTEAD:
        time.sleep(2.0)
        # 6. Lift back up
        self.get_logger().info("Lifting...")
        self.moveit2.move_to_pose(position=pick_position, quat_xyzw=quat_xyzw, cartesian=True)
        self.moveit2.wait_until_executed()

        # 7. Move to drop joint configuration
        self.get_logger().info("Moving to Drop Zone...")
        self.moveit2.move_to_configuration(self.drop_joints)
        self.moveit2.wait_until_executed()

        # 8. Open gripper to release
        self.gripper.move_to_configuration(self.gripper_open)
        self.gripper.wait_until_executed()

        # 9. Return to start
        self.get_logger().info("Returning to Start...")
        self.moveit2.move_to_configuration(self.start_joints)
        self.moveit2.wait_until_executed()

        self.get_logger().info("Pick-and-place sequence completely finished!")


def main():
    rclpy.init()
    node = PickAndPlace()

    # Move to starting pose before listening to the camera
    node.get_logger().info("Initializing: Moving to start position...")
    node.moveit2.move_to_configuration(node.start_joints)
    node.moveit2.wait_until_executed()

    node.get_logger().info(f"Waiting for {node.target_color} from /color_coordinates...")

    # Safely spin the node in the main thread until the camera finds the target
    while rclpy.ok() and node.target_coords is None:
        rclpy.spin_once(node)

    # Now that we have coordinates and the node is paused, execute the heavy lifting!
    if rclpy.ok() and node.target_coords is not None:
        node.execute_sequence()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()