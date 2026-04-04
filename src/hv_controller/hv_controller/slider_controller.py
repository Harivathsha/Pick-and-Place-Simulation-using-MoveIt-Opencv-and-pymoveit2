#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class SliderControl(Node):
    def __init__(self):
        super().__init__("slider_control")
        self.arm_pub_ = self.create_publisher(JointTrajectory, "arm_controller/joint_trajectory", 10)
        self.gripper_pub_ = self.create_publisher(JointTrajectory, "gripper_controller/joint_trajectory", 10)
        self.sub_ = self.create_subscription(JointState, "joint_commands", self.sliderCallback, 10)
        self.get_logger().info("Slider Control Node started")

    def sliderCallback(self, msg):
        # 1. Create a dictionary mapping current joint names to their slider positions
        current_positions = dict(zip(msg.name, msg.position))
        
        arm_controller = JointTrajectory()
        gripper_controller = JointTrajectory()
        
        arm_joints = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7"]
        gripper_joints = ["panda_finger_joint1", "panda_finger_joint2"]
        
        arm_controller.joint_names = arm_joints
        gripper_controller.joint_names = gripper_joints
        
        arm_goal = JointTrajectoryPoint()
        gripper_goal = JointTrajectoryPoint()
        
        # 2. Safely grab the arm positions by name, defaulting to 0.0 if not found
        arm_goal.positions = [current_positions.get(joint, 0.0) for joint in arm_joints]
        
        # 3. Safely grab the finger positions. If finger2 is missing from the GUI, we just copy finger1's value.
        f1 = current_positions.get("panda_finger_joint1", 0.0)
        f2 = current_positions.get("panda_finger_joint2", f1)
        gripper_goal.positions = [f1, f2]
        
        # 4. Add the required time_from_start
        point_time = Duration(sec=0, nanosec=100000000) 
        arm_goal.time_from_start = point_time
        gripper_goal.time_from_start = point_time
        
        arm_controller.points.append(arm_goal)
        gripper_controller.points.append(gripper_goal)
        
        self.arm_pub_.publish(arm_controller)
        self.gripper_pub_.publish(gripper_controller)

def main():
    rclpy.init()
    simple_publisher = SliderControl()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()