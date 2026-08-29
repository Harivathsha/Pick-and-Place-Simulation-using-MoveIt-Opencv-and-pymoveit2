#!/usr/bin/env python3

import math
from typing import Sequence

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from moveit_msgs.srv import (
    ApplyPlanningScene,
    GetCartesianPath,
    GetPlanningScene,
)
from std_msgs.msg import Empty, String
from trajectory_msgs.msg import JointTrajectoryPoint

from pymoveit2 import MoveIt2
from pymoveit2.robots import panda
import time

from geometry_msgs.msg import Pose
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    MoveItErrorCodes,
    PlanningScene,
    PlanningSceneComponents,
)

class BottleShakeTask(Node):

    def __init__(self):
        super().__init__("bottle_shake_task")

        # ---------------- ROS parameters ----------------

        self.declare_parameter("target_color", "R")
        self.declare_parameter("approach_offset", 0.12)
        self.declare_parameter("cartesian_fraction_threshold", 0.98)
        # Conservative MoveIt collision geometry, expressed in panda_link0.
        self.declare_parameter("table_size", [0.80, 1.20, 0.10])
        self.declare_parameter("table_position", [0.60, 0.00, -0.10])
        self.declare_parameter(
            "bottle_center_z",
            0.04,
        )

        self.declare_parameter(
            "workspace_x_limits",
            [0.40, 0.75],
        )

        self.declare_parameter(
            "workspace_y_limits",
            [-0.30, 0.30],
        )
        self.declare_parameter("bottle_height", 0.20)
        self.declare_parameter("bottle_radius", 0.035)
        self.declare_parameter(
            "shake_horizontal_amplitude",
            0.05,
        )
        self.declare_parameter(
            "shake_vertical_amplitude",
            0.035,
        )
        self.declare_parameter("shake_cycles", 4)
        self.declare_parameter("shake_samples_per_cycle", 32)
        self.declare_parameter("shake_velocity", 0.30)
        self.declare_parameter("shake_acceleration", 0.40)

        self.declare_parameter("place_clearance", 0.06)
        self.declare_parameter("place_velocity", 0.05)
        self.declare_parameter("place_acceleration", 0.05)
        self.target_color = (
            self.get_parameter("target_color").value.strip().upper()
        )
        self.approach_offset = float(
            self.get_parameter("approach_offset").value
        )
        self.cartesian_fraction_threshold = float(
            self.get_parameter("cartesian_fraction_threshold").value
        )
        self.table_size = list(
        self.get_parameter("table_size").value
        )
        self.table_position = list(
            self.get_parameter("table_position").value
        )
        self.bottle_center_z = float(
            self.get_parameter(
                "bottle_center_z"
            ).value
        )

        self.workspace_x_limits = list(
            self.get_parameter(
                "workspace_x_limits"
            ).value
        )

        self.workspace_y_limits = list(
            self.get_parameter(
                "workspace_y_limits"
            ).value
        )
        self.bottle_height = float(
            self.get_parameter("bottle_height").value
        )
        self.bottle_radius = float(
            self.get_parameter("bottle_radius").value
        )
        self.shake_horizontal_amplitude = float(
            self.get_parameter(
                "shake_horizontal_amplitude"
            ).value
        )

        self.shake_vertical_amplitude = float(
            self.get_parameter(
                "shake_vertical_amplitude"
            ).value
        )

        self.shake_cycles = int(
            self.get_parameter("shake_cycles").value
        )
        self.shake_samples_per_cycle = int(
            self.get_parameter(
                "shake_samples_per_cycle"
            ).value
        )
        self.shake_velocity = float(
            self.get_parameter("shake_velocity").value
        )

        self.shake_acceleration = float(
            self.get_parameter("shake_acceleration").value
        )

        self.place_clearance = float(
            self.get_parameter("place_clearance").value
        )

        self.place_velocity = float(
            self.get_parameter("place_velocity").value
        )

        self.place_acceleration = float(
            self.get_parameter("place_acceleration").value
        )
        self.table_collision_id = "work_table"
        self.bottle_collision_id = "target_bottle"
        # Filled once the selected colour has been detected.
        self.target_coords = None
        # The vision-derived bottle pose that will also be reused
        # when returning the bottle to its original location.
        self.original_bottle_position = None
        # ---------------- MoveIt arm interface ----------------

        self.arm = MoveIt2(
            node=self,
            joint_names=panda.joint_names(),
            base_link_name=panda.base_link_name(),
            end_effector_name=panda.end_effector_name(),
            group_name=panda.MOVE_GROUP_ARM,
            ignore_new_calls_while_executing=True,
        )

        # These are scaling factors, not direct rad/s values.
        self.arm.max_velocity = 0.10
        self.arm.max_acceleration = 0.10

        # RRTConnect is suitable for fast free-space planning.
        self.arm.planner_id = "RRTConnectkConfigDefault"

        # Cartesian motions should still perform collision checks.
        self.arm.cartesian_avoid_collisions = True

        # Zero keeps the repository's existing jump-threshold behaviour.
        self.arm.cartesian_jump_threshold = 0.0

        # ---------------- Direct gripper controller ----------------

        self.gripper_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/gripper_controller/follow_joint_trajectory",
        )

        self.gripper_joint_names = panda.gripper_joint_names()
        self.gripper_open = [0.040, 0.040]
        self.gripper_closed = [0.002, 0.002]
        self.gripper_grasp = [0.035, 0.035]

        self.attachment_state = "unknown"

        self.side_grasp_depth = 0.105
        self.lift_distance = 0.35
        # ---------------- Target subscription ----------------

        self.coordinate_subscription = self.create_subscription(
            String,
            "/color_coordinates",
            self.coordinates_callback,
            10,
        )

        # ---------------- Known safe configurations ----------------

        self.start_joints = [
            0.0,
            0.0,
            0.0,
            -1.5,
            0.0,
            0.0,
            math.radians(-125.0),
        ]

        self.home_joints = [
            0.0,
            0.0,
            0.0,
            math.radians(-90.0),
            0.0,
            math.radians(92.0),
            math.radians(50.0),
        ]

        self.drop_joints = [
            math.radians(-155.0),
            math.radians(30.0),
            math.radians(-20.0),
            math.radians(-124.0),
            math.radians(44.0),
            math.radians(163.0),
            math.radians(7.0),
        ]
        self.attach_publisher = self.create_publisher(
            Empty,
            "/bottle/attach",
            10,
        )
        self.detach_publisher = self.create_publisher(
            Empty,
            "/bottle/detach",
            10,
        )
        self.attachment_state_subscription = self.create_subscription(
            String,
            "/bottle/attachment_state",
            self.attachment_state_callback,
            10,
        )
        self.planning_scene_client = self.create_client(
            GetPlanningScene,
            "/get_planning_scene",
        )
        self.apply_planning_scene_client = self.create_client(
            ApplyPlanningScene,
            "/apply_planning_scene",
        )

        self.cartesian_path_client = self.create_client(
            GetCartesianPath,
            "/compute_cartesian_path",
        )
        self.bottle_touch_links = [
            "panda_hand",
            "panda_leftfinger",
            "panda_rightfinger",
        ]
        self.grasp_contact_links = [
            "panda_leftfinger",
            "panda_rightfinger",
        ]
    # ==============================================================
    # Perception callback
    # ==============================================================

    def coordinates_callback(self, message):
        # Lock the first valid target and ignore later camera updates.
        if self.target_coords is not None:
            return

        try:
            fields = [field.strip() for field in message.data.split(",")]

            if len(fields) != 4:
                raise ValueError(
                    "Expected: color,x,y,z"
                )

            color_id = fields[0].upper()

            if color_id != self.target_color:
                return

            self.target_coords = [
                float(fields[1]),
                float(fields[2]),
                float(fields[3]),
            ]

            self.get_logger().info(
                f"Locked target {color_id}: "
                f"x={self.target_coords[0]:.3f}, "
                f"y={self.target_coords[1]:.3f}, "
                f"z={self.target_coords[2]:.3f}"
            )

        except (ValueError, IndexError) as error:
            self.get_logger().error(
                f"Invalid /color_coordinates message: {error}"
            )
    # ==============================================================
    # Gazebo and MoveIt attachment helpers
    # ==============================================================
    def validate_detected_target(self) -> bool:
        """Check that the camera target is finite and safely reachable."""

        if self.target_coords is None:
            self.get_logger().error(
                "[TARGET REJECTED] No camera target is available"
            )
            return False

        if len(self.target_coords) != 3:
            self.get_logger().error(
                "[TARGET REJECTED] Expected x, y and z"
            )
            return False

        if (
            len(self.workspace_x_limits) != 2
            or len(self.workspace_y_limits) != 2
        ):
            self.get_logger().error(
                "[TARGET REJECTED] Workspace limits are invalid"
            )
            return False

        x_position = float(self.target_coords[0])
        y_position = float(self.target_coords[1])
        z_position = float(self.target_coords[2])

        if not all(
            math.isfinite(value)
            for value in (
                x_position,
                y_position,
                z_position,
            )
        ):
            self.get_logger().error(
                "[TARGET REJECTED] Coordinates are not finite"
            )
            return False

        minimum_x, maximum_x = self.workspace_x_limits
        minimum_y, maximum_y = self.workspace_y_limits

        if not minimum_x <= x_position <= maximum_x:
            self.get_logger().error(
                f"[TARGET REJECTED] x={x_position:.3f} "
                f"is outside [{minimum_x:.3f}, {maximum_x:.3f}]"
            )
            return False

        if not minimum_y <= y_position <= maximum_y:
            self.get_logger().error(
                f"[TARGET REJECTED] y={y_position:.3f} "
                f"is outside [{minimum_y:.3f}, {maximum_y:.3f}]"
            )
            return False

        self.get_logger().info(
            "[TARGET ACCEPTED] "
            f"x={x_position:.3f}, "
            f"y={y_position:.3f}, "
            f"camera_z={z_position:.3f}"
        )

        return True
    def attachment_state_callback(self, message: String) -> None:
        """Receive the real Gazebo attachment state."""

        state = message.data.strip().lower()

        if state not in ("attached", "detached"):
            self.get_logger().warn(
                f"Unknown bottle attachment state: '{message.data}'"
            )
            return

        self.attachment_state = state

        self.get_logger().info(
            f"[GAZEBO STATE] Bottle is {state}"
        )


    def command_gazebo_attachment(
        self,
        target_state: str,
        timeout_seconds: float = 6.0,
    ) -> bool:
        """Request attachment/detachment and wait for Gazebo feedback."""

        if target_state == "attached":
            publisher = self.attach_publisher
            command_topic = "/bottle/attach"

        elif target_state == "detached":
            publisher = self.detach_publisher
            command_topic = "/bottle/detach"

        else:
            raise ValueError(
                "target_state must be 'attached' or 'detached'"
            )

        self.attachment_state = "unknown"

        deadline = time.monotonic() + timeout_seconds
        next_publish_time = time.monotonic()
        command_was_published = False

        self.get_logger().info(
            f"[GAZEBO REQUEST] Waiting for '{target_state}'"
        )

        while rclpy.ok() and time.monotonic() < deadline:
            current_time = time.monotonic()

            # Wait until the ROS-Gazebo bridge is listening.
            if (
                publisher.get_subscription_count() > 0
                and current_time >= next_publish_time
            ):
                publisher.publish(Empty())
                command_was_published = True

                # Retry every half-second until feedback arrives.
                next_publish_time = current_time + 0.5

                self.get_logger().info(
                    f"[GAZEBO COMMAND] Published {command_topic}"
                )

            rclpy.spin_once(self, timeout_sec=0.1)

            if self.attachment_state == target_state:
                self.get_logger().info(
                    f"[GAZEBO CONFIRMED] Bottle is {target_state}"
                )
                return True

        if not command_was_published:
            self.get_logger().error(
                f"[FAILED] No bridge subscriber for {command_topic}"
            )
        else:
            self.get_logger().error(
                f"[FAILED] Gazebo did not confirm '{target_state}'"
            )

        return False


    def wait_for_moveit_bottle_attachment(
        self,
        timeout_seconds: float = 5.0,
    ) -> bool:
        """Confirm that MoveIt transferred the bottle into robot state."""

        if not self.planning_scene_client.wait_for_service(
            timeout_sec=3.0
        ):
            self.get_logger().error(
                "[FAILED] /get_planning_scene is unavailable"
            )
            return False

        deadline = time.monotonic() + timeout_seconds

        component_mask = (
            PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
            | PlanningSceneComponents.WORLD_OBJECT_NAMES
        )

        while rclpy.ok() and time.monotonic() < deadline:
            request = GetPlanningScene.Request()
            request.components.components = component_mask

            future = self.planning_scene_client.call_async(request)

            rclpy.spin_until_future_complete(
                self,
                future,
                timeout_sec=1.0,
            )

            if not future.done():
                self.planning_scene_client.remove_pending_request(
                    future
                )
                continue

            try:
                response = future.result()
            except Exception as error:
                self.get_logger().warn(
                    f"Planning Scene query failed: {error}"
                )
                continue

            if response is None:
                continue

            attached_ids = {
                attached_object.object.id
                for attached_object
                in response.scene.robot_state.attached_collision_objects
            }

            world_ids = {
                collision_object.id
                for collision_object
                in response.scene.world.collision_objects
            }

            correctly_attached = (
                self.bottle_collision_id in attached_ids
                and self.bottle_collision_id not in world_ids
            )

            if correctly_attached:
                self.get_logger().info(
                    "[MOVEIT CONFIRMED] 'target_bottle' is attached "
                    "to the robot and removed from the world"
                )
                return True

            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().error(
            "[FAILED] MoveIt did not confirm bottle attachment"
        )
        return False

    def wait_for_moveit_bottle_detachment(
        self,
        timeout_seconds: float = 5.0,
    ) -> bool:
        """Confirm the bottle returned from robot state to world state."""

        if not self.planning_scene_client.wait_for_service(
            timeout_sec=3.0
        ):
            self.get_logger().error(
                "[FAILED] /get_planning_scene is unavailable"
            )
            return False

        deadline = time.monotonic() + timeout_seconds

        component_mask = (
            PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
            | PlanningSceneComponents.WORLD_OBJECT_NAMES
        )

        while rclpy.ok() and time.monotonic() < deadline:

            request = GetPlanningScene.Request()
            request.components.components = component_mask

            future = self.planning_scene_client.call_async(request)

            rclpy.spin_until_future_complete(
                self,
                future,
                timeout_sec=1.0,
            )

            if not future.done():
                self.planning_scene_client.remove_pending_request(
                    future
                )
                continue

            try:
                response = future.result()
            except Exception as error:
                self.get_logger().warn(
                    f"Planning Scene query failed: {error}"
                )
                continue

            if response is None:
                continue

            attached_ids = {
                attached_object.object.id
                for attached_object
                in response.scene.robot_state.attached_collision_objects
            }

            world_ids = {
                collision_object.id
                for collision_object
                in response.scene.world.collision_objects
            }

            correctly_detached = (
                self.bottle_collision_id not in attached_ids
                and self.bottle_collision_id in world_ids
            )

            if correctly_detached:
                self.get_logger().info(
                    "[MOVEIT CONFIRMED] Bottle is detached "
                    "and present in the world"
                )
                return True

            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().error(
            "[FAILED] MoveIt did not confirm bottle detachment"
        )
        return False

    
    def attach_bottle_for_transport(self) -> bool:
        """Synchronise the Gazebo and MoveIt bottle representations."""

        # First attach the real simulated bottle.
        if not self.command_gazebo_attachment("attached"):
            self.get_logger().error(
                "[ATTACH FAILED] Lift is forbidden"
            )
            return False

        # Wait until move_group listens for attachment updates.
        deadline = time.monotonic() + 5.0

        while (
            rclpy.ok()
            and self.count_subscribers(
                "/attached_collision_object"
            ) == 0
        ):
            if time.monotonic() >= deadline:
                self.get_logger().error(
                    "[FAILED] move_group is not listening on "
                    "/attached_collision_object"
                )
                return False

            rclpy.spin_once(self, timeout_sec=0.1)

        # Transfer the existing world object into robot state.
        self.arm.attach_collision_object(
            id=self.bottle_collision_id,
            link_name=panda.end_effector_name(),
            touch_links=self.bottle_touch_links,
        )

        self.get_logger().info(
            "[MOVEIT REQUEST] Attaching 'target_bottle' "
            "to 'panda_hand'"
        )

        if not self.wait_for_moveit_bottle_attachment():
            self.get_logger().error(
                "[ATTACH FAILED] Gazebo attached the bottle, "
                "but MoveIt did not; lift is forbidden"
            )
            return False

        self.get_logger().info(
            "[ATTACHMENT READY] Gazebo and MoveIt agree"
        )
        return True

    def detach_bottle_after_transport(self) -> bool:
        """Detach the bottle safely in both MoveIt and Gazebo."""

        self.arm.detach_collision_object(
            id=self.bottle_collision_id,
        )

        self.get_logger().info(
            "[MOVEIT REQUEST] Detaching 'target_bottle'"
        )

        if not self.wait_for_moveit_bottle_detachment():
            return False

        if not self.command_gazebo_attachment("detached"):
            return False

        self.get_logger().info(
            "[DETACHMENT READY] Gazebo and MoveIt agree"
        )
        return True

    def place_bottle_gently(
        self,
        grasp_position: Sequence[float],
        retreat_position: Sequence[float],
        orientation_xyzw: Sequence[float],
    ) -> bool:
        """Return the bottle to its original table pose and release it."""

        place_above_position = list(grasp_position)
        place_above_position[2] += self.place_clearance

        previous_velocity = self.arm.max_velocity
        previous_acceleration = self.arm.max_acceleration

        try:
            # Descend most of the 0.35 m lift at normal transport speed.
            self.arm.max_velocity = 0.10
            self.arm.max_acceleration = 0.10

            if not self.move_arm_to_pose(
                "PLACE ABOVE TABLE",
                place_above_position,
                orientation_xyzw,
                cartesian=True,
            ):
                return False

            # The bottle will touch the table during the final descent.
            if not self.set_bottle_contact_permission(
                [self.table_collision_id],
                allowed=True,
            ):
                return False

            # Only the final 6 cm is performed very slowly.
            self.arm.max_velocity = self.place_velocity
            self.arm.max_acceleration = self.place_acceleration

            if not self.move_arm_to_pose(
                "GENTLE PLACE",
                grasp_position,
                orientation_xyzw,
                cartesian=True,
            ):
                return False

            self.get_logger().info(
                "[PLACE CONTACT] Bottle returned to original grasp pose"
            )

            # Briefly hold the bottle still on the table.
            rclpy.spin_once(self, timeout_sec=0.5)

            if not self.detach_bottle_after_transport():
                return False

            if not self.command_gripper(
                "RELEASE BOTTLE",
                self.gripper_open,
                duration_seconds=1.5,
            ):
                return False

            # Move horizontally away along the original approach path.
            self.arm.max_velocity = 0.08
            self.arm.max_acceleration = 0.08

            if not self.move_arm_to_pose(
                "RETREAT AFTER PLACE",
                retreat_position,
                orientation_xyzw,
                cartesian=True,
            ):
                return False

        finally:
            self.arm.max_velocity = previous_velocity
            self.arm.max_acceleration = previous_acceleration

        # Contact is no longer needed after the fingers retreat.
        if not self.set_bottle_contact_permission(
            self.grasp_contact_links,
            allowed=False,
        ):
            return False

        self.get_logger().info(
            "[PLACE COMPLETE] Bottle released safely on table"
        )
        return True
    # ==============================================================
    # MoveIt Planning Scene helpers
    # ==============================================================

    def wait_for_planning_scene_subscriber(
        self,
        timeout_seconds: float = 5.0,
    ) -> bool:
        """Wait until move_group subscribes to /collision_object."""

        deadline = time.monotonic() + timeout_seconds

        while rclpy.ok():
            if self.count_subscribers("/collision_object") > 0:
                return True

            if time.monotonic() >= deadline:
                self.get_logger().error(
                    "[SCENE FAILED] move_group is not listening on "
                    "/collision_object"
                )
                return False

            rclpy.spin_once(self, timeout_sec=0.1)

        return False


    def add_table_collision(self) -> None:
        """Represent the work surface as a conservative collision box."""

        self.arm.add_collision_box(
            id=self.table_collision_id,
            size=self.table_size,
            position=self.table_position,
            quat_xyzw=[0.0, 0.0, 0.0, 1.0],
            frame_id=panda.base_link_name(),
        )

        self.get_logger().info(
            f"[SCENE] Published table '{self.table_collision_id}' "
            f"at {self.table_position} with size {self.table_size}"
        )


    def add_bottle_collision(
        self,
        position: Sequence[float],
    ) -> None:
        """Represent the upright bottle as a collision cylinder."""

        if len(position) != 3:
            raise ValueError(
                "Bottle position must contain x, y, and z"
            )

        self.arm.add_collision_cylinder(
            id=self.bottle_collision_id,
            height=self.bottle_height,
            radius=self.bottle_radius,
            position=list(position),
            quat_xyzw=[0.0, 0.0, 0.0, 1.0],
            frame_id=panda.base_link_name(),
        )

        self.get_logger().info(
            f"[SCENE] Published bottle '{self.bottle_collision_id}' "
            f"at {list(position)}"
        )

    def wait_for_world_bottle_pose(
        self,
        expected_position: Sequence[float],
        timeout_seconds: float = 5.0,
    ) -> bool:
        """Confirm MoveIt stored the bottle at the detected pose."""

        if not self.planning_scene_client.wait_for_service(
            timeout_sec=3.0
        ):
            self.get_logger().error(
                "[SCENE FAILED] /get_planning_scene unavailable"
            )
            return False

        deadline = time.monotonic() + timeout_seconds

        component_mask = (
            PlanningSceneComponents.WORLD_OBJECT_NAMES
            | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        )

        while rclpy.ok() and time.monotonic() < deadline:

            request = GetPlanningScene.Request()
            request.components.components = component_mask

            future = self.planning_scene_client.call_async(
                request
            )

            rclpy.spin_until_future_complete(
                self,
                future,
                timeout_sec=1.0,
            )

            if not future.done():
                self.planning_scene_client.remove_pending_request(
                    future
                )
                continue

            try:
                response = future.result()
            except Exception as error:
                self.get_logger().warn(
                    f"Planning Scene query failed: {error}"
                )
                continue

            if response is None:
                continue

            for collision_object in (
                response.scene.world.collision_objects
            ):
                if collision_object.id != self.bottle_collision_id:
                    continue

                if len(collision_object.primitives) == 0:
                    self.get_logger().warn(
                        "[SCENE] target_bottle exists but has no geometry"
                    )
                    continue

                object_pose = collision_object.pose

                stored_position = [
                    object_pose.position.x,
                    object_pose.position.y,
                    object_pose.position.z,
                ]

                self.get_logger().info(
                    "[SCENE CONFIRMED] MoveIt contains "
                    f"'{self.bottle_collision_id}' | "
                    f"published in '{panda.base_link_name()}' "
                    f"at {list(expected_position)} | "
                    f"stored in '{collision_object.header.frame_id}' "
                    f"at {stored_position}"
                )

                return True

            rclpy.spin_once(
                self,
                timeout_sec=0.1,
            )

        self.get_logger().error(
            "[SCENE FAILED] MoveIt did not confirm "
            "the vision-derived bottle pose"
        )
        return False

    def synchronize_bottle_with_vision(self) -> bool:
        """Create the MoveIt bottle at the camera-detected position."""

        if not self.validate_detected_target():
            return False

        detected_position = [
            float(self.target_coords[0]),
            float(self.target_coords[1]),
            self.bottle_center_z,
        ]

        self.get_logger().info(
            "[VISION -> SCENE] Publishing bottle at "
            f"{detected_position}"
        )

        self.add_bottle_collision(
            detected_position
        )

        if not self.wait_for_world_bottle_pose(
            detected_position
        ):
            return False

        self.original_bottle_position = (
            detected_position.copy()
        )

        self.get_logger().info(
            "[DYNAMIC TARGET READY] Camera and MoveIt agree"
        )

        return True

    
    def remove_bottle_collision(self) -> None:
        """Remove the bottle from the Planning Scene world."""

        self.arm.remove_collision_object(
            id=self.bottle_collision_id,
        )

        self.get_logger().info(
            f"[SCENE] Requested removal of "
            f"'{self.bottle_collision_id}'"
        )


    def add_static_environment_collision_objects(self) -> bool:
        """Publish only objects whose poses never change."""

        if not self.wait_for_planning_scene_subscriber():
            return False

        self.add_table_collision()

        rclpy.spin_once(
            self,
            timeout_sec=0.5,
        )

        self.get_logger().info(
            "[STATIC SCENE READY] Table published in "
            f"'{panda.base_link_name()}'"
        )

        return True
    
    # ==============================================================
    # Arm planning and execution helpers
    # ==============================================================
    def set_bottle_contact_permission(
        self,
        other_names: Sequence[str],
        allowed: bool,
    ) -> bool:
        """Allow or forbid selected contacts with target_bottle."""

        other_names = list(dict.fromkeys(other_names))

        if not self.planning_scene_client.wait_for_service(
            timeout_sec=3.0
        ):
            self.get_logger().error(
                "[ACM FAILED] /get_planning_scene is unavailable"
            )
            return False

        if not self.apply_planning_scene_client.wait_for_service(
            timeout_sec=3.0
        ):
            self.get_logger().error(
                "[ACM FAILED] /apply_planning_scene is unavailable"
            )
            return False

        # Obtain MoveIt's current Allowed Collision Matrix.
        get_request = GetPlanningScene.Request()
        get_request.components.components = (
            PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        )

        get_future = self.planning_scene_client.call_async(
            get_request
        )

        rclpy.spin_until_future_complete(
            self,
            get_future,
            timeout_sec=3.0,
        )

        if not get_future.done():
            self.planning_scene_client.remove_pending_request(
                get_future
            )
            self.get_logger().error(
                "[ACM FAILED] Timed out reading collision matrix"
            )
            return False

        try:
            get_response = get_future.result()
        except Exception as error:
            self.get_logger().error(
                f"[ACM FAILED] Could not read collision matrix: {error}"
            )
            return False

        if get_response is None:
            self.get_logger().error(
                "[ACM FAILED] Empty Planning Scene response"
            )
            return False

        acm = get_response.scene.allowed_collision_matrix

        matrix_size = len(acm.entry_names)

        if (
            len(acm.entry_values) != matrix_size
            or any(
                len(row.enabled) != matrix_size
                for row in acm.entry_values
            )
        ):
            self.get_logger().error(
                "[ACM FAILED] MoveIt returned a malformed matrix"
            )
            return False

        # Collision objects are not necessarily already represented
        # in the matrix, so add every missing name.
        required_names = [
            self.bottle_collision_id,
            *other_names,
        ]

        for name in required_names:
            if name in acm.entry_names:
                continue

            acm.entry_names.append(name)

            # Add a new column to every existing row.
            for row in acm.entry_values:
                row.enabled.append(False)

            # Add the corresponding new row.
            new_row = AllowedCollisionEntry()
            new_row.enabled = [False] * len(acm.entry_names)
            acm.entry_values.append(new_row)

        name_to_index = {
            name: index
            for index, name in enumerate(acm.entry_names)
        }

        bottle_index = name_to_index[
            self.bottle_collision_id
        ]

        # The matrix is symmetric, so update both directions.
        for other_name in other_names:
            other_index = name_to_index[other_name]

            acm.entry_values[
                bottle_index
            ].enabled[other_index] = allowed

            acm.entry_values[
                other_index
            ].enabled[bottle_index] = allowed

        scene_update = PlanningScene()
        scene_update.is_diff = True
        scene_update.robot_state.is_diff = True
        scene_update.allowed_collision_matrix = acm

        apply_request = ApplyPlanningScene.Request()
        apply_request.scene = scene_update

        apply_future = (
            self.apply_planning_scene_client.call_async(
                apply_request
            )
        )

        rclpy.spin_until_future_complete(
            self,
            apply_future,
            timeout_sec=3.0,
        )

        if not apply_future.done():
            self.apply_planning_scene_client.remove_pending_request(
                apply_future
            )
            self.get_logger().error(
                "[ACM FAILED] Timed out applying collision matrix"
            )
            return False

        try:
            apply_response = apply_future.result()
        except Exception as error:
            self.get_logger().error(
                f"[ACM FAILED] Could not apply collision matrix: "
                f"{error}"
            )
            return False

        if apply_response is None or not apply_response.success:
            self.get_logger().error(
                "[ACM FAILED] MoveIt rejected the collision matrix"
            )
            return False

        mode = "ALLOWED" if allowed else "FORBIDDEN"

        self.get_logger().info(
            f"[ACM] {mode}: '{self.bottle_collision_id}' "
            f"<-> {other_names}"
        )
        return True
    def move_arm_to_joints(
        self,
        stage_name: str,
        joint_positions: Sequence[float],
    ) -> bool:

        self.get_logger().info(
            f"[PLAN] {stage_name}: joint-space goal"
        )

        trajectory = self.arm.plan(
            joint_positions=list(joint_positions),
            joint_names=panda.joint_names(),
        )

        if trajectory is None or len(trajectory.points) == 0:
            self.get_logger().error(
                f"[FAILED] {stage_name}: MoveIt found no trajectory"
            )
            return False

        self.get_logger().info(
            f"[EXECUTE] {stage_name}: "
            f"{len(trajectory.points)} trajectory points"
        )

        self.arm.execute(trajectory)
        success = self.arm.wait_until_executed()

        if not success:
            self.get_logger().error(
                f"[FAILED] {stage_name}: trajectory execution failed"
            )
            return False

        self.get_logger().info(f"[SUCCESS] {stage_name}")
        return True

    def move_arm_to_pose(
        self,
        stage_name: str,
        position: Sequence[float],
        orientation_xyzw: Sequence[float],
        cartesian: bool,
    ) -> bool:

        planning_mode = "Cartesian" if cartesian else "free-space"

        self.get_logger().info(
            f"[PLAN] {stage_name}: {planning_mode} pose goal"
        )

        trajectory = self.arm.plan(
            position=list(position),
            quat_xyzw=list(orientation_xyzw),
            frame_id=panda.base_link_name(),
            target_link=panda.end_effector_name(),
            tolerance_position=0.005,
            tolerance_orientation=0.01,
            cartesian=cartesian,
            max_step=0.0025,
            cartesian_fraction_threshold=(
                self.cartesian_fraction_threshold
                if cartesian
                else 0.0
            ),
        )

        if trajectory is None or len(trajectory.points) == 0:
            self.get_logger().error(
                f"[FAILED] {stage_name}: MoveIt found no trajectory"
            )
            return False

        self.get_logger().info(
            f"[EXECUTE] {stage_name}: "
            f"{len(trajectory.points)} trajectory points"
        )

        self.arm.execute(trajectory)
        success = self.arm.wait_until_executed()

        if not success:
            self.get_logger().error(
                f"[FAILED] {stage_name}: trajectory execution failed"
            )
            return False

        self.get_logger().info(f"[SUCCESS] {stage_name}")
        return True

    def build_figure_eight_waypoints(
        self,
        center_position: Sequence[float],
        orientation_xyzw: Sequence[float],
        horizontal_amplitude: float,
        vertical_amplitude: float,
        cycles: int,
        samples_per_cycle: int,
    ) -> list[Pose]:
        """Generate sampled poses for a continuous X-Z figure-eight."""

        waypoints = []
        total_samples = cycles * samples_per_cycle

        for sample_number in range(1, total_samples + 1):

            phase = (
                2.0
                * math.pi
                * sample_number
                / samples_per_cycle
            )

            pose = Pose()

            pose.position.x = (
                center_position[0]
                + horizontal_amplitude * math.sin(phase)
            )

            pose.position.y = center_position[1]

            pose.position.z = (
                center_position[2]
                + vertical_amplitude * math.sin(2.0 * phase)
            )

            pose.orientation.x = orientation_xyzw[0]
            pose.orientation.y = orientation_xyzw[1]
            pose.orientation.z = orientation_xyzw[2]
            pose.orientation.w = orientation_xyzw[3]

            waypoints.append(pose)

        return waypoints

    def execute_continuous_mixing_path(
        self,
        waypoints: Sequence[Pose],
        cycles: int,
    ) -> bool:
        """Plan and execute all mixing poses as one trajectory."""

        if len(waypoints) == 0:
            self.get_logger().error(
                "[MIX FAILED] No Cartesian waypoints were generated"
            )
            return False

        if not self.cartesian_path_client.wait_for_service(
            timeout_sec=3.0
        ):
            self.get_logger().error(
                "[MIX FAILED] /compute_cartesian_path unavailable"
            )
            return False

        # The Cartesian request must begin from the robot's current
        # post-lift joint state.
        current_joint_state = self.arm.joint_state
        deadline = time.monotonic() + 3.0

        while (
            rclpy.ok()
            and current_joint_state is None
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(self, timeout_sec=0.1)
            current_joint_state = self.arm.joint_state

        if current_joint_state is None:
            self.get_logger().error(
                "[MIX FAILED] Current joint state is unavailable"
            )
            return False

        request = GetCartesianPath.Request()

        request.header.frame_id = panda.base_link_name()
        request.header.stamp = (
            self.get_clock().now().to_msg()
        )

        request.start_state.joint_state = current_joint_state
        request.group_name = panda.MOVE_GROUP_ARM
        request.link_name = panda.end_effector_name()
        request.waypoints = list(waypoints)

        # Preserve the resolution used by your existing Cartesian motions.
        request.max_step = 0.0025

        request.jump_threshold = 0.0
        request.prismatic_jump_threshold = 0.0
        request.revolute_jump_threshold = 0.0

        request.avoid_collisions = True

        request.max_velocity_scaling_factor = (
            self.shake_velocity
        )
        request.max_acceleration_scaling_factor = (
            self.shake_acceleration
        )

        # Zero disables a separate Cartesian linear-speed limit.
        request.cartesian_speed_limited_link = ""
        request.max_cartesian_speed = 0.0

        self.get_logger().info(
            f"[PLAN] CONTINUOUS MIX: "
            f"{len(waypoints)} Cartesian waypoints"
        )

        future = self.cartesian_path_client.call_async(
            request
        )

        rclpy.spin_until_future_complete(
            self,
            future,
            timeout_sec=20.0,
        )

        if not future.done():
            self.cartesian_path_client.remove_pending_request(
                future
            )
            self.get_logger().error(
                "[MIX FAILED] Cartesian planning timed out"
            )
            return False

        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(
                f"[MIX FAILED] Cartesian service error: {error}"
            )
            return False

        if response is None:
            self.get_logger().error(
                "[MIX FAILED] Cartesian service returned no result"
            )
            return False

        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                "[MIX FAILED] MoveIt error code: "
                f"{response.error_code.val}"
            )
            return False

        # Never execute a partial mixing trajectory. Otherwise the arm
        # could stop away from the expected lift centre.
        if response.fraction < 0.999:
            self.get_logger().error(
                "[MIX FAILED] MoveIt completed only "
                f"{response.fraction:.3f} of the requested path"
            )
            return False

        trajectory = response.solution.joint_trajectory

        if len(trajectory.points) == 0:
            self.get_logger().error(
                "[MIX FAILED] Planned trajectory contains no points"
            )
            return False

        trajectory_times = [
            (
                point.time_from_start.sec
                + point.time_from_start.nanosec * 1e-9
            )
            for point in trajectory.points
        ]

        # The controller requires strictly increasing timestamps.
        for index in range(1, len(trajectory_times)):
            if trajectory_times[index] <= trajectory_times[index - 1]:
                self.get_logger().error(
                    "[MIX FAILED] Trajectory timestamps are "
                    "not strictly increasing"
                )
                return False

        duration_seconds = trajectory_times[-1]

        if duration_seconds <= 0.0:
            self.get_logger().error(
                "[MIX FAILED] Trajectory was not time-parameterized"
            )
            return False

        average_frequency = cycles / duration_seconds

        self.get_logger().info(
            f"[PLANNED] CONTINUOUS MIX: "
            f"fraction={response.fraction:.3f} | "
            f"trajectory_points={len(trajectory.points)} | "
            f"duration={duration_seconds:.2f}s | "
            f"average_frequency={average_frequency:.2f}Hz"
        )

        self.get_logger().info(
            "[EXECUTE] CONTINUOUS MIX: one trajectory"
        )

        self.arm.execute(trajectory)
        success = self.arm.wait_until_executed()

        if not success:
            self.get_logger().error(
                "[MIX FAILED] Continuous trajectory execution failed"
            )
            return False

        self.get_logger().info(
            "[SUCCESS] CONTINUOUS MIX"
        )
        return True

    
    def perform_mixing_shake(
        self,
        center_position: Sequence[float],
        orientation_xyzw: Sequence[float],
        horizontal_amplitude: float,
        vertical_amplitude: float,
        cycles: int,
    ) -> bool:
        """Perform one continuous figure-eight mixing trajectory."""

        center_position = list(center_position)
        orientation_xyzw = list(orientation_xyzw)

        if len(center_position) != 3:
            self.get_logger().error(
                "[SHAKE FAILED] Centre must contain x, y and z"
            )
            return False

        if len(orientation_xyzw) != 4:
            self.get_logger().error(
                "[SHAKE FAILED] Orientation must be a quaternion"
            )
            return False

        if horizontal_amplitude <= 0.0:
            self.get_logger().error(
                "[SHAKE FAILED] Horizontal amplitude must be positive"
            )
            return False

        if vertical_amplitude <= 0.0:
            self.get_logger().error(
                "[SHAKE FAILED] Vertical amplitude must be positive"
            )
            return False

        if cycles < 1:
            self.get_logger().error(
                "[SHAKE FAILED] Cycle count must be at least one"
            )
            return False

        if self.shake_samples_per_cycle < 16:
            self.get_logger().error(
                "[SHAKE FAILED] Use at least 16 samples per cycle"
            )
            return False

        if not 0.0 < self.shake_velocity <= 1.0:
            self.get_logger().error(
                "[SHAKE FAILED] Velocity scaling must be in (0, 1]"
            )
            return False

        if not 0.0 < self.shake_acceleration <= 1.0:
            self.get_logger().error(
                "[SHAKE FAILED] Acceleration scaling must be in (0, 1]"
            )
            return False

        waypoints = self.build_figure_eight_waypoints(
            center_position=center_position,
            orientation_xyzw=orientation_xyzw,
            horizontal_amplitude=horizontal_amplitude,
            vertical_amplitude=vertical_amplitude,
            cycles=cycles,
            samples_per_cycle=self.shake_samples_per_cycle,
        )

        self.get_logger().info(
            f"[CONTINUOUS MIX] cycles={cycles} | "
            f"samples_per_cycle={self.shake_samples_per_cycle} | "
            f"horizontal=±{horizontal_amplitude:.3f}m | "
            f"vertical=±{vertical_amplitude:.3f}m | "
            f"velocity={self.shake_velocity:.2f} | "
            f"acceleration={self.shake_acceleration:.2f}"
        )

        if not self.execute_continuous_mixing_path(
            waypoints=waypoints,
            cycles=cycles,
        ):
            return False

        self.get_logger().info(
            "[MIXING COMPLETE] Bottle returned to lift centre"
        )
        return True
    # ==============================================================
    # Direct gripper trajectory action
    # ==============================================================

    def command_gripper(
        self,
        stage_name: str,
        joint_positions: Sequence[float],
        duration_seconds: float = 1.0,
    ) -> bool:

        if len(joint_positions) != 2:
            self.get_logger().error(
                f"[FAILED] {stage_name}: expected two finger positions"
            )
            return False

        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(
                "[FAILED] Gripper action server is unavailable"
            )
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.gripper_joint_names

        point = JointTrajectoryPoint()
        point.positions = list(joint_positions)
        point.time_from_start = Duration(
            seconds=duration_seconds
        ).to_msg()

        goal.trajectory.points = [point]

        self.get_logger().info(
            f"[EXECUTE] {stage_name}: fingers -> "
            f"{list(joint_positions)}"
        )

        send_future = self.gripper_client.send_goal_async(goal)

        rclpy.spin_until_future_complete(
            self,
            send_future,
            timeout_sec=5.0,
        )

        if not send_future.done():
            self.get_logger().error(
                f"[FAILED] {stage_name}: goal request timed out"
            )
            return False

        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(
                f"[FAILED] {stage_name}: controller rejected the goal"
            )
            return False

        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(
            self,
            result_future,
            timeout_sec=duration_seconds + 5.0,
        )

        if not result_future.done():
            self.get_logger().error(
                f"[FAILED] {stage_name}: execution timed out"
            )
            return False

        result_wrapper = result_future.result()

        if result_wrapper is None:
            self.get_logger().error(
                f"[FAILED] {stage_name}: controller returned no result"
            )
            return False

        controller_result = result_wrapper.result

        success = (
            result_wrapper.status == GoalStatus.STATUS_SUCCEEDED
            and controller_result.error_code
            == FollowJointTrajectory.Result.SUCCESSFUL
        )

        if not success:
            self.get_logger().error(
                f"[FAILED] {stage_name}: "
                f"error_code={controller_result.error_code}, "
                f"message='{controller_result.error_string}'"
            )
            return False

        self.get_logger().info(f"[SUCCESS] {stage_name}")
        return True

    # ==============================================================
    # Day 1 task sequence
    # ==============================================================

    def execute_day1_pick_place(self) -> bool:

        if self.target_coords is None:
            self.get_logger().error(
                "Cannot begin because no target has been detected"
            )
            return False
        
        if self.original_bottle_position is None:
            self.get_logger().error(
                "Cannot begin because the vision bottle "
                "has not been synchronized with MoveIt"
            )
            return False
        # Preserve the original repository's working coordinate correction
        # for Day 1. We will replace this during perception improvement.
        # For Day 1, perception supplies bottle X and Y.
        # Horizontal grasp from the bottle's negative-Y side.
        #
        # panda_hand +Z -> world +Y: approach direction
        # panda_hand +Y -> world +X: finger opening direction
        # panda_hand +X -> world +Z: keeps the jaws vertical
        side_grasp_orientation = [
            -0.5,
            -0.5,
            -0.5,
            0.5,
        ]

        # Grasp the upper body of the bottle to clear the tabletop.
        grasp_height = (
            self.original_bottle_position[2] + 0.05
        )

        grasp_position = [
            self.target_coords[0],
            self.target_coords[1] - self.side_grasp_depth,
            grasp_height,
        ]

        pre_grasp_position = [
            grasp_position[0],
            grasp_position[1] - self.approach_offset,
            grasp_position[2],
        ]

        lift_position = [
            grasp_position[0],
            grasp_position[1],
            grasp_position[2] + self.lift_distance,
        ]
        self.get_logger().info(
            f"Pre-grasp position: {pre_grasp_position}"
        )
        self.get_logger().info(
            f"Grasp position: {grasp_position}"
        )

        if not self.move_arm_to_joints(
            "HOME",
            self.home_joints,
        ):
            return False

        if not self.command_gripper(
            "OPEN GRIPPER",
            self.gripper_open,
        ):
            return False

        if not self.move_arm_to_pose(
            "PRE-GRASP",
            pre_grasp_position,
            side_grasp_orientation,
            cartesian=False,
        ):
            return False
        
        if not self.set_bottle_contact_permission(
            self.grasp_contact_links,
            allowed=True,
        ):
            return False

        
        if not self.move_arm_to_pose(
            "APPROACH",
            grasp_position,
            side_grasp_orientation,
            cartesian=True,
        ):
            return False

        if not self.command_gripper(
            "GRASP BOTTLE",
            self.gripper_grasp,
        ):
            return False

        self.get_logger().info(
            "SIDE-GRASP VERIFIED: beginning attachment and lift"
        )

        # The bottle initially touches the table, so temporarily allow
        # this contact while creating the attached collision object.
        if not self.set_bottle_contact_permission(
            [self.table_collision_id],
            allowed=True,
        ):
            return False

        # Attach the physical Gazebo bottle and its MoveIt representation.
        if not self.attach_bottle_for_transport():
            return False

        # Lift vertically while preserving the successful side-grasp orientation.
        if not self.move_arm_to_pose(
            "LIFT",
            lift_position,
            side_grasp_orientation,
            cartesian=True,
        ):
            return False

        # The lifted bottle is now clear of the table.
        if not self.set_bottle_contact_permission(
            [self.table_collision_id],
            allowed=False,
        ):
            return False

        self.get_logger().info(
            "LIFT VERIFIED: beginning controlled shake"
        )

        if not self.perform_mixing_shake(
            center_position=lift_position,
            orientation_xyzw=side_grasp_orientation,
            horizontal_amplitude=(
                self.shake_horizontal_amplitude
            ),
            vertical_amplitude=(
                self.shake_vertical_amplitude
            ),
            cycles=self.shake_cycles,
        ):
            return False

        if not self.place_bottle_gently(
            grasp_position=grasp_position,
            retreat_position=pre_grasp_position,
            orientation_xyzw=side_grasp_orientation,
        ):
            return False

        if not self.move_arm_to_joints(
            "RETURN HOME",
            self.home_joints,
        ):
            return False

        self.get_logger().info(
            "BOTTLE MIXING TASK COMPLETED SUCCESSFULLY"
        )
        return True

def main(args=None):
    rclpy.init(args=args)
    node = BottleShakeTask()
    try:
        scene_success = node.add_static_environment_collision_objects()

        if not scene_success:
            node.get_logger().error(
                "Initialization failed; Planning Scene is unavailable"
            )
            return
        node.get_logger().info(
            f"Waiting for target color {node.target_color}"
        )

        while rclpy.ok() and node.target_coords is None:
            rclpy.spin_once(
                node,
                timeout_sec=0.1,
            )

        if not rclpy.ok():
            return

        if not node.synchronize_bottle_with_vision():
            node.get_logger().error(
                "Initialization failed; dynamic target unavailable"
            )
            return

        
        node.get_logger().info(
            "Initializing robot at START configuration"
        )

        start_success = node.move_arm_to_joints(
            "START",
            node.start_joints,
        )

        if not start_success:
            node.get_logger().error(
                "Initialization failed; task will not start"
            )
            return


        task_success = node.execute_day1_pick_place()

        if not task_success:
            node.get_logger().error(
                "TASK ABORTED: inspect the first failed stage"
            )


    except KeyboardInterrupt:
        node.get_logger().warn("Task interrupted by user")

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()