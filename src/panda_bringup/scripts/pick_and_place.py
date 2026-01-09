#!/usr/bin/env python3

import time
from threading import Thread

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs

from pymoveit2 import MoveIt2, GripperInterface
from pymoveit2.robots import panda


class PickCube(Node):

    def __init__(self):
        super().__init__("pick_cube")

        self.cube_pose_world  = [0.7, 0.5, 0.70]
        self.place_pose_world = [0.2, 0.5, 0.70]
        self.wall_pose_world  = [0.4, 0.5, 0.70]

        self.cube_size = 0.06
        self.wall_size = [0.1, 0.3, 0.6]

        self.hover_offset     = 0.05
        self.post_pick_lift   = 0.50
        self.place_hover      = 0.20
        self.tcp_offset       = 0.16
        self.grasp_depth      = 0.07

        self.ee_quat = [0.0, 1.0, 0.0, 0.0]
        self.identity_quat = [0.0, 0.0, 0.0, 1.0]

        self.cartesian_step = 0.01
        self.cartesian_fraction = 0.9

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=panda.joint_names(),
            base_link_name=panda.base_link_name(),
            end_effector_name=panda.end_effector_name(),
            group_name=panda.MOVE_GROUP_ARM,
        )

        self.moveit2.max_velocity = 0.05
        self.moveit2.max_acceleration = 0.05
        self.planning_frame = self.moveit2.base_link_name

      
        self.gripper = GripperInterface(
            node=self,
            gripper_joint_names=panda.gripper_joint_names(),
            open_gripper_joint_positions=panda.OPEN_GRIPPER_JOINT_POSITIONS,
            closed_gripper_joint_positions=panda.CLOSED_GRIPPER_JOINT_POSITIONS,
            gripper_group_name=panda.MOVE_GROUP_GRIPPER,
            gripper_command_action_name="gripper_action_controller/gripper_cmd",
        )

        Thread(target=self.run, daemon=True).start()


    # World → panda_link0
    def world_to_panda(self, xyz):
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.pose.position.x = xyz[0]
        pose.pose.position.y = xyz[1]
        pose.pose.position.z = xyz[2]
        pose.pose.orientation.w = 1.0

        return self.tf_buffer.transform(
            pose,
            self.planning_frame,
            timeout=rclpy.duration.Duration(seconds=1),
        )

    def run(self):

        self.get_logger().info("Waiting for TF...")
        time.sleep(2.0)

        # adding wall to moveit2
        
        self.moveit2.add_collision_box(
            id="wall",
            size=tuple(self.wall_size),
            position=self.wall_pose_world,
            quat_xyzw=self.identity_quat,
            frame_id="world",
        )
        
        # Wait for the collision object to be registered in the planning scene
        self.get_logger().info("Waiting for wall to be registered...")
        time.sleep(2.0)

        cube_panda = self.world_to_panda(self.cube_pose_world)

        cx, cy, cz = (
            cube_panda.pose.position.x,
            cube_panda.pose.position.y,
            cube_panda.pose.position.z,
        )

        cube_top_z = cz + self.cube_size / 2.0

        hover_z      = cube_top_z + self.hover_offset   - self.tcp_offset
        grasp_z      = cube_top_z - self.grasp_depth    - self.tcp_offset
        high_hover_z = cube_top_z + self.post_pick_lift - self.tcp_offset

        hover_pose      = [cx, cy, hover_z]
        grasp_pose      = [cx, cy, grasp_z]
        high_hover_pose = [cx, cy, high_hover_z]

# Move to pre-pick hover position (above the cube)
        self.moveit2.move_to_pose(
            position=hover_pose,
            quat_xyzw=self.ee_quat,
            frame_id=self.planning_frame,
        )
        self.moveit2.wait_until_executed()

# Open gripper to prepare for pick
        self.gripper.open()
        self.gripper.wait_until_executed()

# Descend
        self.moveit2.move_to_pose(
            position=grasp_pose,
            quat_xyzw=self.ee_quat,
            frame_id=self.planning_frame,
            cartesian=True,
            cartesian_max_step=self.cartesian_step,
            cartesian_fraction_threshold=self.cartesian_fraction,
        )
        self.moveit2.wait_until_executed()

# Close gripper
        self.gripper.close()
        self.gripper.wait_until_executed()


# Lift the cube
        self.moveit2.move_to_pose(
            position=high_hover_pose,
            quat_xyzw=self.ee_quat,
            frame_id=self.planning_frame,
            cartesian=True,
        )
        self.moveit2.wait_until_executed()

        place_panda = self.world_to_panda(self.place_pose_world)
        px, py, pz = (
            place_panda.pose.position.x,
            place_panda.pose.position.y,
            place_panda.pose.position.z,
        )

        place_hover_z = pz + self.place_hover - self.tcp_offset
        place_down_z  = pz - self.grasp_depth - self.tcp_offset

        place_hover_pose = [px, py, place_hover_z]
        place_down_pose  = [px, py, place_down_z]

# Move to hover position above the goal location
        self.moveit2.move_to_pose(
            position=place_hover_pose,
            quat_xyzw=self.ee_quat,
            frame_id=self.planning_frame,
        )
        self.moveit2.wait_until_executed()

# Place the cube
        self.moveit2.move_to_pose(
            position=place_down_pose,
            quat_xyzw=self.ee_quat,
            frame_id=self.planning_frame,
            cartesian=True,
            cartesian_max_step=self.cartesian_step,
            cartesian_fraction_threshold=self.cartesian_fraction,
        )
        self.moveit2.wait_until_executed()

# Open gripper
        self.gripper.open()
        self.gripper.wait_until_executed()
        
# back to hover position
        self.moveit2.move_to_pose(
            position=place_hover_pose,
            quat_xyzw=self.ee_quat,
            frame_id=self.planning_frame,
            cartesian=True,
        )
        self.moveit2.wait_until_executed()

        self.get_logger().info("place complete")


def main():
    rclpy.init()
    node = PickCube()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
