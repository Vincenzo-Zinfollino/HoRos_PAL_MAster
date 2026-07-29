#!/usr/bin/env python3

# Copyright (c) 2025 PAL Robotics S.L. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPlanningScene
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import PoseStamped


class AddGroundPlaneNode(Node):
    def __init__(self):
        super().__init__("add_ground_scene_node")
        self.cli = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.scene_publisher = self.create_publisher(
            PlanningScene, "/planning_scene", 10
        )

        self.get_logger().info("Waiting for MoveIt's /get_planning_scene service...")

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("MoveIt service not available yet, retrying...")

        self.get_logger().info("MoveIt is ready!")

    def get_planning_scene(self):
        req = GetPlanningScene.Request()

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            planning_scene = future.result().scene
            self.get_logger().info("Retrieved existing planning scene.")
        else:
            self.get_logger().warn(
                "Failed to retrieve planning scene. Proceeding with an empty scene."
            )
            planning_scene = PlanningScene()

        return planning_scene

    def add_ground_plane(self):
        ground_object = CollisionObject()
        ground_object.header.frame_id = "base_footprint"
        ground_object.id = "ground"

        # Define ground as a box
        ground_primitive = SolidPrimitive()
        ground_primitive.type = SolidPrimitive.BOX
        ground_primitive.dimensions = [3.0, 3.0, 0.045]  # Size of the ground plane

        # Define the pose (slightly below base link to avoid floating point issues)
        ground_pose = PoseStamped()
        ground_pose.header.frame_id = "base_footprint"
        ground_pose.pose.position.z = -0.01

        ground_object.primitives.append(ground_primitive)
        ground_object.primitive_poses.append(ground_pose.pose)
        ground_object.operation = CollisionObject.ADD

        planning_scene = self.get_planning_scene()
        planning_scene.is_diff = True
        planning_scene.world.collision_objects.append(ground_object)

        self.scene_publisher.publish(planning_scene)

        self.get_logger().info("Updated planning scene with ground plane.")


def main():
    rclpy.init()
    node = AddGroundPlaneNode()
    node.add_ground_plane()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
