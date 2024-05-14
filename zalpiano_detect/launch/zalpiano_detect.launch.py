# Copyright 2022 Factor Robotics
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

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    aruco_detect_node = Node(
        package="zalpiano_detect",
        executable="aruco_detect",
        name="aruco_detect",
    )

    aruco_to_center_node = Node(
        package="zalpiano_detect",
        executable="aruco_to_center",
        name="aruco_to_center",
    )

    image_publisher_node = Node(
        package="zalpiano_detect",
        executable="image_publisher",
        name="image_publisher",
    )

    return LaunchDescription([
        aruco_detect_node,
        aruco_to_center_node,
        # image_publisher_node,
    ])
