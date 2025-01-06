#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
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

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.duration import Duration
import yaml
import os

from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformListener
#METTI NEL CMAKE
# from tf2_ros import StaticTransformBroadcaster
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf_transformations import euler_from_quaternion, quaternion_from_euler
import math
import sys
import numpy as np
from rclpy.node import Node


def main():
    rclpy.init()

    # Nodo per gestire i parametri
    node = Node("waypoint_navigator")
    try:
        # Dichiarazione del parametro "target" con valore di default
        node.declare_parameter("target", "nav")

        # Recupera il valore del parametro "target"
        target = node.get_parameter("target").get_parameter_value().string_value

        navigator = BasicNavigator()

        goal_path=get_package_share_directory('rl_fra2mo_description')
        yaml_file = os.path.join(goal_path, 'config','goals.yaml')

        # Carica i waypoint dal file YAML
        with open(yaml_file, 'r') as file:
            waypoints = yaml.safe_load(file)

        # Riordina i goal
        if target == "nav":
            order = ["goal_3", "goal_4", "goal_2", "goal_1"]
        elif target == "map":
            order = ["goal_5", "goal_6", "goal_7", "goal_8", "goal_4", "goal_9", "goal_10"]
        elif target == "aruco":
            order = ["goal_11", "goal_0"]


        ordered_goals = [goal for name in order for goal in waypoints["goals"] if goal["name"] == name] # nested list 

        def create_pose(transform): # function to assign the goals and the transform into PoseStamped
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = navigator.get_clock().now().to_msg()
            pose.pose.position.x = transform["position"]["x"]
            pose.pose.position.y = transform["position"]["y"]
            pose.pose.position.z = transform["position"]["z"]
            pose.pose.orientation.x = transform["orientation"]["x"]
            pose.pose.orientation.y = transform["orientation"]["y"]
            pose.pose.orientation.z = transform["orientation"]["z"]
            pose.pose.orientation.w = transform["orientation"]["w"]
            return pose
        


        # goal_poses = list(map(create_pose, waypoints["goals"]))
        goal_poses = list(map(create_pose, ordered_goals))



        # Wait for navigation to fully activate, since autostarting nav2
        navigator.waitUntilNav2Active(localizer="smoother_server")

        # sanity check a valid path exists
        # path = navigator.getPath(initial_pose, goal_pose)

        nav_start = navigator.get_clock().now()
        navigator.followWaypoints(goal_poses)

        i = 0
        while not navigator.isTaskComplete():
            ################################################
            #
            # Implement some code here for your application!
            #
            ################################################

            # Do something with the feedback
            i = i + 1
            feedback = navigator.getFeedback()

            if feedback and i % 5 == 0:
                print('Executing current waypoint: ' +
                    str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)))
                now = navigator.get_clock().now()

                # Some navigation timeout to demo cancellation
                if now - nav_start > Duration(seconds=600):
                    navigator.cancelTask()
        
        

        # Do something depending on the return code
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')
        # navigator.lifecycleShutdown()

    except KeyboardInterrupt:
        print("Nodo interrotto dall'utente.")
    finally:
        node.destroy_node()  # Distrugge il nodo
        rclpy.shutdown()
        print("Shutdown completato.")


    # exit(0)


if __name__ == '__main__':
    main()
