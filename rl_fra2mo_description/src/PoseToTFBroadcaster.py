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


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations as tf

# global_var = 0

class PoseToTFBroadcaster(Node):
    def __init__(self):
        super().__init__('aruco_sub_pub')

        # Subscriber al topic /aruco_single/pose
        self.subscription = self.create_subscription(
            PoseStamped,
            '/aruco_single/pose',
            self.pose_callback,
            10
        )

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.marker_published = False
        self.subscription
        self.get_logger().info('Nodo avviato e in ascolto su /aruco_single/pose')

        # Broadcaster per tf_static
       

        # self.get_logger().info('Nodo avviato e in ascolto su /aruco_single/pose')

    def pose_callback(self, msg):

        if self.marker_published:
            # Se il marker è già stato pubblicato, evita ulteriori aggiornamenti
            return


        # global global_var
        # Converte il messaggio Pose in un TransformStamped
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'  # Frame di riferimento principale
        t.child_frame_id = 'aruco_marker_frame'  # Frame del marker

        #if msg.pose.position.z > 0.0 and not global_var:
        self.get_logger().info('Transformazione pubblicata su /tf_static')
        # Imposta posizione
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z

        # Imposta orientamento
        t.transform.rotation.x = msg.pose.orientation.x
        t.transform.rotation.y = msg.pose.orientation.y
        t.transform.rotation.z = msg.pose.orientation.z
        t.transform.rotation.w = msg.pose.orientation.w

           # global_var = 1

        # Pubblica la trasformazione su /tf_static
        self.tf_static_broadcaster.sendTransform(t)
        self.marker_published = True  # Imposta il flag a True
        self.get_logger().info('Trasformazione statica pubblicata per il marker.')
        #self.get_logger().info('Transformazione pubblicata su /tf_static')


def main():
    rclpy.init()

    node_aruco = PoseToTFBroadcaster()

    try:
        rclpy.spin(node_aruco)
    except KeyboardInterrupt:
        node_aruco.get_logger().info('Nodo interrotto dall utente')
    finally:
        node_aruco.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()