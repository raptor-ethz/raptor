# Copyright 2016 Open Source Robotics Foundation, Inc.
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

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import imagezmq
from imutils.video import VideoStream

class SimulatorImageSender(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/realsense_front_rgb',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.br = CvBridge()
        self.sender = imagezmq.ImageSender(connect_to='tcp://localhost:4444')

    def listener_callback(self, msg):
        self.get_logger().info("Receiving video frame")
        frame = self.br.imgmsg_to_cv2(msg)
        self.sender.send_image("Gazebo Simulator", frame)
        cv2.imshow("camera", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    sim_image_sender = SimulatorImageSender()

    rclpy.spin(sim_image_sender)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sim_image_sender.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
