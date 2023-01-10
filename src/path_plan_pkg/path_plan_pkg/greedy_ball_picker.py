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

from detections.msg import DetectionSet                           # CHANGE
from detections.msg import Detection                           # CHANGE

# Scores a ball on how likely the ball is the closest ball in frame
def detect_score(detection):
    ret = 0

    # Calculate the area (bigger area in camera = closer)
    ret += abs(detection.bounds[2] - detection.bounds[0])   
    ret +=  abs(detection.bounds[3] - detection.bounds[1])         # Top - Bottom coordinates

    ret += detection.bounds[3];      # Balls that are closer to the bottom of frame get higher score

    ret += detection.conf;      # Add this as a tie breaker so we go towards balls we're more condident in

    return ret

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('greedy_detection_selector')
        self.subscription = self.create_subscription(
            DetectionSet,
            'Detections',
            self.listener_callback,     
            10)             # queue depth
        self.publisher_ = self.create_publisher(Detection, 'Closest_Detection', 10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):

        closest_detect = Detection()
        highest_detect_score = 0
        ind_detect = 0
        ret_ind = 0

        # find the closest detection based on "detect score" policy
        for detection in  msg.detections:
            score = detect_score(detection)
            if score > highest_detect_score:
                highest_detect_score = score
                closest_detect = detection
                ret_ind = ind_detect
            ind_detect += 1

        self.publisher_.publish(closest_detect)             # publish the closest detection
        
        self.get_logger().info(str('Detection #: ' + str(ret_ind) + ',  is the closest Detection'))



def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()