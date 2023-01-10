import rclpy
from rclpy.node import Node

import random as rand
from detections.msg import DetectionSet                            # CHANGE
from detections.msg import Detection                            # CHANGE

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(DetectionSet, 'Detections', 10)
        timer_period = 0.07692  # seconds (period of 13Hz)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = DetectionSet()
        log = ""
        cnt = 0

        for ball_ind in range(rand.randint(0, 9)):
            sample_detect = Detection()

            log_str = "BOUNDS: " 

            for i in range(4):
                sample_detect.bounds[i] = rand.uniform(0.0, 4032.0)         # set some random value to the bounding box
                log_str += (str(sample_detect.bounds[i]) + ", ")  

            msg.detections.append(sample_detect)
                                                                        # Does not guarentee that top>bottom, left<right
            log += ( "Detection #" + str(ball_ind) + " : " + log_str + " || ")

            cnt+=1

        msg.num_detections = cnt
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % (log))
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()