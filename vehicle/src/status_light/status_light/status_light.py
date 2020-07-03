import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

from std_msgs.msg import UInt32

import datetime

import Jetson.GPIO as GPIO

class StatusLight(Node):

    def __init__(self):
        super().__init__('status_light')

        self.sample_received    = False
        self.light_on           = False

        # Init GPIO
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.setup(7, GPIO.OUT, initial=GPIO.LOW)
        GPIO.output(7, GPIO.LOW)

        # Create QOS for subscriber
        qos_klr_v = QoSProfile(history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                        depth=1,
                        reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                        durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE)

        # Create subscriber
        self.sub = self.create_subscription( msg_type=UInt32,
            topic='camera_trigger',
            callback=self.listener_callback,
            qos_profile=qos_klr_v )

        # Setup timer
        timer_period_secs = 0.2
        self.timer = self.create_timer( timer_period_secs, self.timer_callback )

    def listener_callback(self, msg):
        self.sample_received = True

    def timer_callback(self):
        # After one timer period, the light will turn off, completing the blink
        if self.light_on:
            GPIO.output(7, GPIO.LOW)
            self.light_on = False

        # If a sample has been received, turn the light on for one period
        if self.sample_received:
            GPIO.output(7, GPIO.HIGH)
            self.light_on = True
            self.sample_received = False

def main(args=None):
    rclpy.init(args=args)

    node = StatusLight()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()