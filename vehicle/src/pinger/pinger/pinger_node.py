import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Range
from brping import Ping1D

class Talker(Node):

    def __init__(self):
        super().__init__('pinger_node')

        # Setup Ping1D sensor
        self.setup_ping()        

        # Create publisher
        self.pub = self.create_publisher( Range, 'depth', 1 )

        # Setup timer
        timer_period = 1.0
        self.timer = self.create_timer( timer_period, self.timer_callback )

    def setup_ping(self):
        # Create Ping1D object
        self.ping = Ping1D( "/dev/ttyUSB0", 115200 )

        # Initialize
        if self.ping.initialize() is False:
            print("Failed to initialize Ping!")
            exit(1)
        
        # Speed of sound (1500m/s)
        if self.ping.set_speed_of_sound( 1500000 ) is False:
            print("Failed to set speed of sound")
            exit(1)

        # Auto-mode
        if self.ping.set_mode_auto( 1 ) is False:
            print("Failed to set speed of auto mode")
            exit(1)

        # Interval (10Hz)
        if self.ping.set_ping_interval( 10 ) is False:
            print("Failed to set ping interval")
            exit(1)

        # Enable
        if self.ping.set_ping_enable( 1 ) is False:
            print("Failed to enable Ping1D")
            exit(1)

    def sample( self ):
        data = self.ping.get_distance()
        if data:
            msg = Range()
            msg.radiation_type  = 0
            msg.min_range       = 0.5
            msg.max_range       = 30.0
            msg.range           = float( data["distance"] ) / 1000.0
            msg.field_of_view   = float( data["confidence"] ) / 100.0   # Abusing this field to store the confidence value
            msg.header.stamp    = self.get_clock().now().to_msg()
            self.pub.publish(msg)
        else:
            print( "Failed to get distance data" )

    def timer_callback(self):
        self.sample()

def main(args=None):
    rclpy.init(args=args)

    node = Talker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()