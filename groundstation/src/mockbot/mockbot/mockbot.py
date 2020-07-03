import sys, os, threading, datetime

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

from sensor_msgs.msg import Range
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import UInt64, String, Bool
from sensor_msgs.msg import NavSatFix, CompressedImage

import random

class Talker(Node):

    def __init__(self):
        super().__init__('mockbot')

        qos_best_effort = QoSProfile(history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                                depth=1,
                                reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)

        qos_klr_v       = QoSProfile(history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                                depth=1,
                                reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                                durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE)

        qos_klr_tl      = QoSProfile(history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                                depth=1,
                                reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                                durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)
        self.start_time = random.uniform(10,100)
        self.time_elapsed = 0
        
        self.recorded_samples = 0
        self.is_recording = False

        # Create bridge publishers
        self.p_time     = self.create_publisher( UInt64, 'time_unix_ms', qos_best_effort )
        self.p_gps      = self.create_publisher( NavSatFix, 'gps', qos_klr_tl )
        self.p_attitude = self.create_publisher( Vector3Stamped, 'attitude', qos_klr_tl )
        self.p_speed    = self.create_publisher( Vector3Stamped, 'speed', qos_klr_tl )
        self.p_depth    = self.create_publisher( Range, 'depth', qos_klr_tl )

        # Create recorder publishers
        self.p_heartbeat            = self.create_publisher( String, 'heartbeat', qos_klr_v )
        self.p_camera_status        = self.create_publisher( String, 'camera_status', qos_klr_tl )
        self.p_gps_status           = self.create_publisher( String, 'gps_status', qos_klr_tl )
        self.p_storage_status       = self.create_publisher( String, 'storage_status', qos_klr_tl )
        self.p_system_time          = self.create_publisher( String, 'system_time', qos_klr_tl )
        self.p_recording_status     = self.create_publisher( String, 'recording_status', qos_klr_tl )
        self.p_recorded_samples     = self.create_publisher( String, 'recorded_samples', qos_klr_tl )
        self.p_video_left           = self.create_publisher( CompressedImage, 'camera_left/image_raw/compressed', qos_best_effort )

        # Create recorder subscriber
        self.s_recording_enabled_target = self.create_subscription(
            msg_type=Bool,
            topic='recording_enabled_target',
            callback=self.update_recording_enabled,
            qos_profile=qos_klr_tl)

        # Setup timer
        timer_period = 1.0
        self.timer = self.create_timer( timer_period, self.timer_callback )

        # Get path to images
        package_share_directory = get_package_share_directory('mockbot')
        img_path = os.path.join( package_share_directory, "img" )

        # Load images
        self.image_index = 0
        self.turtle1_msg = CompressedImage(format="jpeg")
        self.turtle2_msg = CompressedImage(format="jpeg")

        with open( os.path.join( img_path, "turtle1.jpg" ), "rb") as f:
            self.turtle1_msg.data = f.read()
        
        with open( os.path.join( img_path, "turtle2.jpg" ), "rb") as f:
            self.turtle2_msg.data = f.read()

    def update_recording_enabled(self, msg):
        print(msg)
        self.is_recording = msg.data

    def timer_callback(self):
        print( "Publishing..." )
        self.time_elapsed = self.time_elapsed + 1
        self.pub_time()
        self.pub_gps()
        self.pub_attitude()
        self.pub_speed()
        self.pub_depth()

        self.pub_heartbeat()
        self.pub_camera_status()
        self.pub_gps_status()
        self.pub_storage_status()
        self.pub_system_time()
        self.pub_recording_status()
        self.pub_recorded_samples()
        self.pub_image()

    def pub_heartbeat(self):
        msg = String()
        self.p_heartbeat.publish( msg )
    
    def pub_camera_status(self):
        msg = String()
        msg.data = "OK"
        self.p_camera_status.publish( msg )
    
    def pub_gps_status(self):
        msg = String()
        msg.data = "OK"
        self.p_gps_status.publish( msg )
    
    def pub_storage_status(self):
        msg = String()
        msg.data = "OK"
        self.p_storage_status.publish( msg )
    
    def pub_system_time(self):
        msg = String()
        msg.data = "OK"
        self.p_system_time.publish( msg )

    def pub_recording_status(self):
        msg = String()

        if self.is_recording:
            msg.data = "RECORDING"
        else:
            msg.data = "OFF"
        
        self.p_recording_status.publish( msg )
    
    def pub_recorded_samples(self):
        if self.is_recording:
            self.recorded_samples += 1

        msg = String()
        msg.data = str( self.recorded_samples )
        self.p_recorded_samples.publish( msg )

    def pub_image(self):
        if self.image_index == 0:
            self.p_video_left.publish( self.turtle1_msg )
            self.image_index = 1
        elif self.image_index == 1:
            self.p_video_left.publish( self.turtle2_msg )
            self.image_index = 0

    def pub_gps(self):
        msg = NavSatFix()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.status.status = 0
        msg.latitude        = random.uniform( 32.000, 32.000200 )
        msg.longitude       = random.uniform( 72.000, 72.000200 )
        self.p_gps.publish(msg)

    def pub_time(self):
        msg = UInt64()
        msg.data = self.time_elapsed
        self.p_time.publish(msg)

    def pub_attitude(self):
        msg = Vector3Stamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.vector.x        = random.uniform(0.0,0.5)
        msg.vector.y        = random.uniform(0.0,0.5)
        msg.vector.z        = random.uniform(0.0,6.28)
        self.p_attitude.publish(msg)

    def pub_speed(self):
        msg = Vector3Stamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.vector.z        = random.uniform(0.0,2.0)
        self.p_speed.publish(msg)

    def pub_depth(self):
        msg = Range()
        msg.radiation_type  = 0
        msg.min_range       = 0.5
        msg.max_range       = 30.0
        msg.range           = random.uniform(1.0,5.0)
        msg.field_of_view   = 100.0
        msg.header.stamp    = self.get_clock().now().to_msg()
        self.p_depth.publish(msg)


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