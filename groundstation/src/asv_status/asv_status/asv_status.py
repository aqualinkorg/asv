import sys, os, threading, datetime

from PyQt5 import QtWidgets, QtCore, uic
from PyQt5.QtGui import QPixmap

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

from std_msgs.msg import String, Bool
from sensor_msgs.msg import CompressedImage

class Ui(QtWidgets.QMainWindow):
    def __init__(self):
        super(Ui, self).__init__()

        # Initialize UI
        self.load_ui()
        self.reset_ui_state()

        # Init state
        self.recording_enabled_target = False

        # Create ROS
        self.node = Node( "asv_status" )

        # Init ROS Entities
        self.init_ros()
    
        # Create Qt timer to poll ROS2 node for events
        self.ros_timer = QtCore.QTimer(self)
        self.ros_timer.timeout.connect(self.handle_ros)
        self.ros_timer.start(10)

        # Connect Checkbox signal to publish ROS message with updated state
        self.checkbox_recording_enabled.stateChanged.connect( self.update_recording_enable_target )

        # Display window
        self.show()

    def update_recording_enable_target(self, state):
        if (QtCore.Qt.Checked == state):
            self.recording_enabled_target = True
        else:
            self.recording_enabled_target = False
        
        msg = Bool()
        msg.data = self.recording_enabled_target
        self.pub_req_enable_recording.publish(msg)

    def init_ros(self):
        # Create ROS Timer for vehicle heartbeat
        self.heartbeat_received = False
        self.last_heartbeat_time = datetime.datetime.now()
        self.heartbeat_timer = self.node.create_timer( 1, self.heartbeat_callback )

        # Create QOS profiles
        qos_be      = QoSProfile(history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                                depth=1,
                                reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)

        qos_klr_v   = QoSProfile(history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                                depth=1,
                                reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                                durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE)

        qos_klr_tl  = QoSProfile(history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                                depth=1,
                                reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                                durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)

        # Create Subscribers
        self.sub_heartbeat = self.node.create_subscription(
            msg_type=String,
            topic='heartbeat',
            callback=self.update_heartbeat,
            qos_profile=qos_klr_v)

        self.sub_camera_status = self.node.create_subscription(
            msg_type=String,
            topic='camera_status',
            callback=self.update_camera_status,
            qos_profile=qos_klr_tl)
        
        self.sub_gps_status = self.node.create_subscription(
            msg_type=String,
            topic='gps_status',
            callback=self.update_gps_status,
            qos_profile=qos_klr_tl)
        
        self.sub_storage_status = self.node.create_subscription(
            msg_type=String,
            topic='storage_status',
            callback=self.update_storage_status,
            qos_profile=qos_klr_tl)
        
        self.sub_system_time = self.node.create_subscription(
            msg_type=String,
            topic='system_time',
            callback=self.update_system_time,
            qos_profile=qos_klr_tl)
        
        self.sub_recording_status = self.node.create_subscription(
            msg_type=String,
            topic='recording_status',
            callback=self.update_recording_status,
            qos_profile=qos_klr_tl)
        
        self.sub_recorded_samples = self.node.create_subscription(
            msg_type=String,
            topic='recorded_samples',
            callback=self.update_recorded_samples,
            qos_profile=qos_klr_tl)
        
        self.sub_video_left = self.node.create_subscription(
            msg_type=CompressedImage,
            topic='camera_left/image_raw/compressed',
            callback=self.update_video_image,
            qos_profile=qos_be)

        # Create publisher
        self.pub_req_enable_recording = self.node.create_publisher( 
            msg_type=Bool, 
            topic='recording_enabled_target', 
            qos_profile=qos_klr_tl )

        # Set initial recording enable target
        msg = Bool()
        msg.data = self.recording_enabled_target
        self.pub_req_enable_recording.publish(msg)

    def load_ui(self):
        # Get path to Qt UI file
        package_share_directory = get_package_share_directory('asv_status')
        ui_path = os.path.join( package_share_directory, "ui/asv_status.ui" )

        # Load UI
        uic.loadUi(ui_path, self)

        # Find UI elements
        self.label_camera_status        = self.findChild(QtWidgets.QLabel, 'label_camera_status')
        self.label_gps_status           = self.findChild(QtWidgets.QLabel, 'label_gps_status')
        self.label_storage_status       = self.findChild(QtWidgets.QLabel, 'label_storage_status')
        self.label_system_time          = self.findChild(QtWidgets.QLabel, 'label_system_time')
        self.label_recording_status     = self.findChild(QtWidgets.QLabel, 'label_recording_status')
        self.label_recorded_samples     = self.findChild(QtWidgets.QLabel, 'label_recorded_samples')
        self.label_video                = self.findChild(QtWidgets.QLabel, 'label_video')
        self.checkbox_recording_enabled = self.findChild(QtWidgets.QCheckBox, 'checkbox_recording_enabled')
    
    def reset_ui_state(self):
        # Set starting values for labels
        self.update_camera_status( String( data="UNKNOWN" ) )
        self.update_gps_status( String( data="UNKNOWN" ) )
        self.update_storage_status( String( data="UNKNOWN" ) )
        self.update_system_time( String( data="UNKNOWN" ) )
        self.update_recording_status( String( data="NOT READY" ) )
        self.update_recorded_samples( String( data="0" ) )

        # Reset video label
        self.label_video.clear()
        self.label_video.setAlignment( QtCore.Qt.AlignRight | QtCore.Qt.AlignBottom )
        self.label_video.setScaledContents(True)
        self.label_video.setText( "DISCONNECTED" )

    def cleanup(self):
        self.node.destroy_node()

    def handle_ros(self):
        rclpy.spin_once(node=self.node, timeout_sec=0)

    def heartbeat_callback(self):
        # If we haven't received a new heartbeat message in the last 3 seconds, clear the video feed
        elapsed_ms = ( datetime.datetime.now() - self.last_heartbeat_time ).total_seconds() * 1000
        if( elapsed_ms > 3000 ):
            self.last_heartbeat_time = datetime.datetime.now()
            if self.heartbeat_received is False:
                self.label_video.clear()
                self.label_video.setText( "DISCONNECTED" )
            else:
                self.label_video.setText( "" )
        
            self.heartbeat_received = False
    
    # Message handlers
    def update_camera_status( self, msg ):
        self.camera_status = msg.data
        self.label_camera_status.setText( msg.data )
    
    def update_gps_status( self, msg ):
        self.gps_status = msg.data
        self.label_gps_status.setText( msg.data )

    def update_storage_status( self, msg ):
        self.storage_status = msg.data
        self.label_storage_status.setText( msg.data )

    def update_system_time( self, msg ):
        self.system_time = msg.data
        self.label_system_time.setText( msg.data )

    def update_recording_status( self, msg ):
        self.recording_status = msg.data
        self.label_recording_status.setText( msg.data )

    def update_recorded_samples( self, msg ):
        self.recorded_samples = msg.data
        self.label_recorded_samples.setText( msg.data )

    def update_heartbeat( self, msg ):
        self.heartbeat_received = True

    def update_video_image( self, msg ):
        qp = QPixmap()
        if qp.loadFromData(msg.data):
            self.label_video.setPixmap(qp)
    

def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)

    app = QtWidgets.QApplication(sys.argv)
    window = Ui()
    app.exec_()
    window.cleanup()

    rclpy.shutdown()