#include <recorder/recorder_node.hpp>

#include <cstdlib>
#include <fstream>
#include <sstream>
#include <thread>
#include <chrono>

#include <boost/filesystem.hpp>

#include <turbojpeg.h>

namespace qos {
    const rclcpp::QoS BEST_EFFORT                           = rclcpp::QoS( rclcpp::KeepLast( 1 ) ).best_effort().durability_volatile();
    const rclcpp::QoS KEEP_LAST_RELIABLE_VOLATILE           = rclcpp::QoS( rclcpp::KeepLast( 1 ) ).reliable().durability_volatile();
    const rclcpp::QoS KEEP_LAST_RELIABLE_TRANSIENT_LOCAL    = rclcpp::QoS( rclcpp::KeepLast( 1 ) ).reliable().transient_local();
    const rclcpp::QoS DEFAULT                               = BEST_EFFORT;
}

RecorderNode::RecorderNode()
    : Node( "recorder" )
    , _minimum_sample_period_msecs( 2000 )
    , _storage_path( "/tmp" )
    , _data_left{ new uint8_t[ 2208 * 1242 * 4 ] }
    , _data_right{ new uint8_t[ 2208 * 1242 * 4 ] }
    , _data_depth_image{ new uint8_t[ 2208 * 1242 * 4 ] }       // 8 bit
    , _data_depth_raw{ new uint8_t[ 2208 * 1242 * 4 ] }         // 32 bit
    , _heartbeat_count{ 0 }
    , _pub_heartbeat{ create_publisher<std_msgs::msg::String>( TOPIC_HEARTBEAT, qos::KEEP_LAST_RELIABLE_VOLATILE ) }
    , _pub_camera_status{ create_publisher<std_msgs::msg::String>( TOPIC_CAMERA_STATUS, qos::KEEP_LAST_RELIABLE_TRANSIENT_LOCAL ) }
    , _pub_gps_status{ create_publisher<std_msgs::msg::String>( TOPIC_GPS_STATUS, qos::KEEP_LAST_RELIABLE_TRANSIENT_LOCAL ) }
    , _pub_storage_status{ create_publisher<std_msgs::msg::String>( TOPIC_STORAGE_STATUS, qos::KEEP_LAST_RELIABLE_TRANSIENT_LOCAL ) }
    , _pub_system_time{ create_publisher<std_msgs::msg::String>( TOPIC_SYSTEM_TIME, qos::KEEP_LAST_RELIABLE_TRANSIENT_LOCAL ) }
    , _pub_recording_status{ create_publisher<std_msgs::msg::String>( TOPIC_RECORDING_STATUS, qos::KEEP_LAST_RELIABLE_TRANSIENT_LOCAL ) }
    , _pub_recorded_samples{ create_publisher<std_msgs::msg::String>( TOPIC_RECORDED_SAMPLES, qos::KEEP_LAST_RELIABLE_TRANSIENT_LOCAL ) }
    , _pub_camera_left{ create_publisher<sensor_msgs::msg::CompressedImage>( TOPIC_CAMERA_LEFT, qos::BEST_EFFORT ) }
    , _pub_camera_trigger{ create_publisher<std_msgs::msg::UInt32>( TOPIC_CAMERA_TRIGGER, qos::KEEP_LAST_RELIABLE_VOLATILE ) }
    , _scheduler{ std::chrono::microseconds{ 100 * 1000 } }
{
    // Immediately publish initial status messages and heartbeat to signal a connection and clear any previous values
    publish_initial_status();

    initialize_parameters();

    create_subscriptions();

    // Start heartbeat timer
    _timer_hb = this->create_wall_timer(
        std::chrono::milliseconds( 1000 ),
        [this](){
            std_msgs::msg::String status;
            status.data = std::to_string( _heartbeat_count++ );
            _pub_heartbeat->publish( status );

            spdlog::info( "Sending heartbeat" );

            // Check if GPS was lost
            if( !( _gps_samples > _gps_samples_last_hb ) )
            {
                std_msgs::msg::String status;
                status.data = "UNKNOWN";
                _pub_gps_status->publish( status );
            }

            _gps_samples_last_hb = _gps_samples;
        }
    );

    // Initialize storage system
    try 
    {
        initialize_storage();

        std_msgs::msg::String status;
        status.data = "OK";
        _pub_storage_status->publish( status );
    }
    catch( const std::exception& e )
    {
        std_msgs::msg::String status;
        status.data = "ERROR";
        _pub_storage_status->publish( status );

        std::this_thread::sleep_for( std::chrono::milliseconds( 500 ) );
        throw;
    }

    // Connect to the ZED camera
    try 
    {
        _cam_params.sdk_verbose         = true;
        _cam_params.camera_resolution   = sl::RESOLUTION::HD2K;

        sl::ERROR_CODE err = _camera.open( _cam_params );
        if (err != sl::ERROR_CODE::SUCCESS) {
            _camera.close();
            throw std::runtime_error( "Failed to open ZED camera: " + std::to_string( static_cast<int>( err ) ) );
        }

        std_msgs::msg::String status;
        status.data = "OK";
        _pub_camera_status->publish( status );
    }
    catch( const std::exception& e )
    {
        std_msgs::msg::String status;
        status.data = "ERROR";
        _pub_camera_status->publish( status );

        std::this_thread::sleep_for( std::chrono::milliseconds( 500 ) );
        throw;
    }

    // Resize jpeg buffer to hold max size of jpeg frame
    _jpeg_buffer.resize( static_cast<size_t>( tjBufSize( 2208, 1242, TJSAMP_444 ) ) );

    _last_pub_time = std::chrono::steady_clock::now();
}

RecorderNode::~RecorderNode()
{
    _camera.close();
}

void RecorderNode::publish_initial_status()
{
    std_msgs::msg::String status;

    status.data = "UNKNOWN";
    _pub_camera_status->publish( status );
    _pub_gps_status->publish( status );
    _pub_storage_status->publish( status );
    _pub_system_time->publish( status );

    status.data = "NOT READY";
    _pub_recording_status->publish( status );

    status.data = "0";
    _pub_recorded_samples->publish( status );
    _pub_heartbeat->publish( status );

    std::this_thread::sleep_for( std::chrono::milliseconds( 200 ) );
}

void RecorderNode::initialize_parameters()
{
    rcl_interfaces::msg::ParameterDescriptor format_description_sample_period; 
    format_description_sample_period.name = "minimum_sample_period_msecs";
    format_description_sample_period.type=rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER; 
    format_description_sample_period.description = "Minimum allowed time between recording data samples from camera"; 
    format_description_sample_period.read_only = true; 
    format_description_sample_period.additional_constraints = "Supported values: [0:UINT32_MAX]";

    rcl_interfaces::msg::ParameterDescriptor format_description_storage_path; 
    format_description_storage_path.name = "storage_path";
    format_description_storage_path.type=rcl_interfaces::msg::ParameterType::PARAMETER_STRING; 
    format_description_storage_path.description = "Directory to store data samples in"; 
    format_description_storage_path.read_only = true; 
    format_description_storage_path.additional_constraints = "Supported values: [any valid path with proper permissions]";

    // Declare parameters
    int64_t period = static_cast<int64_t>(_minimum_sample_period_msecs);
    this->declare_parameter<int64_t>("minimum_sample_period_msecs", period, format_description_sample_period );
    this->declare_parameter<std::string>("storage_path", _storage_path, format_description_storage_path );

    // Get configuration from param file or use defaults
    this->get_parameter<uint32_t>("minimum_sample_period_msecs", _minimum_sample_period_msecs);
    this->get_parameter<std::string>("storage_path", _storage_path);
}

void RecorderNode::initialize_storage()
{
    boost::filesystem::path storage_path( _storage_path );

    // Check existence of storage path specified in config (usually your USB drive)
    if( !boost::filesystem::is_directory( storage_path ) )
    {
        throw std::runtime_error( "NOT FOUND" );
    }

    boost::filesystem::path dir( "openasv_data" );
    auto data_path = ( storage_path / dir );

    // Create ASV data directory if it doesn't exist already.
    if( !boost::filesystem::is_directory( data_path ) )
    {
        if( !boost::filesystem::create_directories( data_path ) ) 
        {
            throw std::runtime_error( "PERMISSION ERROR" );
        }
    }
    
    // Test permissions to make sure data dir is writable, even if it already existed
    bool create_test = boost::filesystem::create_directory( data_path / "temp" );
    bool remove_test = boost::filesystem::remove_all( data_path / "temp" );
    if( !( create_test || remove_test ) )
    {
        throw std::runtime_error( "PERMISSION ERROR" );
    }

    _data_dir = data_path.string();
}

void RecorderNode::create_subscriptions()
{
    // Time
    auto cb_time = [this]( const std_msgs::msg::UInt64::SharedPtr msg ) -> void
    {
        spdlog::info( "Got time" );

        // Only set this once
        if( _got_time == false )
        {
            _time = msg->data;
            _got_time = true;

            std_msgs::msg::String status;
            status.data = "OK";
            _pub_system_time->publish( status );
        }
    };

    // GPS
    auto cb_gps = [this]( const sensor_msgs::msg::NavSatFix::SharedPtr msg ) -> void
    {
        spdlog::info( "Got gps" );
        _gps_samples++;

        // Check for fix
        if( msg->status.status >= 0 )
        {
            // Set internal state
            _got_gps    = true;
            _lat        = msg->latitude;
            _long       = msg->longitude;

            std_msgs::msg::String status;
            status.data = "OK";
            _pub_gps_status->publish( status );
        }
        else
        {
            std_msgs::msg::String status;
            status.data = "NO FIX";
            _pub_gps_status->publish( status );

            status.data = "NOT READY";
            _pub_recording_status->publish( status );
        }

        update();
    };

    // Attitude
    auto cb_attitude = [this]( const geometry_msgs::msg::Vector3Stamped::SharedPtr msg ) -> void
    {
        spdlog::info( "Got attitude" );
        _roll   = msg->vector.x;
        _pitch  = msg->vector.y;
        _yaw    = msg->vector.z;
    };

    // Speed
    auto cb_speed = [this]( const geometry_msgs::msg::Vector3Stamped::SharedPtr msg ) -> void
    {
        spdlog::info( "Got speed" );
        _speed = msg->vector.z;
    };

    // Enable
    auto cb_enable = [this]( const std_msgs::msg::Bool::SharedPtr msg ) -> void
    {
        spdlog::info( "Got Enable" );
        _is_enabled = msg->data;
    };

    _sub_time               = create_subscription<std_msgs::msg::UInt64>( TOPIC_TIME, qos::BEST_EFFORT, cb_time );
    _sub_gps                = create_subscription<sensor_msgs::msg::NavSatFix>( TOPIC_GPS, qos::KEEP_LAST_RELIABLE_TRANSIENT_LOCAL, cb_gps );
    _sub_attitude           = create_subscription<geometry_msgs::msg::Vector3Stamped>( TOPIC_ATTITUDE, qos::KEEP_LAST_RELIABLE_TRANSIENT_LOCAL, cb_attitude );
    _sub_speed              = create_subscription<geometry_msgs::msg::Vector3Stamped>( TOPIC_SPEED, qos::KEEP_LAST_RELIABLE_TRANSIENT_LOCAL, cb_speed );
    _sub_enable_recording   = create_subscription<std_msgs::msg::Bool>( TOPIC_ENABLE_RECORDING, qos::KEEP_LAST_RELIABLE_TRANSIENT_LOCAL, cb_enable );
}

void RecorderNode::update()
{
    // Limit frame capture rate to specified rate
    auto now = std::chrono::steady_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>( now - _last_pub_time ).count();
    if( elapsed_ms >= _minimum_sample_period_msecs )
    {
        // Take camera capture
        spdlog::info( "Capturing frame" );
        sl::ERROR_CODE err = _camera.grab();
        if( err == sl::ERROR_CODE::SUCCESS ) 
        {
            spdlog::info( "Got frame" );
            uint8_t* data = nullptr;

            size_t raw_image_size = 2208 * 1242 * 4;
            size_t raw_depth_size = 2208 * 1242 * 4;

            // Retrieve left image
            _camera.retrieveImage( _image, sl::VIEW::LEFT );
            data = _image.getPtr<uint8_t>();
            std::memcpy( _data_left.get(), data, _image.getWidth() * _image.getHeight() * _image.getPixelBytes() );

            // Retrieve right image
            _camera.retrieveImage( _image, sl::VIEW::RIGHT );
            data = _image.getPtr<uint8_t>();
            std::memcpy( _data_right.get(), data, _image.getWidth() * _image.getHeight() * _image.getPixelBytes() );

            // Get depth raw
            _camera.retrieveMeasure( _depth_raw, sl::MEASURE::DEPTH );
            data = _depth_raw.getPtr<uint8_t>();
            std::memcpy( _data_depth_raw.get(), data, _depth_raw.getWidth() * _depth_raw.getHeight() * _depth_raw.getPixelBytes() );

            // Get depth image
            _camera.retrieveImage( _depth_image, sl::VIEW::DEPTH );
            data = _depth_image.getPtr<uint8_t>();
            std::memcpy( _data_depth_image.get(), data, _depth_image.getWidth() * _depth_image.getHeight() * _depth_image.getPixelBytes() );
            
            // Format data
            auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::system_clock::now().time_since_epoch() ).count();

            // When a GPS sample comes in
            if( _got_gps )
            {
                _got_gps = false;

                // Write to disk when enabled and time has been acquired from GPS
                if( _got_time )
                {
                    if( _is_enabled )
                    {
                        std_msgs::msg::String status;
                        status.data = "RECORDING";
                        _pub_recording_status->publish( status );

                        std::stringstream ss;
                        ss.precision( 7 );
                        ss  << "{\n"
                            <<      "  \"systime_ms\": " << timestamp << "\n"
                            <<      "  \"frame\": " << _frame_number << "\n"
                            <<      "  \"lat\": " << _lat << "\n"
                            <<      "  \"lon\": " << _long << "\n"
                            <<      "  \"speed\": " << _speed << "\n"
                            <<      "  \"roll\": " << _roll << "\n"
                            <<      "  \"pitch\": " << _pitch << "\n"
                            <<      "  \"yaw\": " << _yaw << "\n"
                            <<  "}";

                        std::string file_prefix = _data_dir + "/" + std::to_string( _time ) + "_"; 

                        std::ofstream out_left( file_prefix + std::to_string( _frame_number ) + "_left.rgba", std::ios::out | std::ios::binary); 
                        out_left.write( reinterpret_cast<char*>( _data_left.get() ), static_cast<long>( raw_image_size ) );

                        std::ofstream out_right( file_prefix + std::to_string( _frame_number ) + "_right.rgba", std::ios::out | std::ios::binary); 
                        out_right.write( reinterpret_cast<char*>( _data_right.get() ), static_cast<long>( raw_image_size ) );

                        std::ofstream out_depth_raw( file_prefix + std::to_string( _frame_number ) + "_depth.dat", std::ios::out | std::ios::binary); 
                        out_depth_raw.write( reinterpret_cast<char*>( _data_depth_raw.get() ), static_cast<long>( raw_depth_size ) );

                        // For debugging only
                        // size_t depth_image_size = 2208 * 1242 * 4;
                        // std::ofstream out_depth_image( file_prefix + std::to_string( _frame_number ) + "_depth.gray", std::ios::out | std::ios::binary); 
                        // out_depth_image.write( reinterpret_cast<char*>( _data_depth_image.get() ), static_cast<long>( depth_image_size ) );

                        std::ofstream out_data( file_prefix + std::to_string( _frame_number ) + "_data.json", std::ios::out ); 
                        out_data << ss.rdbuf();

                        _recorded_samples++;

                        std_msgs::msg::String samples;
                        samples.data = std::to_string( _recorded_samples );
                        _pub_recorded_samples->publish( samples );

                        spdlog::info( "Wrote image[{}]", _frame_number );
                    }
                    else
                    {
                        std_msgs::msg::String status;
                        status.data = "READY";
                        _pub_recording_status->publish( status );
                    }
                }
                else
                {
                    std_msgs::msg::String status;
                    status.data = "NOT READY";
                    _pub_recording_status->publish( status );
                }
            }

            // Perform jpeg encoding
            size_t compressed_frame_size = 0;
            tjhandle jpeg_compressor = tjInitCompress();

            sensor_msgs::msg::CompressedImage image_msg;
            image_msg.header.stamp    = rclcpp::Clock().now();
            image_msg.format          = "jpeg";

            uint8_t* dest = _jpeg_buffer.data();

            // Publish Left frame
            tjCompress2( jpeg_compressor, _data_left.get(), 2208, 0, 1242, TJPF_BGRX,
                &dest, &compressed_frame_size, TJSAMP_444, 20,
                TJFLAG_NOREALLOC );
            image_msg.data.resize( compressed_frame_size );
            std::memcpy( image_msg.data.data(), _jpeg_buffer.data(), compressed_frame_size );
            _pub_camera_left->publish( image_msg );
            
            _frame_number++;
            _last_pub_time = std::chrono::steady_clock::now();
        }
        else 
        {
            spdlog::info( "Failed to grab image" );
        }
    }
}

void RecorderNode::write_data()
{

}