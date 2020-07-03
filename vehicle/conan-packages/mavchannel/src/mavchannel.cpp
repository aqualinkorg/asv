#include "mavchannel/mavchannel.hpp"

#include <mavlink2/common/mavlink.h>
#include <iostream>

uint64_t timeSinceEpochMillisec() {
  using namespace std::chrono;
  return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}



namespace mr
{

Mavchannel::Mavchannel( const std::string& device_path_in, uint32_t baudrate_in, uint32_t timeout_ms_in )
    : _serial_port{ device_path_in, baudrate_in, timeout_ms_in }
    , _terminate{ false }
    , _connected{ false }
    , _reader_queue{ READER_QUEUE_SIZE }
    , _writer_queue{ WRITER_QUEUE_MIN_SIZE }
    , _mavlink_channel{ 0 }
    , _drop_count{ 0 }
    , _bad_crc_count{ 0 }
    , _bad_signature_count{ 0 }
{
}

Mavchannel::~Mavchannel()
{
    disconnect();
}

void Mavchannel::connect()
{
    if( _connected )
    {
        throw std::runtime_error( "Already connected!" );
    }
    else if( !_on_data )
    {
        throw std::runtime_error( "No Data Handler!" );
    }
    else
    {
        // Create Reader Thread
        _reader_thread = std::thread{ [&](){
            reader_thread_method();
        }};

        // Create Writer Thread
        _writer_thread = std::thread{ [&](){
            writer_thread_method();
        }};

        _connected = true;
    }
}

void Mavchannel::disconnect()
{
    if( _connected )
    {
        try
        {
            terminate_threads();
        }
        catch(...)
        {
            // TODO: Handle this properly.
        }

        _connected = false;
    }
}

size_t Mavchannel::available()
{
    return _reader_queue.size_approx();
}

void Mavchannel::reader_thread_method()
{
    uint8_t next_byte = 0;

    // First flush the serial port
    _serial_port.flush();

    while( !_terminate )
    {
        if( _serial_port.read( &next_byte, 1 ) )
        {
            uint8_t ret = mavlink_parse_with_checks( _mavlink_channel, next_byte, &_incoming_message, &_status );
            switch( ret )
            {
                case MAVLINK_FRAMING_OK:
                {
                    if( _reader_queue.try_enqueue( _incoming_message ) )
                    {
                        _on_data();
                    }
                    else
                    {
                        std::cout << "Queue full" << std::endl;
                        _drop_count++;
                    }
                    
                    break;
                }

                case MAVLINK_FRAMING_BAD_CRC:       { std::cout << "Got bad crc" << std::endl; _bad_crc_count++;         break; }
                case MAVLINK_FRAMING_BAD_SIGNATURE: { std::cout << "Got bad sig" << std::endl; _bad_signature_count++;   break; }

                // This is normal
                case MAVLINK_FRAMING_INCOMPLETE:
                default:
                {
                    break;
                }
            }
        }
    }
}

void Mavchannel::writer_thread_method()
{
    mavlink_message_t outbound_message;
    std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> buffer;
    size_t length = 0;

    while( !_terminate )
    {
        // Wait for a message to send to the serial port
        if( _writer_queue.wait_dequeue_timed( outbound_message, std::chrono::milliseconds{ 100 } ) )
        {
            // Copy to buffer
            length = mavlink_msg_to_send_buffer( buffer.data(), &outbound_message );

            // Send over serial port
            size_t bytes_sent = _serial_port.write( buffer.data(), length );
            if( bytes_sent != length )
            {
                std::cout << "send error" << std::endl;
                // ERROR
                // TODO: Not sure how to best handle this just yet. 
                // Maybe a way to flush the line with a reset sequence to help the remote device.
            }
            else
            {
            }
            
        }
    }
}

void Mavchannel::terminate_threads()
{
    _terminate = true;
    if( _reader_thread.joinable() ){ _reader_thread.join(); }
    if( _writer_thread.joinable() ){ _writer_thread.join(); }
}


uint8_t Mavchannel::mavlink_parse_with_checks(    uint8_t channel, 
                                                            uint8_t next_byte, 
                                                            mavlink_message_t* message_out, 
                                                            mavlink_status_t* status_out )
{
    uint8_t msg_state = mavlink_frame_char( channel, next_byte, message_out, status_out );
    if( MAVLINK_FRAMING_BAD_CRC == msg_state || MAVLINK_FRAMING_BAD_SIGNATURE == msg_state )
    {
        mavlink_message_t* message  = mavlink_get_channel_buffer( channel );
        mavlink_status_t* status    = mavlink_get_channel_status( channel );
        
        status->parse_error++;
        status->msg_received    = msg_state;
        status->parse_state     = MAVLINK_PARSE_STATE_IDLE;
        if( MAVLINK_STX == next_byte )
        {
            status->parse_state = MAVLINK_PARSE_STATE_GOT_STX;
            message->len = 0;
            mavlink_start_checksum( message );
        }

        // This is the only real change. Normally it returns 0, for some reason
        return msg_state;
    }

    return msg_state;
}

bool Mavchannel::read( mavlink_message_t& message_out )
{
    return _reader_queue.try_dequeue( message_out );
}

bool Mavchannel::write( const mavlink_message_t& message_in )
{
    return _writer_queue.try_enqueue( message_in );
}

void Mavchannel::register_on_data( std::function<void()> handler_in )
{
    _on_data = handler_in;
}

}