#pragma once

#include <thread>
#include <atomic>
#include <functional>

#include <readerwriterqueue/readerwriterqueue.h>
#include <concurrentqueue/blockingconcurrentqueue.h>

#include <mavlink2/ardupilotmega/mavlink.h>

#include "visibility_control.hpp"
#include "serialport.hpp"

namespace mr
{

class Mavchannel
{
public:
    static constexpr size_t READER_QUEUE_SIZE      = 128;
    static constexpr size_t WRITER_QUEUE_MIN_SIZE  = 128;

    Mavchannel( const std::string& device_path_in, uint32_t baudrate_in, uint32_t timeout_ms_in );
    virtual ~Mavchannel();

    bool write( const mavlink_message_t& message_in );      // Attempts to write message to queue
    bool read( mavlink_message_t& message_out );            // Attempts to read message from queue

    size_t available();                                     // Returns an estimate of number of messages in queue

    void connect();                                         // Creates the serial port object and starts threads
    void disconnect();                                      // Stops threads and destroys serial port object

    void register_on_data( std::function<void()> handler_in );

private:
    SerialPort              _serial_port;
    std::function<void()>   _on_data;

    std::atomic<bool>       _terminate;
    std::thread             _reader_thread;
    std::thread             _writer_thread;

    bool                    _connected;

    moodycamel::ReaderWriterQueue<mavlink_message_t>        _reader_queue;  // Threadsafe for Single Producer, Single Consumer
    moodycamel::BlockingConcurrentQueue<mavlink_message_t>  _writer_queue;  // Threadsafe for Multi Producer, Single Consumer

    // Data
    uint8_t                 _mavlink_channel;
    mavlink_message_t       _incoming_message;
    mavlink_status_t        _status;

    uint32_t                _drop_count;
    uint32_t                _bad_crc_count;
    uint32_t                _bad_signature_count;

    void reader_thread_method();
    void writer_thread_method();
    void terminate_threads();

    // Custom version of mavlink_parse_char() function that allows detection of CRC failures
    static uint8_t mavlink_parse_with_checks(    uint8_t channel, 
                                                    uint8_t next_byte, 
                                                    mavlink_message_t* message_out, 
                                                    mavlink_status_t* status_out );
};

}