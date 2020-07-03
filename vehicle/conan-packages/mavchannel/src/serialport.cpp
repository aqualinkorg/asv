#include <mavchannel/serialport.hpp>

namespace mr {

SerialPort::SerialPort( const std::string& device_path_in, uint32_t baudrate_in, uint32_t timeout_ms_in )
    : _device_path{ device_path_in }
    , _baudrate{ baudrate_in }
    , _timeout_ms{ timeout_ms_in }
    , _device{ device_path_in, baudrate_in, serial::Timeout::simpleTimeout(timeout_ms_in ) }
{
}

size_t SerialPort::read( uint8_t* data_out, size_t length )
{
    return _device.read( data_out, length );
}

size_t SerialPort::write( uint8_t* data_in, size_t length )
{
    return _device.write( data_in, length );
}

size_t SerialPort::available()
{
    return _device.available();
}

void SerialPort::flush()
{
    _device.flushInput();
    _device.flushOutput();
}

std::string SerialPort::device_path() const
{
    return _device_path;
}

uint32_t SerialPort::baudrate() const
{
    return _baudrate;
}

uint32_t SerialPort::timeout_ms() const
{
    return _timeout_ms;
}
    
}

    