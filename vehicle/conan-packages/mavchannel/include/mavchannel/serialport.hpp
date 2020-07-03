#pragma once

#include <serial/serial.h>

namespace mr
{

class SerialPort {
public:
    SerialPort( const std::string& device_path_in, uint32_t baudrate_in, uint32_t timeout_ms_in );

    size_t read( uint8_t* data_out, size_t length );
    size_t write( uint8_t* data_in, size_t length );

    size_t available();
    void flush();

    std::string device_path() const;
    uint32_t baudrate() const;
    uint32_t timeout_ms() const;

private:
    const std::string   _device_path;
    const uint32_t      _baudrate;
    const uint32_t      _timeout_ms;

    serial::Serial      _device;

};

}