#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP

#include <sstream>
#include <bitset>
#include <cstddef>
#include <libserial/SerialPort.h>
#include <iostream>
#include <cstring>
#include <jsoncpp/json/json.h>

#include <zlib.h>

LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
  case 1200:
    return LibSerial::BaudRate::BAUD_1200;
  case 1800:
    return LibSerial::BaudRate::BAUD_1800;
  case 2400:
    return LibSerial::BaudRate::BAUD_2400;
  case 4800:
    return LibSerial::BaudRate::BAUD_4800;
  case 9600:
    return LibSerial::BaudRate::BAUD_9600;
  case 19200:
    return LibSerial::BaudRate::BAUD_19200;
  case 38400:
    return LibSerial::BaudRate::BAUD_38400;
  case 57600:
    return LibSerial::BaudRate::BAUD_57600;
  case 115200:
    return LibSerial::BaudRate::BAUD_115200;
  case 230400:
    return LibSerial::BaudRate::BAUD_230400;
  default:
    std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
    return LibSerial::BaudRate::BAUD_57600;
  }
}

class ArduinoComms
{

public:
  ArduinoComms() = default;

  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
    serial_conn_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_conn_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    serial_conn_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
    serial_conn_.SetParity(LibSerial::Parity::PARITY_NONE);
  }

  void disconnect()
  {
    serial_conn_.Close();
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }

  bool read_hardware_states(std::string &str_out, bool print_output = false)
  {
    serial_conn_.FlushIOBuffers();
    std::string send_str = "150088878{\"topic\":\"ros2_state\"}\r\n";
    std::string read_str = "";
    serial_conn_.Write(send_str);
    try
    {
      serial_conn_.ReadLine(read_str, '\n', 100);
    }
    catch (const LibSerial::ReadTimeout &)
    {
      // std::cerr << "The ReadByte() call has timed out." << std::endl;
      // read_str = "read timeout!\n";
    }
    if (print_output)
    {
      std::cout << ">>> " << send_str;
      std::cout << "<<< " << read_str;
    }
    std::size_t startIndex = read_str.find('{');
    std::size_t stopIndex = read_str.find('}');

    if (startIndex != std::string::npos && stopIndex != std::string::npos)
    {
      uLong crc_value = 0;
      std::string crc_str = read_str.substr(0, startIndex);
      crc_value = std::stoul(crc_str);

      std::string str_to_parse = read_str.substr(startIndex, stopIndex - startIndex + 1);
      uLong crc_cal = crc32(0L, Z_NULL, 0);

      std::byte bytes[str_to_parse.length()];
      std::memcpy(bytes, str_to_parse.data(), str_to_parse.length());

      crc_cal = crc32(crc_cal, (const Bytef *)bytes, sizeof(bytes));
      if (crc_value == crc_cal)
      {
        str_out = str_to_parse;
        return true;
      }
    }
    return false;
  }

  std::string write_hardware_command(const std::string &msg_to_send, bool print_output = false)
  {
    uLong crc_cal = crc32(0L, Z_NULL, 0);

    std::byte bytes[msg_to_send.length()];
    std::memcpy(bytes, msg_to_send.data(), msg_to_send.length());

    crc_cal = crc32(crc_cal, (const Bytef *)bytes, sizeof(bytes));
    std::string msg_to_serial = std::to_string(crc_cal) + msg_to_send + "\r\n";
    serial_conn_.FlushOutputBuffer();
    serial_conn_.Write(msg_to_serial);

    std::string response = "";
    try
    {
      serial_conn_.ReadLine(response, '\n', 100);
    }
    catch (const LibSerial::ReadTimeout &)
    {
      // std::cerr << "The ReadByte() call has timed out." << std::endl;
      // response = "read timeout!\n";
    }

    if (print_output)
    {
      std::cout << ">>> " << msg_to_serial;
      std::cout << "<<< " << response;
    }

    return response;
  }

private:
  LibSerial::SerialPort serial_conn_;
  int timeout_ms_;
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP