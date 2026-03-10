#ifndef __SERIAL_HELPER_H
#define __SERIAL_HELPER_H

#include <atomic>
#include <functional>
#include <libserial/SerialPort.h>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
class SerialHelper {
public:
  using Data = std::vector<uint8_t>;
  using ConnectedCallback = std::function<void(bool)>;
  using DataReceived = std::function<void(const Data &)>;
  std::mutex &lock;
  std::string port;
  LibSerial::BaudRate baudrate;
  LibSerial::CharacterSize bytesize;
  LibSerial::Parity parity;
  LibSerial::StopBits stopbits;
  int threshold_value = 1;
  std::atomic<uint32_t> current_height;
  Data receive_data;
  SerialHelper(std::mutex &lock, std::string Port = "/dev/ttyUSB0",
               uint32_t BaudRate = 115200, uint8_t ByteSize = 8,
               char Parity = 'N', uint8_t Stopbits = 1);
  ~SerialHelper();
  void connect(/* uint16_t timeout = 2 */);
  void disconnect();
  void write(const Data &data /* , bool isHex = false */);
  void on_connected_changed(const ConnectedCallback &func);
  void on_data_received(const DataReceived &func);
  Data hex_to_bytes(const std::string &hex);

private:
  LibSerial::SerialPort _serial;
  std::atomic<bool> _is_connected;
  std::atomic<bool> _is_connected_temp;
  std::atomic<bool> _running{true};
  std::thread _connect_thread;
  std::thread _recv_thread;
  void _on_connected_changed(const ConnectedCallback &func);
  void _on_data_received(const DataReceived &func);
  // std::vector<std::string> find_usb_tty(uint16_t vendor_id = 0, uint16_t
  // product_id = 0);
};

#endif