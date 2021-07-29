
#ifndef SERIAL_HPP
#define SERIAL_HPP

#include <memory>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/system/system_error.hpp>

namespace lx16a {

class Serial {
public:
  Serial(std::string device_name, unsigned int baudrate);
  bool connect();
  bool receive(unsigned char &receive_data);
  bool transmit(unsigned char &data);
  bool transmit(const std::vector<uint8_t> &data);

private:
  std::string device_name;
  unsigned int baudrate;
  std::unique_ptr<boost::asio::serial_port> serial_port;
  std::unique_ptr<boost::asio::io_service> io_service;
};

} // namespace lx16a
#endif
