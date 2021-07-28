
#include <iostream>
#include <memory>
#include <string>

#include "lx16a/serial.hpp"

using namespace lx16a;

Serial::Serial(std::string device_name, unsigned int baudrate) {

  this->device_name = device_name;
  this->baudrate = baudrate;

  this->io_service = std::make_unique<boost::asio::io_service>();
  this->serial_port = std::make_unique<boost::asio::serial_port>(
      boost::asio::serial_port(*this->io_service));
}

bool Serial::connect() {

  try {
    this->serial_port->open(this->device_name);

    boost::asio::serial_port_base::flow_control flowCrtl(
        boost::asio::serial_port_base::flow_control::none);

    this->serial_port->set_option(flowCrtl);

    unsigned int baudrate = 0;

    // setting baudrate
    this->serial_port->set_option(
        boost::asio::serial_port_base::baud_rate(baudrate));

    // charsize
    unsigned int charsize = 8;

    boost::asio::serial_port_base::stop_bits stopBits_1(
        boost::asio::serial_port_base::stop_bits::one);
    boost::asio::serial_port_base::parity parityBits_0(
        boost::asio::serial_port_base::parity::none);
    this->serial_port->set_option(stopBits_1);
    this->serial_port->set_option(parityBits_0);

    this->serial_port->set_option(
        boost::asio::serial_port_base::character_size(charsize));

  } catch (boost::system::system_error &error) {
    return false;
  }

  return true;
}

bool Serial::receive(unsigned char &receive_data) {
  this->serial_port->read_some(boost::asio::buffer(&receive_data, 1));
  return true;
}

bool Serial::receive(std::vector<uint8_t> &receive_data) {
  this->serial_port->read_some(
      boost::asio::buffer(&receive_data.front(), receive_data.size()));
  return true;
}

bool Serial::transmit(unsigned char &data) {
  this->serial_port->write_some(boost::asio::buffer(&data, 1));
  return true;
}

bool Serial::transmit(const std::vector<uint8_t> &data) {
  this->serial_port->write_some(
      boost::asio::buffer(&data.front(), data.size()));
  return true;
}