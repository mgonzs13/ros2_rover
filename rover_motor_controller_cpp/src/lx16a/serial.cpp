// MIT License

// Copyright (c) 2023  Miguel Ángel González Santamarta

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <memory>
#include <string>

#include "lx16a/serial.hpp"

using namespace lx16a;

Serial::Serial(std::string device_name, unsigned int baud_rate) {

  this->device_name = device_name;
  this->baud_rate = baud_rate;

  this->io_service = std::make_unique<boost::asio::io_service>();
  this->serial_port = std::make_unique<boost::asio::serial_port>(
      boost::asio::serial_port(*this->io_service));
  this->timer =
      std::make_unique<boost::asio::deadline_timer>(*this->io_service);
}

bool Serial::connect() {

  try {
    this->serial_port->open(this->device_name);

    // flow control
    boost::asio::serial_port_base::flow_control flowCrtl(
        boost::asio::serial_port_base::flow_control::none);
    this->serial_port->set_option(flowCrtl);

    // setting baud_rate
    this->serial_port->set_option(
        boost::asio::serial_port_base::baud_rate(this->baud_rate));

    // charsize
    boost::asio::serial_port_base::stop_bits stopBits_1(
        boost::asio::serial_port_base::stop_bits::one);
    this->serial_port->set_option(stopBits_1);

    boost::asio::serial_port_base::parity parityBits_0(
        boost::asio::serial_port_base::parity::none);
    this->serial_port->set_option(parityBits_0);

    unsigned int charsize = 8;
    this->serial_port->set_option(
        boost::asio::serial_port_base::character_size(charsize));

  } catch (boost::system::system_error &error) {
    return false;
  }

  return true;
}

bool Serial::read(unsigned char &receive_data) {
  this->serial_port->read_some(boost::asio::buffer(&receive_data, 1));
  return true;
}

void Serial::read_complete(bool &read_error,
                           const boost::system::error_code &error,
                           size_t bytes_transferred) {

  read_error = (error || bytes_transferred == 0);
  this->timer->cancel();
}

void Serial::time_out(const boost::system::error_code &error) {

  if (error) {
    return;
  }

  this->serial_port->cancel();
}

bool Serial::read_with_timeout(unsigned char &receive_data, int timeout) {

  if (!timeout) {
    return this->read(receive_data);
  }

  unsigned char buffer[1];
  bool read_error = true;

  // asynchronously read 1 char
  this->serial_port->async_read_some(
      boost::asio::buffer(buffer, 1),
      boost::bind(&Serial::read_complete, this, boost::ref(read_error),
                  boost::asio::placeholders::error,
                  boost::asio::placeholders::bytes_transferred));

  // setup a deadline time to implement our timeout.
  this->timer->expires_from_now(boost::posix_time::seconds(timeout));
  this->timer->async_wait(
      boost::bind(&Serial::time_out, this, boost::asio::placeholders::error));

  // block until a character is read or until it is cancelled.
  this->io_service->run();

  // reset after a timeout and cancel
  this->io_service->reset();

  if (!read_error)
    receive_data = buffer[0];

  return !read_error;
}

bool Serial::write(unsigned char &data) {
  this->serial_port->write_some(boost::asio::buffer(&data, 1));
  return true;
}

bool Serial::write(const std::vector<uint8_t> &data) {
  this->serial_port->write_some(
      boost::asio::buffer(&data.front(), data.size()));
  return true;
}