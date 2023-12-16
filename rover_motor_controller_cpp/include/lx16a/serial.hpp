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

#ifndef SERIAL_HPP
#define SERIAL_HPP

#include <memory>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/system/system_error.hpp>

namespace lx16a {

class Serial {
public:
  Serial(std::string device_name, unsigned int baud_rate);
  bool connect();

  bool read(unsigned char &receive_data);
  bool read_with_timeout(unsigned char &receive_data, int timeout = 3);
  void time_out(const boost::system::error_code &error);
  void read_complete(bool &read_error, const boost::system::error_code &error,
                     size_t bytes_transferred);

  bool write(unsigned char &data);
  bool write(const std::vector<uint8_t> &data);

private:
  std::string device_name;
  unsigned int baud_rate;
  std::unique_ptr<boost::asio::io_service> io_service;
  std::unique_ptr<boost::asio::serial_port> serial_port;
  std::unique_ptr<boost::asio::deadline_timer> timer;
};

} // namespace lx16a
#endif
