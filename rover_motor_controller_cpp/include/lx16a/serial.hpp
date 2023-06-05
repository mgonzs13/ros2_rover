// Copyright (C) 2023  Miguel Ángel González Santamarta

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

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
