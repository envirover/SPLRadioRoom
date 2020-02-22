/*
 MAVLinkTCPChannelTest.cc

 MAVIO MAVLink I/O library

 (C) Copyright 2019 Envirover.

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#include <iostream>

#include "MAVLinkTCPChannel.h"

using std::cout;
using std::endl;

using mavio::MAVLinkTCPChannel;

int main(int argc, char** argv) {
  cout << "MAVLinkTCPChannel class test." << endl;

  for (int i = 0; i < argc; i++) {
    cout << argv[i] << endl;
  }

  if (argc < 3) {
    cout << "Usage: tcptest <host> <port>" << endl;
    return 1;
  }

  std::string host = argv[1];
  uint16_t port = atoi(argv[2]);

  cout << "Testing TCP channel " << host << ":" << port << "..." << endl;

  MAVLinkTCPChannel tcp_channel;

  if (tcp_channel.init(host, port)) {
      cout << "TCP channel init() succeeded." << endl;
      tcp_channel.close();
  } else {
      cout << "TCP channel init() failed." << endl;
      return 1;
  }

  return 0;
}
