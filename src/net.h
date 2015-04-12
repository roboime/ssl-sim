/*
 * This file is part of the ssl-sim project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef NET_H
#define NET_H

#include <string>

namespace priv {
struct SocketImpl;
}

class Socket {
  priv::SocketImpl *i;

public:
  Socket(int port, std::string addr = "", std::string iface = "");
  ~Socket();

  bool receiver_bind();
  int receive(char *into_buffer, int buffer_size);

  bool sender_bind();
  int send(char *from_buffer, int send_size);
};

#endif
