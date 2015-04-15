/*
 * This file is part of the ssl-sim project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * This software is provided 'as-is', without any express or implied warranty.
 * In no event will the authors be held liable for any damages arising from the
 * use of this software.  Permission is granted to anyone to use this software
 * for any purpose, including commercial applications, and to alter it and
 * redistribute it freely, subject to the following restrictions:
 */

#include "net.h"

// TODO: Windows implementation for better cross-platform

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include <string>

static const unsigned int ONE = 1;
static const unsigned char TTL = 3;

namespace priv {
struct SocketImpl {
  int sockfd;
  struct sockaddr_in saddr;
  struct ip_mreq mreq;

  SocketImpl() {
    memset(&saddr, 0, sizeof(struct sockaddr_in));
    memset(&mreq, 0, sizeof(struct ip_mreq));
  }
};
}

Socket::Socket(int port, std::string addr, std::string iface)
    : i(new priv::SocketImpl) {
  // open a UDP socket
  if ((i->sockfd = socket(PF_INET, SOCK_DGRAM, IPPROTO_IP)) < 0)
    perror("Could not open socket file descriptor");

  i->saddr.sin_family = PF_INET;
  i->saddr.sin_port = htons(port);
  // bind socket to the multicast address if present
  i->saddr.sin_addr.s_addr =
      (addr.empty() ? htonl(INADDR_ANY) : inet_addr(addr.c_str()));

  i->mreq.imr_multiaddr.s_addr = inet_addr(addr.c_str());
  i->mreq.imr_interface.s_addr =
      (iface.empty() ? htonl(INADDR_ANY) : inet_addr(iface.c_str()));
}

Socket::~Socket() {
  // shutdown socket
  shutdown(i->sockfd, 2);

  // close socket
  close(i->sockfd);

  delete i;
}

bool Socket::receiver_bind() {
  // enable address reuse
  if (setsockopt(i->sockfd, SOL_SOCKET, SO_REUSEADDR, &ONE, sizeof(ONE)) < 0) {
    perror("Could not enable address reuse");
    return false;
  }

  // bind to interface
  if (bind(i->sockfd, (struct sockaddr *)&i->saddr, sizeof(i->saddr)) < 0) {
    perror("Could not bind socket to interface");
    return false;
  }

  // join multicast group
  if (setsockopt(i->sockfd, IPPROTO_IP, IP_ADD_MEMBERSHIP,
                 (const void *)&i->mreq, sizeof(i->mreq)) < 0) {
    perror("Could not join to multicast");
    return false;
  }

  return true;
}

int Socket::receive(char *into_buffer, int buffer_size) {
  socklen_t socklen = sizeof(struct sockaddr_in);

  // receive packet from socket
  int len;
  if ((len = recvfrom(i->sockfd, into_buffer, buffer_size - 1, 0,
                      (struct sockaddr *)&i->saddr, &socklen)) < 0)
    perror("Could not receive from socket");

  into_buffer[len] = 0;
  return len;
}

bool Socket::sender_bind() {
  struct sockaddr_in saddr;
  saddr.sin_family = PF_INET;
  saddr.sin_port = htons(0);                 // Use the first free port
  saddr.sin_addr.s_addr = htonl(INADDR_ANY); // bind socket to any address

  if (bind(i->sockfd, (struct sockaddr *)&saddr, sizeof(struct sockaddr_in)) <
      0) {
    perror("Error binding socket to interface");
    return false;
  }

  struct in_addr iaddr;
  memset(&iaddr, 0, sizeof(struct in_addr));
  iaddr.s_addr = INADDR_ANY; // use DEFAULT interface

  // Set the outgoing interface to DEFAULT
  if (setsockopt(i->sockfd, IPPROTO_IP, IP_MULTICAST_IF, &iaddr,
                 sizeof(struct in_addr)) < 0) {
    perror("Could not set outgoing interface");
    return false; // XXX: maybe some of these errors can be ignored?
  }

  // Set multicast packet TTL to 3; default TTL is 1
  if (setsockopt(i->sockfd, IPPROTO_IP, IP_MULTICAST_TTL, &TTL, sizeof(TTL)) <
      0) {
    perror("Could not set multicast packet TTL");
    return false;
  }

  // send multicast traffic to myself too
  if (setsockopt(i->sockfd, IPPROTO_IP, IP_MULTICAST_LOOP, &ONE, sizeof(ONE)) <
      0) {
    perror("Could not enable multicast loop to self");
    return false;
  }

  return true;
}

int Socket::send(char *from_buffer, int send_size) {
  socklen_t socklen = sizeof(struct sockaddr_in);
  int len = sendto(i->sockfd, from_buffer, send_size, 0,
                   (struct sockaddr *)&i->saddr, socklen);
  return len;
}

#if 0
int main() {
  Socket socket(5007, "224.1.1.1");
  socket.receiver_bind();

  char buffer[10240];
  socket.receive(buffer, 10240);
  puts(buffer);
}
#endif

#if 0
int main() {
  Socket socket(5007, "224.1.1.1");
  socket.sender_bind();

  std::string text = "Hurray!!!";
  socket.send(const_cast<char *>(text.c_str()), text.size());
}
#endif
