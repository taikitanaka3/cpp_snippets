/*
 * udp.h
 *
 *  Created on: Dec 2, 2019
 *      Author: tanakasan
 */

#ifndef ECLIPS_CPP__COMMUNICATORTEST__UDP_H_
#define ECLIPS_CPP__COMMUNICATORTEST__UDP_H_

#include <arpa/inet.h>
#include <iostream>
#include <netinet/in.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

class simple_udp {
  int sock;
  struct sockaddr_in addr;

public:
  simple_udp(std::string address, int port) {
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(address.c_str());
    addr.sin_port = htons(port);
  }
  void udp_send(std::string word) {
    sendto(sock, word.c_str(), word.length(), 0, (struct sockaddr *)&addr,
           sizeof(addr));
  }

  void udp_bind() { bind(sock, (const struct sockaddr *)&addr, sizeof(addr)); }
  std::string udp_recv() {
#define BUFFER_MAX 400
    char buf[BUFFER_MAX];
    memset(buf, 0, sizeof(buf));
    recv(sock, buf, sizeof(buf), 0);
    return std::string(buf);
  }
  void udp_recv(char *buf, int size) {
    memset(buf, 0, size);
    recv(sock, buf, size, 0);
  }

  ~simple_udp() { close(sock); }
};

#endif // ECLIPS_CPP__COMMUNICATORTEST__UDP_H_
