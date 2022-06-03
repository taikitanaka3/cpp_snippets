/*
 * reciever.cpp
 *
 *  Created on: Dec 2, 2019
 *      Author: tanakasan
 */

#include <stdio.h>
#include <string.h>
#include "udp.h"
simple_udp udp0("0.0.0.0",4001);

int main(int argc, char **argv){
  udp0.udp_bind();
  while (1){
    std::string rdata=udp0.udp_recv();
    printf("recv:%s\n", rdata.c_str());
  }
  return 0;
}


