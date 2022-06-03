/*
 * sender.cpp
 *
 *  Created on: Dec 2, 2019
 *      Author: tanakasan
 */




#include <stdio.h>
#include "udp.h"
simple_udp udp0("127.0.0.1",4001);

int main(int argc, char **argv){
  udp0.udp_send("hello!");
  return 0;
}
