/*
 * Ethernet_Driver.h
 *
 *  Created on: Nov 25, 2019
 *      Author: tdubuke
 */

#ifndef ETHERNET_DRIVER_H_
#define ETHERNET_DRIVER_H_

#include "main.h"
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>
#include <ctype.h>
#include "socket.h"
#include "dhcp.h"
#include "dns.h"
#include "loopback.h"

#define TCP_SOCKET  0
#define UDP_SOCKET  1

void EthInit(SPI_HandleTypeDef *hspi);
void loop();
void W5500_Select(void);
void W5500_Unselect(void);
void W5500_ReadBuff(uint8_t* buff, uint16_t len);
void W5500_WriteBuff(uint8_t* buff, uint16_t len);
uint8_t W5500_ReadByte(void);
void W5500_WriteByte(uint8_t byte);
void Callback_IPAssigned(void);
void Callback_IPConflict(void);
void loop_tcps(uint16_t port);
void loop_udps(uint16_t port);

#endif /* ETHERNET_DRIVER_H_ */
