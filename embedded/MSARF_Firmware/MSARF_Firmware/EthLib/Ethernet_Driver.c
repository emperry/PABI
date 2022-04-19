/*
 * Ethernet_Driver.c
 *
 *  Created on: Nov 25, 2019
 *      Author: tdubuke
 */

#include "Ethernet_Driver.h"

// 1K should be enough, see https://forum.wiznet.io/t/topic/1612/2
uint8_t buffer[1024];

volatile bool ip_assigned = false;

SPI_HandleTypeDef hspi_eth;

void EthInit(SPI_HandleTypeDef *hspi) {
	hspi_eth = *hspi;

    reg_wizchip_cs_cbfunc(W5500_Select, W5500_Unselect);
    reg_wizchip_spi_cbfunc(W5500_ReadByte, W5500_WriteByte);
    reg_wizchip_spiburst_cbfunc(W5500_ReadBuff, W5500_WriteBuff);

    uint8_t rx_tx_buff_sizes[] = {2, 2, 2, 2, 2, 2, 2, 2};
    wizchip_init(rx_tx_buff_sizes, rx_tx_buff_sizes);

    wiz_NetInfo net_info = {
    	.mac = { 0xea, 0x11, 0x22, 0x33, 0x44, 0x3a },
    	.ip = { 192, 168, 50, 10 },
		.gw = { 192, 168, 50, 1 },
		.sn = { 255, 255, 255, 0 },
		.dns = { 0, 0, 0, 0 },
		.dhcp = NETINFO_STATIC,
    };

    reg_dhcp_cbfunc(
        Callback_IPAssigned,
        Callback_IPAssigned,
        Callback_IPConflict
    );

    wizchip_setnetinfo(&net_info);

    wiz_NetInfo net_info_res;
    wizchip_getnetinfo(&net_info_res);
}

void loop() {
    HAL_Delay(1000);
}

void W5500_Select(void) {
    HAL_GPIO_WritePin(ETH_NSS_GPIO_Port, ETH_NSS_Pin, GPIO_PIN_RESET);
}

void W5500_Unselect(void) {
    HAL_GPIO_WritePin(ETH_NSS_GPIO_Port, ETH_NSS_Pin, GPIO_PIN_SET);
}

void W5500_ReadBuff(uint8_t* buff, uint16_t len) {
    HAL_SPI_Receive(&hspi_eth, buff, len, 10);
}

void W5500_WriteBuff(uint8_t* buff, uint16_t len) {
    HAL_SPI_Transmit(&hspi_eth, buff, len, 10);
}

uint8_t W5500_ReadByte(void) {
    uint8_t byte;
    W5500_ReadBuff(&byte, sizeof(byte));
    return byte;
}

void W5500_WriteByte(uint8_t byte) {
    W5500_WriteBuff(&byte, sizeof(byte));
}

void Callback_IPAssigned(void) {
    ip_assigned = true;
}

void Callback_IPConflict(void) {
}

void loop_tcps(uint16_t port){
	loopback_tcps(TCP_SOCKET, buffer, port);
}

void loop_udps(uint16_t port){
	loopback_udps(UDP_SOCKET, buffer, port);
}
