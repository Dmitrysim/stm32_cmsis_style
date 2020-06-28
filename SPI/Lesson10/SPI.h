#ifndef __SPI_H
#define __SPI_H

#include "global.h"

void SPI1_init(void);
void SPI1_TX_RX(char * SPI1_TX_data, char * SPI1_RX_data, uint8_t SPI1_TX_RX_data_size);
	
#ifdef glob
 char SPI1_TX_data[20];
 char SPI1_RX_data[20];
 uint8_t SPI1_TX_RX_data_size;
#else
 extern char SPI1_TX_data[20];
 extern char SPI1_RX_data[20];
 extern uint8_t SPI1_TX_RX_data_size;
#endif

#endif
