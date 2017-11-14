#ifndef NRF24L01_H
#define NRF24L01_H

#include <string.h>

typedef unsigned char uint8_t;

#define clear(x)	memset((x), 0, sizeof((x)))

/******************Defines**************************/
#define TX_RX_BUFF_LEN				20
#define NRF24L01_PAYLOAD_LEN		1

/*****************External functions ***************/
extern void nrf24l01_csn_low();		//SPI Chip Select
extern void nrf24l01_csn_high();
extern void nrf24l01_ce_low();		//Chip enable activates RX or TX mode
extern void nrf24l01_ce_high();		
extern void wait_10us();	

extern void spi_init();
extern void spi_transmit(char cData);
extern void spi_transmit_receive(uint8_t *txBuff, uint8_t *rxBuff, uint8_t numBytes);

/*******************Includes************************/
/*******************Functions***********************/
void nrf24l01_init(void);
void nrf24l01_setup_tx(void);
void nrf24l01_setup_rx(void);

void nrf24l01_send_data(uint8_t *txData, uint8_t numBytes);

uint8_t nrf24l01_get_status();

uint8_t nrf24l01_get_config();

uint8_t nrf24l01_write_reg(uint8_t reg, uint8_t *data, uint8_t numBytes);

uint8_t nrf24l01_read_reg(uint8_t reg, uint8_t *buff, uint8_t numBytes);

uint8_t nrf24l01_read_rx(uint8_t *buff, uint8_t numBytes);


void nrf24l01_reset_tx(void);
void nrf24l01_reset_rx(void);


#endif
