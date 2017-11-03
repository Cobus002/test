#include "nrf24l01.h"


/****************Commands*****************/
#define R_MASK				(uint8_t)0b00000000
#define W_MASK				(uint8_t)0b00100000
#define R_RX_PAYLOAD		(uint8_t)0b01100001
#define W_TX_PAYLOAD		(uint8_t)0b10100000
#define FLUSH_TX			(uint8_t)0b11100001
#define NOP 				(uint8_t)0xff			

/***********Registers********************/
//Config register
#define CONFIG_REG				0x00
/*
 * 7: 	Reserved
 * 6:	MASK_RX_DR
 * 5:	MASK_TX_DS
 * 4:	MASK_MAX_RT
 * 3:	EN_CRC
 * 2:	CRCO
 * 1:	PWR_UP				[1: PWR UP, 0: PWR DOWN]
 * 0:	PRIM_RX				RX/TX Control [1: PRX, 0: PTX]
 */
#define	EN_CRC					(uint8_t)(0x03)
#define	PWR_UP					(uint8_t)(0x01)
#define PRIM_RX					(uint8_t)(0x00)

//Status register
#define STATUS_REG				(uint8_t)(0x07)

/*
 * 7: 	Reserved
 * 6:	RX_DR
 * 5:	TX_DS					Set when data successfully sent
 * 4:	MAX_RT					Maximum retransmit tries, write 1 to clear
 * 3:	RX_P_NO[3]
 * 2:	RX_P_NO[2]
 * 1:	RX_P_NO[1]				[1: PWR UP, 0: PWR DOWN]
 * 0:	TX_FULL					TX_FIFO full
 */
#define RX_DR					(uint8_t)(6)
#define TX_DS					(uint8_t)(5)
#define MAX_RT					(uint8_t)(4)
#define TX_FULL					(uint8_t)(0)

#define EN_AA_REG 				(uint8_t)0x01
#define EN_RXADDR_REG			(uint8_t)0x02
#define RX_PW_P0				(uint8_t)0x11

//Local buffers used for the tx and rx transactions
uint8_t txBuff[TX_RX_BUFF_LEN];
uint8_t rxBuff[TX_RX_BUFF_LEN];


/************************************************************************/
/* Initialise the nrf24l01 module and ensure that the spi is initialised*/
/************************************************************************/

void nrf24l01_setup_tx(void){
	//Clear the buffers
	clear(txBuff);
	clear(rxBuff);
	
	//Make sure that the moduule is not active
	nrf24l01_ce_low();
	
	//Set the write mask and then set number of payload len
	txBuff[0]=(W_MASK|RX_PW_P0);
	txBuff[1]=(NRF24L01_PAYLOAD_LEN);
	nrf24l01_csn_low();
	spi_transmit_receive(txBuff, rxBuff, 2);
	nrf24l01_csn_high();
	
	//clear the buffers
	clear(txBuff);
	clear(rxBuff);
	//Power up the module
	txBuff[0]=(W_MASK)|(CONFIG_REG);
	txBuff[1]=(1<<EN_CRC)|(1<<PWR_UP);
	nrf24l01_csn_low();
	spi_transmit_receive(txBuff, rxBuff, 2);
	nrf24l01_csn_high();
	
}


void nrf24l01_setup_rx(void){
	//Clear the buffers 
	clear(txBuff);
	clear(rxBuff);
	
	//Make sure that the moduule is not active
	nrf24l01_ce_low();
	
	//Set the registers
	txBuff[0]	=		(W_MASK|CONFIG_REG);
	txBuff[1]		=	(1<<EN_CRC)|(1<<PWR_UP)|(1<<PRIM_RX);
	//Send the data
	nrf24l01_csn_low();
	spi_transmit_receive(txBuff, rxBuff, 2);
	nrf24l01_csn_high();
	
	
	//Check that the registers where set correct
	clear(txBuff);
	clear(rxBuff);
	
	//Set the module in active RX mode
	nrf24l01_ce_high();
	
}


void nrf24l01_send_data(uint8_t *txData, uint8_t numBytes){
	//Clear the buffers	
	clear(txBuff);
	clear(rxBuff);
	
	//copy the new content into the txBuff
	txBuff[0]=(W_TX_PAYLOAD);
	memcpy(txBuff+sizeof(uint8_t), txData, numBytes);
	//Send the data
	nrf24l01_csn_low();
	spi_transmit_receive(txBuff, rxBuff, numBytes+1);
	nrf24l01_csn_high();
	
	//Now pulse the CE pin for at least 10 us but no more than 4ms
	nrf24l01_ce_high();
	wait_10us();
	nrf24l01_ce_low();
	
}

void nrf24l01_reset_tx(void){
}

void nrf24l01_reset_rx(void){
}
