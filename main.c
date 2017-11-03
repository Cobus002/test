/*
 * RelayController.c
 *
 * Created: 10/27/2017 6:03:05 PM
 * Author : Cobus Hoffmann
 */ 

#define F_CPU 16000000UL

#define TX_MOD	1

#include <avr/io.h>
#include <string.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "nrf24l01.h"

//Define the pins needed for the spi interface
#define DDR_SPI DDRB
#define DD_MOSI 3
#define DD_MISO	4
#define DD_SCK	5
#define DD_SS	2
#define DD_SS2  1


#define NRF24L01_CE_PORT	PORTB
#define NRF24L01_CE_PIN		0

#define SER_ENB	0


//Defines for USART communication
#define BAUD				9600
#define MY_UBRR				(unsigned int)(F_CPU/16/BAUD -1)


//Define some macros
#define setbit(port, bit)		(port) |= (1<< (bit))
#define clearbit(port, bit)		(port) &= ~(1<<(bit))

typedef unsigned char uint8_t;

uint8_t rx_buff[10], tx_buff[10];



//Initialise the spi
void spi_init(void);
//Transmit a single char
void spi_transmit(char cData);
//Transmit and receive a number of bytes
void spi_transmit_receive(uint8_t *txBuff, uint8_t *rxBuff, uint8_t numBytes);

//Enable and disable the nrf24l01 module
void nrf24l01_ce_low();			//Activate RX TX modes functions required by the nrf24l01 lib
void nrf24l01_ce_high();

void nrf24l01_csn_low();		//SPI Chip Select functions required by nrf24l01 lib
void nrf24l01_csn_high();

void wait_10us();				//Wait function required by the nrf24l01 lib




int main(void)
{
	//Initialise the spi
	spi_init();
	//Set the ss low
	setbit(PORTB, DD_SS);
	
	DDRB |= (1<< SER_ENB);
	setbit(PORTB, SER_ENB);
	
	memset(tx_buff, 0, sizeof(tx_buff));
	memset(rx_buff, 0, sizeof(rx_buff));
	//Wait a little bit for the nrf module to start up successfully
	_delay_ms(100);
	
	//set up the interrupt on INT1
	
	
	//eable global interupts
	//sei();
	
	
	//Set up the USART 
	uint8_t str[]={'R', 'e', 'C', '\n', '0'};
	USART_Init(MY_UBRR);
	_delay_ms(1);
	
	
	
	
	#ifdef TX_MOD
		nrf24l01_setup_tx();
	#else
		nrf24l01_setup_rx();
	#endif
	
	_delay_ms(100);
	
	
	
    while (1) 
    {
		USART_Transmit('H');
		_delay_ms(100);
		
		
    }
}

/**
 * @brief  This function initialises the spi
 * @param	None	
 * @retval	None
 */
void spi_init(void)
{
	/* Set MOSI and SCK output, all others input */
	DDR_SPI |= (1<<DD_MOSI)|(1<<DD_SCK)|(1<<DD_SS)|(1<<DD_SS2);
	/* Enable SPI, Master, set clock rate fck/16 */
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
}

/**
 * @brief  This function is used to send data over spi
 * @param	char cData: the byte to be sent		
 * @retval None
 */
void spi_transmit(char cData)
{
	/* Start transmission */
	SPDR = cData;
	/* Wait for transmission complete */
	while(!(SPSR & (1<<SPIF)))
	;
}


/**
 * @brief  This function is used to send and receive data over spi
 * @param	uint8_t *txBuff:	Buffer for transmission
			uint8_t *rxBuffer:	Buffer for the received data
			uint8_t numBytes:	number of bytes to transmit and receive
			
 * @retval None
 */
void spi_transmit_receive(uint8_t *txBuff, uint8_t *rxBuff, uint8_t numBytes){
	
	//Clear the receive buffer
	memset(rxBuff, '\0', sizeof(rxBuff));
	
	for(uint8_t i=0; i<numBytes; i++){
		/* Start transmission */
		SPDR = txBuff[i];
		/* Wait for transmission complete */
		while(!(SPSR & (1<<SPIF)))
		;
		
		//Add to the buffer array
		rxBuff[i]=SPDR;
	}
}


/**
 * @brief	This function is used to pull the chip enable pin low of the nrf24l01 module
 *			which allows the module to enter the active mode.
 * @param	None
 *			
 * @retval	None
 */
void nrf24l01_ce_low(){
	
	clearbit(NRF24L01_CE_PORT, NRF24L01_CE_PIN);
	
}


/**
 * @brief	This function is used to pull the chip enable pin high of the nrf24l01 module
 *			which allows the module to enter the active mode.
 * @param	None
 *			
 * @retval	None
 */
void nrf24l01_ce_high(){
	
	setbit(NRF24L01_CE_PORT, NRF24L01_CE_PIN);
	
	
}



/**
 * @brief	This function is used to pull the spi chip selects pin low of the nrf24l01
 *			module which is needed for spi communication.
 * @param	None
 *			
 * @retval	None
 */
void nrf24l01_csn_low(){
	clearbit(PORTB, DD_SS);
	
}

/**
 * @brief	This function is used to pull the spi chip selects pin high of the nrf24l01
 *			module which is needed for spi communication.
 * @param	None
 *			
 * @retval	None
 */
void nrf24l01_csn_high(){
	
	setbit(PORTB, DD_SS);
	
}

/**
 * @brief	This function is used to generate a delay greater than 10us needed by the nrf module
 *			to allow for a successful transmission.
 * @param	None
 *			
 * @retval	None
 */
void wait_10us(){
	_delay_us(20);
}

/**
 * @brief	This function is used to initialise the usart for serial communication.
 * @param	unsigned int ubrr: used to set up the correct baud rate
							   ubrr = F_CPU/16/BAUD-1
 *			
 * @retval	None
 */
void USART_Init( unsigned int ubrr)
{
	/*Set baud rate */
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	/*Enable receiver and transmitter */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	/* Set frame format: 8data, 1stop bit */
	UCSR0C = (3<<UCSZ00);
}

/**
 * @brief	This function is used to send data over usart
 * @param	unsigned char data: charachter to be sent 
 *			
 * @retval	None
 */
void USART_Transmit( unsigned char data )
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) )
	;
	/* Put data into buffer, sends the data */
	UDR0 = data;
}


/**
 * @brief	This function is a interupt service routine used to check when there is a trigger
 *			on the nrf24l01  IRQ pin, which is acctive low.
 * @param	None
 *			
 * @retval	None
 */
ISR(INT1_vect){
	//Clear the interrupt
	cli();
	
	
	
	
	
	//Re-enable the interrupts
	sei();
	
	
	
}

