/*
 * RelayController.c
 *
 * Created: 10/27/2017 6:03:05 PM
 * Author : Cobus Hoffmann
 */ 

#define F_CPU 16000000UL

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


//Defines for USART communication
#define BAUD				9600
#define MY_UBRR				(unsigned int)(F_CPU/16/BAUD -1)


//Define some macros
#define setbit(port, bit)		(port) |= (1<< (bit))
#define clearbit(port, bit)		(port) &= ~(1<<(bit))

typedef unsigned char uint8_t;

uint8_t rx_buff[10], tx_buff[10];


typedef struct {
	uint8_t buff[20];
	uint8_t valid;
	uint8_t count;
}USART_RX_STRUCT;

USART_RX_STRUCT usart_rx_data;





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

void USART_Init(unsigned int ubrr);
void USART_Transmit(unsigned char data);
void USART_println(char *str);

void USART_println_bin(uint8_t c);

void ISR1_init();

char str[20];

uint8_t data[10];



int main(void)
{
	//Initialise the spi
	spi_init();
	
	//Set the NRF_CE_PIN as output
	DDRB|=(1<<NRF24L01_CE_PIN);
	
	clearbit(NRF24L01_CE_PORT, NRF24L01_CE_PIN);
	
	//Set the ss high
	setbit(PORTB, DD_SS);
	
	memset(tx_buff, 0, sizeof(tx_buff));
	memset(rx_buff, 0, sizeof(rx_buff));
	//Wait a little bit for the nrf module to start up successfully
	_delay_ms(100);
	
	
	//Set up the USART 
	
	clear(str);
	
	USART_Init(MY_UBRR);
	
	_delay_ms(2000);
	
	//enable global interrupts
	ISR1_init();
		
	sei();
	
	
	
    while (1) 
    {
		_delay_ms(100);
		
		if(usart_rx_data.valid == 1){
			cli();
			
			if(!strncmp(usart_rx_data.buff, "Regs", strlen("Regs"))){
				//Print all the register values
				for(uint8_t i=0; i< 10; i++){
					nrf24l01_read_reg(i, data, 1);
					
					USART_Transmit(i+48);
					
					USART_Transmit(' ');
					USART_Transmit(':');
					USART_Transmit('\t');
					USART_println_bin(data);
					USART_Transmit('\n');
					
				}
				
				for(int i=0; i<7; i++ ){
					nrf24l01_read_reg(i+10, &data, 6);
					USART_Transmit('1');
					USART_Transmit(i+48);
					USART_Transmit(' ');
					USART_Transmit(':');
					USART_Transmit('\t');
					USART_println_bin(data[0]);
					USART_Transmit('\t');
					USART_println_bin(data[1]);
					USART_Transmit('\t');
					USART_println_bin(data[2]);
					USART_Transmit('\t');
					USART_println_bin(data[3]);
					USART_Transmit('\t');
					USART_println_bin(data[4]);
					USART_Transmit('\n');
					
					
				}
				
			}else if(!strncmp(usart_rx_data.buff, "Setup TX", strlen("Setup TX"))){
				//Set the module into tx mode
				nrf24l01_setup_tx();
				USART_println("TX - setup complete\n");
				
			}else if(!strncmp(usart_rx_data.buff, "Setup RX", strlen("Setup RX"))){
				//Set up the module in rx mode
				nrf24l01_setup_rx();
				USART_println("RX - setup complete\n");
			}
			
			usart_rx_data.valid=0;
			clear(usart_rx_data.buff);
			sei();
		}
		
		
		

		
		
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
	DDRB |= (1<<DD_MOSI)|(1<<DD_SCK)|(1<<DD_SS)|(1<<DD_SS2);
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
		spi_transmit(txBuff[i]);
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
	PORTB &= ~(1<<DD_SS);
	
}

/**
 * @brief	This function is used to pull the spi chip selects pin high of the nrf24l01
 *			module which is needed for spi communication.
 * @param	None
 *			
 * @retval	None
 */
void nrf24l01_csn_high(){
	PORTB |= (1<<DD_SS);
	
	
}

/**
 * @brief	This function is used to generate a delay greater than 10us needed by the nrf module
 *			to allow for a successful transmission.
 * @param	None
 *			
 * @retval	None
 */
void wait_10us(){
	_delay_us(10);
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
	UCSR0B = (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0);
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
 * @brief	This function is used to receive data over usart
 * @param	None
 *			
 * @retval	None
 */
unsigned char USART_Receive( void )
{
	/* Wait for data to be received */
	while ( !(UCSR0A & (1<<RXC0)) )
	;
	/* Get and return received data from buffer */
	return UDR0;
}

void USART_println(char *str){
	
	for(int i=0; i< strlen(str); i++){
		USART_Transmit(str[i]);
	}
	
	
}

void USART_println_bin(uint8_t c){
	for(int i=0; i<8; i++){
		if(c>>(7-i) & 0x01){
			USART_Transmit('1');
			
			}else{
			USART_Transmit('0');
		}
	}
	

}



void ISR1_init(){
	
	//Set the external interrupt to trigger on falling edge
	EICRA |= (1<<ISC11);
	
	//Enable the int 1
	EIMSK |= (1<<INT1);
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
	USART_Transmit('I');
	USART_Transmit('\n');
	//Re-enable the interrupts
	sei();
}




/**
 * @brief	This function is used to handle the interrupt service routine to
 *			handle the RX complete interrupt.
 * @param	None
 *			
 * @retval	None
 */
ISR(USART_RX_vect){
	cli();
	uint8_t c = UDR0;
	if(c=='\n'){
		//Done 
		usart_rx_data.valid=1;
		//Set the count to zero
		usart_rx_data.count=0;
	}else{
		//Not done

		//Get the new USART char and add to the buffer
		usart_rx_data.buff[usart_rx_data.count]=c;
		//Increment the count
		usart_rx_data.count++;
	}
	sei();
	
	
}

