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
#include <avr/eeprom.h>
#include "nrf24l01.h"

//Used to distinguish if the message was for us
uint8_t EEMEM PairCode;
uint8_t EEMEM Mode;

//Will initially be undefined
#define MODE_TX			(uint8_t)1
#define MODE_RX			(uint8_t)2


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


#define OUTPUT				(uint8_t) 1
#define INPUT				(uint8_t) 0

#define OUTPUT_PORT			PORTC
#define OUTPUT_DDR			DDRC

//Define some macros to clear and set bits on a port
#define setbit(port, bit)		(port) |= (1<< (bit))
#define clearbit(port, bit)		(port) &= ~(1<<(bit))
//Define unsigned char as uint8_t
typedef unsigned char uint8_t;

uint8_t rx_buff[10], tx_buff[10];

//Struct to keep all USART_RX functionality together
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

//Set the port c for input or output depending on the mode that the module is in
void setup_portc_gpio(uint8_t mode);

uint8_t read_portc_gpio();
void set_portc_gpio(uint8_t value);

void ISR1_init();

char str[20];

uint8_t data[10];

volatile uint8_t IRQ=0; 


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
	USART_Transmit('.');
	
	//enable global interrupts
	ISR1_init();
		
	sei();
	
	//Check what mode I am in
	uint8_t my_mode = eeprom_read_byte(&Mode);
	
	if (my_mode == MODE_TX)
	{
		//Set up for TX
		nrf24l01_setup_tx();
		setup_portc_gpio(INPUT);
		
	} 
	else if(my_mode== MODE_RX)
	{
		//Set up for RX
		nrf24l01_setup_rx();
		setup_portc_gpio(OUTPUT);
		
	}
	
	
	
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
					USART_println_bin(data[0]);
					USART_Transmit('\n');
					
				}
				
				for(int i=0; i<7; i++ ){
					nrf24l01_read_reg(i+10, data, 6);
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
				setup_portc_gpio(INPUT);
				eeprom_write_byte(&Mode, MODE_TX);
				my_mode=MODE_TX;
				
			}else if(!strncmp(usart_rx_data.buff, "Setup RX", strlen("Setup RX"))){
				//Set up the module in rx mode
				nrf24l01_setup_rx();
				USART_println("RX - setup complete\n");
				setup_portc_gpio(OUTPUT);
				eeprom_write_byte(&Mode, MODE_RX);
				my_mode=MODE_RX;
				
			}else if(!strncmp(usart_rx_data.buff, "TX1", strlen("TX1"))){
				//Transmit dummy data
				tx_buff[0] = 0xaa;
				tx_buff[1] = read_portc_gpio();
				nrf24l01_send_data(tx_buff,2);
				USART_println("Transmit - ");
				USART_println_bin(tx_buff[1]);
				USART_Transmit('\n');
			}else if(!strncmp(usart_rx_data.buff, "TX2", strlen("TX2"))){
				//Transmit dummy data
				uint8_t dummy = 0b10010110;
				nrf24l01_send_data(&dummy,1);
				USART_println("Transmit - complete\n");
			}else if(!strncmp(usart_rx_data.buff, "Read RX", strlen("Read RX"))){
				clear(data);
				nrf24l01_read_rx(data, 1);
				USART_println("Data Received: ");
				USART_println_bin(data[0]);
				USART_Transmit('\n');
				set_portc_gpio(data[0]);
				
			}else if(!strncmp(usart_rx_data.buff, "Reset TX", strlen("Reset TX"))){
				nrf24l01_reset_tx();
				USART_println("TX Reset\n");
			}else if(!strncmp(usart_rx_data.buff, "Reset RX", strlen("Reset RX"))){
				nrf24l01_reset_rx();
				USART_println("RX Reset\n");
			}else if(!strncmp(usart_rx_data.buff, "Set Addr:", strlen("Set Addr:"))){
				uint8_t pairCode = usart_rx_data.buff[strlen("Set Addr:")];
				eeprom_write_byte(&PairCode, pairCode);
				USART_println("PairCode is:");
				USART_Transmit(eeprom_read_byte(&PairCode));
				USART_Transmit('\n');
			}
			
			usart_rx_data.valid=0;
			clear(usart_rx_data.buff);
			sei();
		}
		
		
		if (my_mode==MODE_TX)
		{
			
			if(IRQ){
				//Check if sent successfully
				nrf24l01_read_reg(0x07, rx_buff, 1);
				if(0x01&(rx_buff[0]>>5)){
					//Data sent successfully
					USART_println("Data sent successfully \n");
				}else if(0x01&(rx_buff[0]>>4)){
					//Max retries occurred
					USART_println("Max Retries occurred \n");
				}
				
				nrf24l01_reset_tx();
				IRQ=0;
				
			}else{
				//Read the switches
				uint8_t switches = read_portc_gpio();
				tx_buff[0]=eeprom_read_byte(&PairCode);
				tx_buff[1]=switches;
				nrf24l01_send_data(tx_buff, 2);
			}
			
		}else if(my_mode==MODE_RX){
			if(IRQ){
				//Something happend, check the status reg
				nrf24l01_read_reg(0x07, rx_buff, 1);
				if(0x01&(rx_buff[0]>>6)){
					//there be data
					USART_println("There be data\n");
					
					//Now read the actual data send from the tx
					nrf24l01_read_rx(rx_buff,2);
					
					if (rx_buff[0]==eeprom_read_byte(&PairCode))
					{
						//Paired device sent data
						set_portc_gpio(rx_buff[1]);
						USART_println("Update Switches\n");
					}else{
						USART_println("Unknown Sender ");
						USART_Transmit(rx_buff[0]);
						USART_Transmit('\n');
					}
					
					
					
				}else{
					//I have no idea what happened
					USART_println("What the !!??\n");
					
				}
				
				nrf24l01_reset_rx();
				IRQ=0;
			}
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
			/* Start transmission */
			SPDR = txBuff[i];
			/* Wait for transmission complete */
			while(!(SPSR & (1<<SPIF)))
			;
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


void setup_portc_gpio(uint8_t mode){
	switch (mode)
	{
		case INPUT:
			//clear the bits for input
			OUTPUT_DDR &= ~((1<<0)|(1<<1)|(1<<2)|(1<<3));
			
		break;
		case OUTPUT:
			//Set the bits for output
			OUTPUT_DDR |= (1<<0)|(1<<1)|(1<<2)|(1<<3);
			//Set the pins low
			PINC &= ~((1<<0)|(1<<1)|(1<<2)|(1<<3));
		break;
	}
}

uint8_t read_portc_gpio(){
	uint8_t val = 0;
	//Read the lower 4 pins
	val = PINC & 0b00001111;
	return val;
}

void set_portc_gpio(uint8_t value){
	//Set the lower 4 bits of portc, ensure no data is entered for upper 4
	value = value&0x0f;	//Clear the top bits just in case
	PORTC = ((PORTC&0xf0)|(value));
	
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
	IRQ=1;
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

