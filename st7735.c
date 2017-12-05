#include "st7735.h"

#define ST7735_NOP				(uint8_t)0x00		//No operation
#define ST7735_SWRESET			(uint8_t)0x01		//Software reset
#define ST7735_RDDID			(uint8_t)0x04		//Read Display ID
#define ST7735_RDDST			(uint8_t)0x09		//Read Display Status
#define ST7735_RDDPM			(uint8_t)0x0A		//Read Display Power
#define ST7735_RDD_MADCTL		(uint8_t)0x0B		//Read Display
#define ST7735_RDD_COLMOD		(uint8_t)0x0C		//Read Display Pixel
#define ST7735_RDDIM			(uint8_t)0x0D
#define ST7735_RDDSM			(uint8_t)0x0E
#define ST7735_SLPIN			(uint8_t)0x10
#define ST7735_SLPOUT			(uint8_t)0x11
#define ST7735_PTLON			(uint8_t)0x12
#define ST7735_NORON			(uint8_t)0x13
#define ST7735_INVOFF			(uint8_t)0x20
#define ST7735_INVON			(uint8_t)0x21
#define ST7735_GAMSET			(uint8_t)0x26
#define ST7735_DISPOFF			(uint8_t)0x28
#define ST7735_DISPON			(uint8_t)0x29
#define ST7735_CASET			(uint8_t)0x2A
#define ST7735_RASET			(uint8_t)0x2B
#define ST7735_RAMWR			(uint8_t)0x2C
#define ST7735_RAMRD			(uint8_t)0x2E
#define ST7735_PTLAR			(uint8_t)0x30
#define ST7735_TEOFF			(uint8_t)0x34
#define ST7735_TEON				(uint8_t)0x35
#define ST7735_MADCTL			(uint8_t)0x36
#define ST7735_IDMOFF			(uint8_t)0x38
#define ST7735_IDMON			(uint8_t)0x39
#define ST7735_COLMOD			(uint8_t)0x3A		//Interface pixel format
#define ST7735_RDID1			(uint8_t)0xDA
#define ST7735_RDID2			(uint8_t)0xDB
#define ST7735_RDID3			(uint8_t)0xDC


void st7735_init(void){
	//Reset the screen
	st7735_cs_high();
	st7735_rst_high();
	delay_ms(5);
	st7735_rst_low();
	delay_ms(5);
	st7735_rst_high();
	delay_ms(5);
	
	//First send NOP
	st7735_write_reg(ST7735_NOP);
	
	//Sleep out 
	st7735_write_reg(ST7735_SLPOUT);
	//Wait for the display to wake
	delay_ms(10);
	
	//Setup 12 bit colour format
	st7735_write_reg(ST7735_COLMOD);
	st7735_write_data(0x05);
	
	//Display Inversion off
	st7735_write_reg(ST7735_INVOFF);
	//Normal Display on
	st7735_write_reg(ST7735_NORON);
	
	//Turn the display on
	st7735_write_reg(ST7735_DISPON);
	delay_ms(10);
	
	st7735_write_reg(ST7735_CASET);

	//Now set the area
	st7735_write_data(0x00);
	st7735_write_data(0x00);
	st7735_write_data(0x00);
	st7735_write_data((uint8_t) 128);

	//set the rows
	st7735_write_reg(ST7735_RASET);

	//Now set the area
	st7735_write_data(0x00);
	st7735_write_data(0x00);
	st7735_write_data(0x00);
	st7735_write_data((uint8_t) 131);
	
	st7735_write_data(ST7735_RAMWR);
	
	
	//Dummy print
	for(uint8_t r = 0; r< 129; r++){
		for(uint8_t c=0; c<132; c++){
			st7735_write_data(0x00);
			st7735_write_data(0x0f);
		}
		delay_ms(1);
	}
	
	st7735_write_reg(ST7735_NOP);

}

void st7735_write_reg(uint8_t reg){
	//Pull the data/cmd pin high
	
	//Pull the chip select pin low
	
	//Send the the data
	
	//Pull the chip select high
}


void st7735_write_data(uint8_t data){
	//Pull the data/cmd pin low
	
	//pull the chip select pin low
	
	//Send the data
	
	//Pull the chip select high
}

void st7735_write_multi_data(uint8_t *data, uint16_t numBytes){
	
}

void st7735_disp_on(){
	
}

void st7735_disp_off(){
	
}

void st7735_drawpixel(uint16_t x, uint16_t y, uint16_t colour){
	
}

void st7735_drawpixel_array(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd, uint16_t *data){
	
}

void st7735_set_window(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd){
	
}

void st7735_clear(uint16_t colour){
	
}

void st7735_fill(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint16_t colour){
	
}