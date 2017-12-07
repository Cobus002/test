#ifndef ST7735_H
#define ST7735_H

/************************************************************************/
/* This driver makes use of the 4-line serial interface on the st7735 TFT
   driver IC.                                                            */
/************************************************************************/

#include <stdio.h>

/******************Defines**************************/
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;

#define ST7735_X_LEN			(uint8_t) 132
#define	ST7735_Y_LEN			(uint8_t) 162


/*****************External functions ***************/
extern void spi_transmit(char cData);
extern void spi_transmit_receive(uint8_t *txBuff, uint8_t *rxBuff, uint8_t numBytes);
extern void st7735_cs_low();			//Chip select for SPI protocol
extern void st7735_cs_high();
extern void st7735_rst_low();			//Used to reset the st7735 driver IC
extern void st7735_rst_high();
extern void st7735_D_CX_low();			//Used to distinguish between data or command
extern void st7735_D_CX_high();

extern void delay_ms(uint8_t delay_time);
/*******************Includes************************/
/*******************Functions***********************/
void st7735_init(void);

void st7735_write_reg(uint8_t reg);

void st7735_write_data(uint8_t data);

void st7735_write_multi_data(uint8_t *data, uint16_t numBytes);

void st7735_disp_on();

void st7735_disp_off();

void st7735_drawpixel(uint16_t x, uint16_t y, uint16_t colour);

void st7735_drawpixel_array(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd, uint16_t *data);

void st7735_set_window(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd);

void st7735_clear(uint16_t colour);

void st7735_fill(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint16_t colour);

//void st7735_write_char(uint8_t x, uint8_t y, uint8_t c, sFONT *font);


#endif
