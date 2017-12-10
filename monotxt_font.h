
#ifndef MONOTXT_FONT_H
#define MONOTXT_FONT_H

// Font data for Monotxt_IV25 12pt

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef uint16_t FONT_CHAR_INFO[26][2];

typedef struct{
	uint8_t height;
	uint8_t startChar;
	uint8_t endChar;
	uint8_t spaceWidth;
	uint8_t **fontCharInfo;
	const uint8_t *bitmapArray;

}FONT_INFO;



extern const uint8_t monotxt_IV25_12ptBitmaps[];

extern const FONT_CHAR_INFO monotxt_IV25_12ptDescriptors;

extern FONT_INFO monotxt_font_info;



#endif
