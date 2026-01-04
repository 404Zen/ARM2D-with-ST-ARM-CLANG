#ifndef __LCD_H__
#define __LCD_H__


/* LCD Transmit Port*/
#include <stdint.h>

extern volatile uint8_t g_full_scr_done;

#define LCD_WIDTH           240
#define LCD_HEIGTH          240
#define LCD_FULL_SCR_SIZE   2*LCD_WIDTH*LCD_HEIGTH


void LCD_WriteData_Port(uint8_t *data, uint16_t len);

/* LCD Function */
void LCD_Init(void);
void LCD_Set_Address(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void LCD_FillScreen(uint16_t color);

/* ARM2D Port */
// void Disp0_DrawBitmap(uint32_t x, uint32_t y, uint32_t width, uint32_t height, const uint8_t *bitmap);

#endif
