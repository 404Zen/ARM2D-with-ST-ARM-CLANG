/*****************************************************************************
* | File      	:   LCD_1in54.c
* | Author      :   Waveshare team
* | Function    :   Hardware underlying interface
* | Info        :
*                Used to shield the underlying layers of each master 
*                and enhance portability
*----------------
* |	This version:   V1.0
* | Date        :   2020-05-20
* | Info        :   Basic version
*
******************************************************************************/
#ifndef __LCD_1IN3_H
#define __LCD_1IN3_H

// #include "../Config/DEV_Config.h"
#include <stdint.h>

#include <stdlib.h>     //itoa()
#include <stdio.h>
#include <stdbool.h>

#include "main.h"       // STM32 HAL Library
#include "spi.h"

#include "arm_2d.h"
#include "arm_2d_helper_shape.h"

#define LCD_1IN3_HEIGHT 240
#define LCD_1IN3_WIDTH 240


#define HORIZONTAL 0
#define VERTICAL   1

#define LCD_1IN3_SetBacklight(Value) ; 


typedef struct{
    uint16_t WIDTH;
    uint16_t HEIGHT;
    uint8_t  SCAN_DIR;
}LCD_1IN3_ATTRIBUTES;
extern LCD_1IN3_ATTRIBUTES LCD_1IN3;

#define EPD_DC_PIN      1
#define EPD_RST_PIN     2
#define EPD_CS_PIN      3
#define EPD_PWR_PIN     4

/* Redefine some APIs */
#define DEV_Digital_Write(x, y)                 switch(x)   \
{                                                           \
    case EPD_CS_PIN:    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, (y==0) ? (GPIO_PIN_RESET):(GPIO_PIN_SET)); break;   \
    case EPD_DC_PIN:    HAL_GPIO_WritePin(LCD_WR_GPIO_Port, LCD_WR_Pin, (y==0) ? (GPIO_PIN_RESET):(GPIO_PIN_SET)); break;   \
    case EPD_RST_PIN:   HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, (y==0) ? (GPIO_PIN_RESET):(GPIO_PIN_SET)); break; \
    case EPD_PWR_PIN:   HAL_GPIO_WritePin(LCD_PWR_GPIO_Port, LCD_PWR_Pin, (y==0) ? (GPIO_PIN_RESET):(GPIO_PIN_SET)); break; \
    default : break;                                                                                                        \
}

#define DEV_Delay_ms(x)                         HAL_Delay(x)

// #define DEV_SPI_WriteByte(x)                  SPI1_WriteData(&(x), 1)

/********************************************************************************
function:	
			Macro definition variable name
********************************************************************************/
void LCD_1IN3_Init(uint8_t Scan_dir);
void LCD_1IN3_Clear(uint16_t Color);
void LCD_1IN3_Display(uint16_t *Image);
void LCD_1IN3_DisplayWindows(uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend, uint16_t *Image);
void LCD_1IN3_DisplayPoint(uint16_t X, uint16_t Y, uint16_t Color);



void Disp0_DrawBitmap(int16_t x, int16_t y, int16_t width, int16_t height, const uint8_t *bitmap);
// void __disp_adapter0_request_async_flushing(void *pTarget, bool bIsNewFrame, int16_t iX, int16_t iY, int16_t iWidth, int16_t iHeight, const COLOUR_INT *pBuffer);
// void Disp0_DrawBitmap_Test(void);       
#endif
