/*****************************************************************************
* | File      	:   LCD_1in14.c
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
#include "LCD_1in3.h"
// #include "../Config/DEV_Config.h"

#include <stdint.h>
#include <stdlib.h>		//itoa()
#include <stdio.h>

#include "arm_2d_disp_adapter_0.h"
#include "perf_counter.h"
#include "stm32g4xx_hal.h"

LCD_1IN3_ATTRIBUTES LCD_1IN3;
ARM2D_ASYNC_FLUSHING_DATA adapter0_async_flushing_data;

static void DEV_SPI_WriteByte(uint8_t data)
{
    HAL_SPI_Transmit(&hspi1, (uint8_t *)&data, 1, HAL_MAX_DELAY);
}

static void DEV_SPI_Write_nByte(uint8_t *data_ptr, uint16_t len)
{
    HAL_SPI_Transmit(&hspi1, data_ptr, len, HAL_MAX_DELAY);
    // HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *)data_ptr, (uint16_t)len);
}



/******************************************************************************
function :	Hardware reset
parameter:
******************************************************************************/
static void LCD_1IN3_Reset(void)
{
    DEV_Digital_Write(EPD_RST_PIN, 1);
    DEV_Delay_ms(100);
    DEV_Digital_Write(EPD_RST_PIN, 0);
    DEV_Delay_ms(100);
    DEV_Digital_Write(EPD_RST_PIN, 1);
    DEV_Delay_ms(100);
}

/******************************************************************************
function :	send command
parameter:
     Reg : Command register
******************************************************************************/
static void LCD_1IN3_SendCommand(uint8_t Reg)
{
    DEV_Digital_Write(EPD_DC_PIN, 0);
    DEV_Digital_Write(EPD_CS_PIN, 0);
    DEV_SPI_WriteByte(Reg);
    DEV_Digital_Write(EPD_CS_PIN, 1);
}

/******************************************************************************
function :	send data
parameter:
    Data : Write data
******************************************************************************/
static void LCD_1IN3_SendData_8Bit(uint8_t Data)
{
    DEV_Digital_Write(EPD_DC_PIN, 1);
    DEV_Digital_Write(EPD_CS_PIN, 0);
    DEV_SPI_WriteByte(Data);
    DEV_Digital_Write(EPD_CS_PIN, 1);
}

/******************************************************************************
function :	send data
parameter:
    Data : Write data
******************************************************************************/
static void LCD_1IN3_SendData_16Bit(uint16_t Data)
{
    DEV_Digital_Write(EPD_DC_PIN, 1);
    DEV_Digital_Write(EPD_CS_PIN, 0);
    DEV_SPI_WriteByte(Data & 0xFF);
    DEV_SPI_WriteByte((Data >> 8) & 0xFF);
    DEV_Digital_Write(EPD_CS_PIN, 1);
}

/******************************************************************************
function :	Initialize the lcd register
parameter:
******************************************************************************/
static void LCD_1IN3_InitReg(void)
{
    LCD_1IN3_SendCommand(0x3A);
    LCD_1IN3_SendData_8Bit(0x05);

    LCD_1IN3_SendCommand(0xB2);
    LCD_1IN3_SendData_8Bit(0x0C);
    LCD_1IN3_SendData_8Bit(0x0C);
    LCD_1IN3_SendData_8Bit(0x00);
    LCD_1IN3_SendData_8Bit(0x33);
    LCD_1IN3_SendData_8Bit(0x33);

    LCD_1IN3_SendCommand(0xB7);  //Gate Control
    LCD_1IN3_SendData_8Bit(0x35);

    LCD_1IN3_SendCommand(0xBB);  //VCOM Setting
    LCD_1IN3_SendData_8Bit(0x19);

    LCD_1IN3_SendCommand(0xC0); //LCM Control     
    LCD_1IN3_SendData_8Bit(0x2C);

    LCD_1IN3_SendCommand(0xC2);  //VDV and VRH Command Enable
    LCD_1IN3_SendData_8Bit(0x01);
    LCD_1IN3_SendCommand(0xC3);  //VRH Set
    LCD_1IN3_SendData_8Bit(0x12);
    LCD_1IN3_SendCommand(0xC4);  //VDV Set
    LCD_1IN3_SendData_8Bit(0x20);

    LCD_1IN3_SendCommand(0xC6);  //Frame Rate Control in Normal Mode
    LCD_1IN3_SendData_8Bit(0x0F);
    
    LCD_1IN3_SendCommand(0xB0);
    LCD_1IN3_SendData_8Bit(0x00);
    LCD_1IN3_SendData_8Bit(0xC8);
    
    LCD_1IN3_SendCommand(0xD0);  // Power Control 1
    LCD_1IN3_SendData_8Bit(0xA4);
    LCD_1IN3_SendData_8Bit(0xA1);

    LCD_1IN3_SendCommand(0xE0);  //Positive Voltage Gamma Control
    LCD_1IN3_SendData_8Bit(0xD0);
    LCD_1IN3_SendData_8Bit(0x04);
    LCD_1IN3_SendData_8Bit(0x0D);
    LCD_1IN3_SendData_8Bit(0x11);
    LCD_1IN3_SendData_8Bit(0x13);
    LCD_1IN3_SendData_8Bit(0x2B);
    LCD_1IN3_SendData_8Bit(0x3F);
    LCD_1IN3_SendData_8Bit(0x54);
    LCD_1IN3_SendData_8Bit(0x4C);
    LCD_1IN3_SendData_8Bit(0x18);
    LCD_1IN3_SendData_8Bit(0x0D);
    LCD_1IN3_SendData_8Bit(0x0B);
    LCD_1IN3_SendData_8Bit(0x1F);
    LCD_1IN3_SendData_8Bit(0x23);

    LCD_1IN3_SendCommand(0xE1);  //Negative Voltage Gamma Control
    LCD_1IN3_SendData_8Bit(0xD0);
    LCD_1IN3_SendData_8Bit(0x04);
    LCD_1IN3_SendData_8Bit(0x0C);
    LCD_1IN3_SendData_8Bit(0x11);
    LCD_1IN3_SendData_8Bit(0x13);
    LCD_1IN3_SendData_8Bit(0x2C);
    LCD_1IN3_SendData_8Bit(0x3F);
    LCD_1IN3_SendData_8Bit(0x44);
    LCD_1IN3_SendData_8Bit(0x51);
    LCD_1IN3_SendData_8Bit(0x2F);
    LCD_1IN3_SendData_8Bit(0x1F);
    LCD_1IN3_SendData_8Bit(0x1F);
    LCD_1IN3_SendData_8Bit(0x20);
    LCD_1IN3_SendData_8Bit(0x23);

    LCD_1IN3_SendCommand(0x2A);  /* Column Address Set */  
    LCD_1IN3_SendData_8Bit(0x00);  
    LCD_1IN3_SendData_8Bit(0x00);  
    LCD_1IN3_SendData_8Bit(0x01);  
    LCD_1IN3_SendData_8Bit(0xEF);  /* 319 */  
    
    LCD_1IN3_SendCommand(0x2B);  /* Row Address Set */  
    LCD_1IN3_SendData_8Bit(0x00);  
    LCD_1IN3_SendData_8Bit(0x00);  
    LCD_1IN3_SendData_8Bit(0x00);  
    LCD_1IN3_SendData_8Bit(0xEF);  /* 239 */

    LCD_1IN3_SendCommand(0x21);  //Display Inversion On

    LCD_1IN3_SendCommand(0x11);  //Sleep Out

    LCD_1IN3_SendCommand(0x29);  //Display On
}

/********************************************************************************
function:	Set the resolution and scanning method of the screen
parameter:
		Scan_dir:   Scan direction
********************************************************************************/
static void LCD_1IN3_SetAttributes(uint8_t Scan_dir)
{
    //Get the screen scan direction
    LCD_1IN3.SCAN_DIR = Scan_dir;
    uint8_t MemoryAccessReg = 0x00;

    //Get GRAM and LCD width and height
    if(Scan_dir == HORIZONTAL) {
        LCD_1IN3.HEIGHT	= LCD_1IN3_WIDTH;
        LCD_1IN3.WIDTH   = LCD_1IN3_HEIGHT;
        MemoryAccessReg = 0X70;
    } else {
        LCD_1IN3.HEIGHT	= LCD_1IN3_HEIGHT;       
        LCD_1IN3.WIDTH   = LCD_1IN3_WIDTH;
        MemoryAccessReg = 0X00;
    }

    // Set the read / write scan direction of the frame memory
    LCD_1IN3_SendCommand(0x36); //MX, MY, RGB mode
    LCD_1IN3_SendData_8Bit(MemoryAccessReg);	//0x08 set RGB
}

/********************************************************************************
function :	Initialize the lcd
parameter:
********************************************************************************/
void LCD_1IN3_Init(uint8_t Scan_dir)
{
    // DEV_SET_PWM(90);
    DEV_Digital_Write(EPD_PWR_PIN, 1);
    //Hardware reset
    LCD_1IN3_Reset();

    //Set the resolution and scanning method of the screen
    LCD_1IN3_SetAttributes(Scan_dir);
    
    //Set the initialization register
    LCD_1IN3_InitReg();
}

/********************************************************************************
function:	Sets the start position and size of the display area
parameter:
		Xstart 	:   X direction Start coordinates
		Ystart  :   Y direction Start coordinates
		Xend    :   X direction end coordinates
		Yend    :   Y direction end coordinates
********************************************************************************/
void LCD_1IN3_SetWindows(uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend)
{
    //set the X coordinates
    LCD_1IN3_SendCommand(0x2A);
    LCD_1IN3_SendData_8Bit(0x00);
    LCD_1IN3_SendData_8Bit(Xstart);
	LCD_1IN3_SendData_8Bit(0x00);
    LCD_1IN3_SendData_8Bit(Xend-1);

    //set the Y coordinates
    LCD_1IN3_SendCommand(0x2B);
    LCD_1IN3_SendData_8Bit(0x00);
	LCD_1IN3_SendData_8Bit(Ystart);
	LCD_1IN3_SendData_8Bit(0x00);
    LCD_1IN3_SendData_8Bit(Yend-1);

    LCD_1IN3_SendCommand(0X2C);
    // printf("%d %d\r\n",x,y);
}

void LCD_1IN3_SetWindowsDMA(uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend)
{
    //set the X coordinates
    LCD_1IN3_SendCommand(0x2A);
    LCD_1IN3_SendData_8Bit(0x00);
    LCD_1IN3_SendData_8Bit(Xstart);
	LCD_1IN3_SendData_8Bit(0x00);
    LCD_1IN3_SendData_8Bit(Xend-1);

    //set the Y coordinates
    LCD_1IN3_SendCommand(0x2B);
    LCD_1IN3_SendData_8Bit(0x00);
	LCD_1IN3_SendData_8Bit(Ystart);
	LCD_1IN3_SendData_8Bit(0x00);
    LCD_1IN3_SendData_8Bit(Ystart + Yend-1);

    LCD_1IN3_SendCommand(0X2C);
    // printf("%d %d\r\n",x,y);
}

/******************************************************************************
function :	Clear screen
parameter:
******************************************************************************/
void LCD_1IN3_Clear(uint16_t Color)
{
    uint16_t j,i;
    (void)i;
    static uint16_t Image[LCD_1IN3_WIDTH];
    
    for (j = 0; j < LCD_1IN3_WIDTH; j++) {
        Image[j] = Color;
    }
    
    LCD_1IN3_SetWindows(0, 0, LCD_1IN3.WIDTH, LCD_1IN3.HEIGHT);
    DEV_Digital_Write(EPD_DC_PIN, 1);
    DEV_Digital_Write(EPD_CS_PIN, 0);

    for(j = 0; j < LCD_1IN3.HEIGHT; j++){
        DEV_SPI_Write_nByte((uint8_t *)Image, LCD_1IN3.WIDTH*2);
    }
    DEV_Digital_Write(EPD_CS_PIN, 1);
}

/******************************************************************************
function :	Sends the image buffer in RAM to displays
parameter:
******************************************************************************/
void LCD_1IN3_Display(uint16_t *Image)
{
    uint16_t j;
    LCD_1IN3_SetWindows(0, 0, LCD_1IN3.WIDTH, LCD_1IN3.HEIGHT);
    DEV_Digital_Write(EPD_DC_PIN, 1);
    DEV_Digital_Write(EPD_CS_PIN, 0);
    for (j = 0; j < LCD_1IN3.HEIGHT; j++) {
        DEV_SPI_Write_nByte((uint8_t *)&Image[j*LCD_1IN3.WIDTH], LCD_1IN3.WIDTH*2);
    }
    DEV_Digital_Write(EPD_CS_PIN, 1);
    LCD_1IN3_SendCommand(0x29);
}



void GLCD_DrawBitmap(   int_fast16_t x, int_fast16_t y, 
                        int_fast16_t width, int_fast16_t height,
                        uint16_t *frame_ptr)
{
    uint16_t j;
    (void)j;
    LCD_1IN3_SetWindows(x, y, x+width, y+height);
    DEV_Digital_Write(EPD_DC_PIN, 1);
    DEV_Digital_Write(EPD_CS_PIN, 0);
    
    DEV_SPI_Write_nByte((uint8_t *)frame_ptr, height*width*2);
    
    DEV_Digital_Write(EPD_CS_PIN, 1);
    LCD_1IN3_SendCommand(0x29);
}

void Disp0_DrawBitmap(  int16_t x, 
                        int16_t y, 
                        int16_t width, 
                        int16_t height, 
                        const uint8_t *bitmap)
{
    GLCD_DrawBitmap(x, y, width, height, (uint16_t *)bitmap);
}

volatile uint8_t sending_cmd = 0;





void __disp_adapter0_request_async_flushing(   
    void *pTarget,  
    bool bIsNewFrame,  
    int16_t iX,   
    int16_t iY,  
    int16_t iWidth,  
    int16_t iHeight,  
    const COLOUR_INT *pBuffer)  
{  
#if DMA_ASYNC_FLUSHING
    if(adapter0_async_flushing_data.async_flushing_step == 0)
    {
        
        adapter0_async_flushing_data.pTarget = pTarget;
        adapter0_async_flushing_data.bIsNewFrame = bIsNewFrame;
        adapter0_async_flushing_data.iX = iX;
        adapter0_async_flushing_data.iY = iY;
        adapter0_async_flushing_data.iWidth = iWidth;
        adapter0_async_flushing_data.iHeight = iHeight;
        adapter0_async_flushing_data.pBuffer = pBuffer;

        LCD_1IN3_SendCommand(0x2A);
        adapter0_async_flushing_data.async_flushing_step = 2;
    }
    else 
    {
        // adapter0_async_flushing_data.async_flushing_step = 255;
    }
#else
    HAL_StatusTypeDef status;    
    uint32_t dataSize = (uint32_t)iWidth * iHeight * sizeof(COLOUR_INT);   
      
    if (NULL == pBuffer || 0 == dataSize) {    
        return;    
    }    
         
    /* 发送列地址命令 */    
    sending_cmd = 1;
    LCD_1IN3_SetWindows(iX, iY, iX+iWidth, iY + iHeight);
    while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
    sending_cmd = 0;

    /* 启动 DMA 传输 */    
    DEV_Digital_Write(EPD_DC_PIN, 1);
    DEV_Digital_Write(EPD_CS_PIN, 0);    
    status = HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *)pBuffer, dataSize);    
        
    if (HAL_OK != status) {    
        DEV_Digital_Write(EPD_CS_PIN, 1);  
        disp_adapter0_insert_async_flushing_complete_event_handler();    
    }
#endif  
}

volatile uint8_t data_buf[4] = {0};
void arm2d_async_flushing_loop(ARM2D_ASYNC_FLUSHING_DATA *data)
{
    HAL_StatusTypeDef status;
    uint32_t dataSize = (uint32_t)data->iWidth * data->iHeight * sizeof(COLOUR_INT);   

    if(data->async_flushing_step == 0)
    {
        return;
    }
    
    if (NULL == data->pBuffer || 0 == dataSize) 
    {    
        data->async_flushing_step = 0xE0;
        return;    
    } 

    if(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
    {
        return;
    }
    switch (data->async_flushing_step) 
    {
        case 1:
            LCD_1IN3_SendCommand(0x2A);
            data->async_flushing_step = 2;
        break;

        case 2:
            data_buf[0] = 0x00;
            data_buf[1] = data->iX;
            data_buf[2] = 0x00;
            data_buf[3] = data->iX + data->iWidth - 1;
            DEV_Digital_Write(EPD_DC_PIN, 1);
            DEV_Digital_Write(EPD_CS_PIN, 0);
            HAL_SPI_Transmit(&hspi1, (uint8_t *)data_buf, 4, HAL_MAX_DELAY);
            data->async_flushing_step = 3;
        break;

        case 3:
            LCD_1IN3_SendCommand(0x2B);
            data->async_flushing_step = 4;
        break;

        case 4:
            data_buf[0] = 0x00;
            data_buf[1] = data->iY;
            data_buf[2] = 0x00;
            data_buf[3] = data->iY + data->iHeight - 1;

            DEV_Digital_Write(EPD_DC_PIN, 1);
            DEV_Digital_Write(EPD_CS_PIN, 0);
            HAL_SPI_Transmit(&hspi1, (uint8_t *)data_buf, 4, HAL_MAX_DELAY);
            data->async_flushing_step = 5;
        break;

        case 5:
            LCD_1IN3_SendCommand(0x2C);
            data->async_flushing_step = 6;
        break;

        case 6:
            DEV_Digital_Write(EPD_DC_PIN, 1);
            DEV_Digital_Write(EPD_CS_PIN, 0);
            status = HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *)data->pBuffer, dataSize);    
        
            if (HAL_OK != status) {    
                DEV_Digital_Write(EPD_CS_PIN, 1);  
                disp_adapter0_insert_async_flushing_complete_event_handler();    
            }
            data->async_flushing_step = 0;
        break;

        default:
            break;
    }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if(hspi->Instance == SPI1)
  {
    DEV_Digital_Write(EPD_CS_PIN, 1);
    // 通知 Arm-2D DMA 传输完成  
    if(sending_cmd == 1)
    {
        sending_cmd = 255;
    }
    else 
    {
        disp_adapter0_insert_async_flushing_complete_event_handler();  
    }
    
  }
}

/* DMA 传输错误中断回调函数 */  
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)  
{  
    if (hspi == &hspi1) {  
        /* 错误处理，恢复 CS 信号 */  
        DEV_Digital_Write(EPD_CS_PIN, 1);
          
        /* 通知 Arm-2D 传输完成（错误情况） */  
        disp_adapter0_insert_async_flushing_complete_event_handler();  
    }  
}
#if 0
void Disp0_DrawBitmap_Test(void)
{
    uint16_t i = 0, j = 0;
    uint8_t test_buf[LCD_1IN3_WIDTH*2];

    for (i = 0; i < LCD_1IN3_HEIGHT; i++) 
    {
        for (j = 0; j < LCD_1IN3_WIDTH*2; j++) 
        {
            test_buf[j] = i;
        }
        Disp0_DrawBitmap(0, i, LCD_1IN3_WIDTH, 1, test_buf);
    }
}
#endif

void LCD_1IN3_DisplayWindows(uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend, uint16_t *Image)
{
    // display
    uint32_t Addr = 0;

    uint16_t j;
    LCD_1IN3_SetWindows(Xstart, Ystart, Xend , Yend);
    DEV_Digital_Write(EPD_DC_PIN, 1);
    DEV_Digital_Write(EPD_CS_PIN, 0);
    for (j = Ystart; j < Yend - 1; j++) {
        Addr = Xstart + j * LCD_1IN3.WIDTH ;
        DEV_SPI_Write_nByte((uint8_t *)&Image[Addr], (Xend-Xstart)*2);
    }
    DEV_Digital_Write(EPD_CS_PIN, 1);
}

void LCD_1IN3_DisplayPoint(uint16_t X, uint16_t Y, uint16_t Color)
{
    LCD_1IN3_SetWindows(X,Y,X,Y);
    LCD_1IN3_SendData_16Bit(Color);
}
