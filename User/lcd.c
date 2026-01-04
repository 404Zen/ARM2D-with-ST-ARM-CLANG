#include "lcd.h"
#include "main.h"
#include <stdint.h>
#include "spi.h"
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_gpio.h"



volatile uint8_t gram[2*240*24];            // 1/10 BUFFER

// test variables
volatile uint8_t g_dma_ptr = 0;
volatile uint8_t g_full_scr_done = 0;

static void LCD_Send_CMD(uint8_t command);
static void LCD_Send_OneByte(uint8_t data);
// static void LCD_Send_Buffer(uint8_t *buf_ptr, uint16_t len);

/* LCD Transmit Port*/
void LCD_WriteData_Port(uint8_t *data, uint16_t len)
{
    SPI1_WriteData(data, len);
}


void LCD_FillScreen(uint16_t color)
{
    uint16_t i, j;
    uint8_t data[2] = {0};
    (void)i;

    data[0] = color >> 8;
    data[1] = color;


    LCD_Set_Address(0, 0, 240, 240);

    for(j = 0; j < 115200 / 2; j++)
    {
        gram[j * 2] =  data[0];
        gram[j * 2 + 1] =  data[1];
    }


    HAL_GPIO_WritePin(LCD_WR_GPIO_Port, LCD_WR_Pin, GPIO_PIN_SET);

#if 0
    for(i = 0; i < (LCD_FULL_SCR_SIZE / sizeof(gram)); i++)
    {
        
        LCD_Send_Buffer((uint8_t *)gram, (uint16_t)sizeof(gram));
    }
#else   /* DMA*/
    // HAL_GPIO_WritePin(LCD_WR_GPIO_Port, LCD_WR_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *)gram, (uint16_t)sizeof(gram));


#endif
}

// void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
// {
//     if(g_dma_ptr < (LCD_FULL_SCR_SIZE / sizeof(gram)) )
//     {
//         g_dma_ptr++;
//         HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *)gram, (uint16_t)sizeof(gram));
//     }
//     else 
//     {
//         g_dma_ptr = 0;
//         HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
//         g_full_scr_done = 1;
//     }
// }




void LCD_Init(void)
{
    uint16_t i = 0;
    (void)i;

    HAL_Delay(120);

    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_PWR_GPIO_Port, LCD_PWR_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(120);
    HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);

    HAL_Delay(120);

    /* Sleep Out */
    LCD_Send_CMD(0x11);
    /* wait for power stability */
    HAL_Delay(120);

    /* Memory Data Access Control */
    LCD_Send_CMD(0x36);
    LCD_Send_OneByte(0x00);

    /* RGB 5-6-5-bit  */
    LCD_Send_CMD(0x3A);
    LCD_Send_OneByte(0x65);

    /* Porch Setting */
    LCD_Send_CMD(0xB2);
    LCD_Send_OneByte(0x0C);
    LCD_Send_OneByte(0x0C);
    LCD_Send_OneByte(0x00);
    LCD_Send_OneByte(0x33);
    LCD_Send_OneByte(0x33);

    /*  Gate Control */
    LCD_Send_CMD(0xB7);
    LCD_Send_OneByte(0x72);

    /* VCOM Setting */
    LCD_Send_CMD(0xBB);
    LCD_Send_OneByte(0x3D);

    /* LCM Control */
    LCD_Send_CMD(0xC0);
    LCD_Send_OneByte(0x2C);

    /* VDV and VRH Command Enable */
    LCD_Send_CMD(0xC2);
    LCD_Send_OneByte(0x01);

    /* VRH Set */
    LCD_Send_CMD(0xC3);
    LCD_Send_OneByte(0x19);

    /* VDV Set */
    LCD_Send_CMD(0xC4);
    LCD_Send_OneByte(0x20);

    /* Frame Rate Control in Normal Mode */
    LCD_Send_CMD(0xC6);
    LCD_Send_OneByte(0x0F);

    /* Power Control 1 */
    LCD_Send_CMD(0xD0);
    LCD_Send_OneByte(0xA4);
    LCD_Send_OneByte(0xA1);

    /* Positive Voltage Gamma Control */
    LCD_Send_CMD(0xE0);
    LCD_Send_OneByte(0xD0);
    LCD_Send_OneByte(0x04);
    LCD_Send_OneByte(0x0D);
    LCD_Send_OneByte(0x11);
    LCD_Send_OneByte(0x13);
    LCD_Send_OneByte(0x2B);
    LCD_Send_OneByte(0x3F);
    LCD_Send_OneByte(0x54);
    LCD_Send_OneByte(0x4C);
    LCD_Send_OneByte(0x18);
    LCD_Send_OneByte(0x0D);
    LCD_Send_OneByte(0x0B);
    LCD_Send_OneByte(0x1F);
    LCD_Send_OneByte(0x23);

    /* Negative Voltage Gamma Control */
    LCD_Send_CMD(0xE1);
    LCD_Send_OneByte(0xD0);
    LCD_Send_OneByte(0x04);
    LCD_Send_OneByte(0x0C);
    LCD_Send_OneByte(0x11);
    LCD_Send_OneByte(0x13);
    LCD_Send_OneByte(0x2C);
    LCD_Send_OneByte(0x3F);
    LCD_Send_OneByte(0x44);
    LCD_Send_OneByte(0x51);
    LCD_Send_OneByte(0x2F);
    LCD_Send_OneByte(0x1F);
    LCD_Send_OneByte(0x1F);
    LCD_Send_OneByte(0x20);
    LCD_Send_OneByte(0x23);

    /* Display Inversion On */
    LCD_Send_CMD(0x21);

    LCD_Send_CMD(0x29);

    LCD_Set_Address(0, 239, 0, 239);


    LCD_FillScreen(0xFFFF);
    // lcd_clear(0xF800);
    // for(i = 0; i < (uint16_t)115200; i++)
    // {
    //     // gram[i] = 0xFF;
    //     LCD_Send_OneByte(0xFF);
    // }

    HAL_GPIO_WritePin(LCD_PWR_GPIO_Port, LCD_PWR_Pin, GPIO_PIN_SET);

    // LCD_Send_Buffer((uint8_t *)gram, (uint16_t)115200);

}


void LCD_Set_Address(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    LCD_Send_CMD(0x2A);
    // LCD_Send_OneByte(x1 >> 8);
    LCD_Send_OneByte(0x00);
    LCD_Send_OneByte(x1);
    LCD_Send_OneByte(0x00);
    LCD_Send_OneByte(x2-1);

    LCD_Send_CMD(0x2B);
    LCD_Send_OneByte(0x00);
    LCD_Send_OneByte(y1);
    LCD_Send_OneByte(0x00);
    LCD_Send_OneByte(y2-1);

    LCD_Send_CMD(0x2C);

}


static void LCD_Send_CMD(uint8_t command)
{
    HAL_GPIO_WritePin(LCD_WR_GPIO_Port, LCD_WR_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
    SPI1_WriteData(&command, 1);
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}
static void LCD_Send_OneByte(uint8_t data)
{
    HAL_GPIO_WritePin(LCD_WR_GPIO_Port, LCD_WR_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
    SPI1_WriteData(&data, 1);
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

/* 测试发现，单次刷屏最多只能刷半屏 */
#if 0
static void LCD_Send_Buffer(uint8_t *buf_ptr, uint16_t len)
{
    HAL_GPIO_WritePin(LCD_WR_GPIO_Port, LCD_WR_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
    SPI1_WriteData((uint8_t *)buf_ptr, (uint16_t)len);
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}
#endif


/* ARM2D Port */
#if 0
void Disp0_DrawBitmap(uint32_t x, uint32_t y, uint32_t width, uint32_t height, const uint8_t *bitmap)
{
    LCD_Set_Address(x, y, x+width, y+height);

    HAL_GPIO_WritePin(LCD_WR_GPIO_Port, LCD_WR_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);

    HAL_SPI_Transmit_DMA(&hspi1, (uint8_t *)bitmap, (uint16_t)height*width*2);

    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);

    LCD_Send_CMD(0x29);
}
#endif
#if 0
void GLCD_DrawBitmap(   int_fast16_t x, int_fast16_t y, 
                        int_fast16_t width, int_fast16_t height,
                        uint16_t *frame_ptr)
{
    UWORD j;
    LCD_1IN3_SetWindows(x, y, x+width, y+height);
    DEV_Digital_Write(EPD_DC_PIN, 1);
    DEV_Digital_Write(EPD_CS_PIN, 0);
    
    DEV_SPI_Write_nByte((uint8_t *)frame_ptr, height*width*2);
    
    DEV_Digital_Write(EPD_CS_PIN, 1);
    LCD_1IN3_SendCommand(0x29);
}
#endif


/*---------------------- END OF FILE ----------------------*/
