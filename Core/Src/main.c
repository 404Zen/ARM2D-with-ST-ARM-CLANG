/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// #include "lcd.h"
#include "LCD_1in3.h"
#include <stdbool.h>
#include <stdint.h>
#include "arm_2d.h"
#include "system_stm32g4xx.h"


/* ARM 2D */
#include "perf_counter.h"

#include "arm_2d_helper.h"
#include "arm_2d_disp_adapters.h"
#include "arm_2d_example_controls.h"
// #include "arm_2d_scenes.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// static  
// IMPL_PFB_ON_DRAW(__pfb_draw_handler)  
// {  
//     arm_2d_canvas(ptTile, __top_container) {  
//         arm_2d_fill_colour(ptTile, NULL, GLCD_COLOR_WHITE);  
          
//         arm_2d_align_centre(__top_container, 100, 100) {  
//             arm_2d_fill_colour(ptTile, &__centre_region, GLCD_COLOR_BLUE);  
//         }  
//     }  
      
//     arm_2d_op_wait_async(NULL);  
//     return arm_fsm_rt_cpl;  
// }  
  
static  
IMPL_PFB_ON_DRAW(__pfb_draw_handler)  
{  
    arm_2d_canvas(ptTile, __top_container) {  
        arm_2d_fill_colour(ptTile, NULL, GLCD_COLOR_WHITE);  
          
        // 显示转圈动画  
        busy_wheel2_show(ptTile, bIsNewFrame);  
    }  
      
    arm_2d_op_wait_async(NULL);  
    return arm_fsm_rt_cpl;  
}

arm_2d_scene_t *ptScenes[] = {  
    &(arm_2d_scene_t){  
        .fnScene = __pfb_draw_handler,  
        .tCanvas = GLCD_COLOR_WHITE,  
        
    },  
};

// 确保所有必要的函数指针都已设置  
arm_2d_scene_t my_scene = {  
    .fnScene = &__pfb_draw_handler,      // 必须设置  
    .fnOnFrameStart = NULL,           // 可选  
    .fnOnFrameCPL = NULL,             // 可选  
    .tCanvas = GLCD_COLOR_WHITE,      // 背景色  
    .bUseDirtyRegionHelper = false,   // 根据需要设置  
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
// uint64_t arm_2d_helper_get_system_timestamp(void)  
// {  
//     return get_system_ticks();  
// }  
  
// uint32_t arm_2d_helper_get_reference_clock_frequency(void)  
// {  
//     return SystemCoreClock;  
// }
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
bool check_dwt_enabled(void)  
{  
    // 检查 Debug Exception and Monitor Control Register  
    uint32_t demcr = CoreDebug->DEMCR;  
    bool debug_enabled = (demcr & CoreDebug_DEMCR_TRCENA_Msk) != 0;  
      
    // 检查 DWT Control Register  
    uint32_t dwt_ctrl = DWT->CTRL;  
    bool dwt_enabled = (dwt_ctrl & DWT_CTRL_CYCCNTENA_Msk) != 0;  
      
    // printf("DEMCR: 0x%08lX (TRCENA=%s)\r\n",   
    //        demcr, debug_enabled ? "ON" : "OFF");  
    // printf("DWT_CTRL: 0x%08lX (CYCCNTENA=%s)\r\n",   
    //        dwt_ctrl, dwt_enabled ? "ON" : "OFF");  
      
    return debug_enabled && dwt_enabled;  
}

void enable_dwt(void)  
{  
    // 启用 Debug Exception and Monitor Control Register  
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  
      
    // 启用 DWT cycle counter  
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;  
      
    // 重置计数器  
    DWT->CYCCNT = 0;  
      
    // printf("DWT enabled\r\n");  
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  volatile uint32_t start_tick = 0;
  arm_fsm_rt_t tResult;
  (void)tResult;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  // ST-CLANG both define __GNUC__ and __clang__

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  

  
  if(check_dwt_enabled() ==  false)
  {
    enable_dwt();
  }

  perfc_init(true);
  LCD_1IN3_Init(VERTICAL);
  LCD_1IN3_Clear(0xFFFF);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /*! \note if you do want to use SysTick in your application, please use 
     *!       init_cycle_counter(true); 
     *!       instead of 
     *!       init_cycle_counter(false); 
     */
  
  
  arm_irq_safe 
  {
    arm_2d_init();
  }
  // arm_lcd_text_init();
  // arm_2d_helper_init();
  /* initialize the display adapter 0 service */
  disp_adapter0_init();
  
  // extern arm_2d_scene_t *ptScenes[];  
    // arm_2d_scene_player_append_scenes(&DISP0_ADAPTER, &my_scene, 1);  
  start_tick = HAL_GetTick();

  // arm_lcd_text_location(0, 0);  
  // arm_lcd_printf("TEST NAV");  // 强制显示测试文本

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // int32_t iCycleResult = 0;
    tResult = disp_adapter0_task();
    if(HAL_GetTick() - start_tick >= 500)
    {
      start_tick = HAL_GetTick();
      HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
    }
    // perfc_delay_ms(500);
    // HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
