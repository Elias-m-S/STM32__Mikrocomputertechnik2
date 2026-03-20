/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/

#include "stm32g4xx_hal.h"
#include "gpio.h"
#include "dma.h"
#include "tim.h"
#include "crc.h"
#include "i2c.h"
#include "usart.h"
#include "rtc.h"

#include "main_state.h"

#include "stm32g4xx_hal_rtc.h"
#include "stm32g4xx_hal_rtc_ex.h"

/*******************************************************************************
 * Defines
 *******************************************************************************/

/*******************************************************************************
 * Local Types and Typedefs
 *******************************************************************************/

/*******************************************************************************
 * Global Variables
 *******************************************************************************/

/*******************************************************************************
 * Static Variables
 *******************************************************************************/

/** @brief Global application main state instance. */
static MainState s_mainState;

/*******************************************************************************
 * Static Function Prototypes
 *******************************************************************************/

/**
 * @brief System Clock Configuration
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

    /** Configure the main internal regulator output voltage
     */
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

    /** Configure LSE drive capability
     */
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
                                       | RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
    RCC_OscInitStruct.PLL.PLLN = 20;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
    {
        Error_Handler();
    }
}

/*!
 * @brief Initializes all timer peripherals in use
 */
static void MX_TIM_Init(void)
{
    /* TIM1: Main state timing */
    MX_TIM1_Init();

    /* TIM2: Free-running measurement timer */
    MX_TIM2_Init();
}

/*!
 * @brief Initializes all ADC peripherals
 */
static void MX_ADC_Init(void)
{
}

/*******************************************************************************
 * Public Functions
 *******************************************************************************/

/**
 * @brief  Main program entry
 * @retval int
 */
int main(void)
{
    /* Reset of all peripherals, initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Stop timers and RTC while debugging */
    DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_TIM2_STOP;
    DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_RTC_STOP;
    DBGMCU->APB2FZ |= DBGMCU_APB2FZ_DBG_TIM1_STOP;

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_CRC_Init();
    MX_DMA_Init();
    MX_TIM_Init();
    MX_RTC_Init();
    MX_I2C3_Init();
    MX_LPUART1_UART_Init();
    MX_ADC_Init();

    /* Initialize application state machine */
    MainState_Init(&s_mainState);

    while (1u)
    {
        MainState_Cyclic(&s_mainState);
    }

    return 0;
}

/**
 * @brief  This function is executed in case of error occurrence.
 */
void Error_Handler(void)
{
    __disable_irq();

    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file Pointer to the source file name
 * @param  line Source line number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
    (void) file;
    (void) line;
}
#endif /* USE_FULL_ASSERT */
