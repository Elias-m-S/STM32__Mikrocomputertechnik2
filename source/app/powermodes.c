/******************************************************************************
 * @file    powermodes.c
 * @brief   Power mode handling
 ******************************************************************************/

/***** Includes **************************************************************/
#include "powermodes.h"

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_pwr_ex.h"
#include "stm32g4xx_hal_rtc_ex.h"
#include "rtc.h"
#include "gpio.h"
#include "acceleration.h"
#include "storage.h"

/***** Defines ***************************************************************/

#define POWERMODES_WAKEUP_TIME_S          10U
#define POWERMODES_RTC_CLOCK_DIVIDER      RTC_WAKEUPCLOCK_CK_SPRE_16BITS

/***** Local Types ***********************************************************/

typedef enum
{
    PowerMode_WaitFirstRelease = 0,
    PowerMode_WaitSecondPress,
    PowerMode_WaitSecondRelease
} PowerMode_State;

/***** Static Function Prototypes ********************************************/

static void PowerModes_EnterShutdown(void);

/***** Static Variables ******************************************************/

static PowerMode_State s_state = PowerMode_WaitFirstRelease;

/***** Public Functions ******************************************************/

void PowerModes_Init(void)
{
    bool wokeFromShutdown;
    bool wokeByButton;
    bool wokeByRtc;

    s_state = PowerMode_WaitFirstRelease;

    (void)HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

    wokeFromShutdown = (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != 0U);
    wokeByButton = (__HAL_PWR_GET_FLAG(PWR_FLAG_WUF2) != 0U);
    wokeByRtc = ((RTC->SR & RTC_SR_WUTF) != 0U);

    HAL_GPIO_WritePin(LED_D0_GPIO_Port, LED_D0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_D1_GPIO_Port, LED_D1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_D2_GPIO_Port, LED_D2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_D3_GPIO_Port, LED_D3_Pin, GPIO_PIN_RESET);

    if ((wokeFromShutdown == true) && (wokeByButton == true))
    {
        HAL_GPIO_WritePin(LED_D2_GPIO_Port, LED_D2_Pin, GPIO_PIN_SET);
    }

    else if ((wokeFromShutdown == true) && (wokeByRtc == true))
    {
        HAL_GPIO_WritePin(LED_D3_GPIO_Port, LED_D3_Pin, GPIO_PIN_SET);
    }

    else
    {
        /* No dedicated wakeup indication */
    }

    RTC->SCR = RTC_SCR_CWUTF;
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF2);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
}

void PowerModes_Cyclic(void)
{
    /* Reserved for future runtime handling */
}

void PowerModes_ButtonEvent(bool pressed)
{
    if (s_state == PowerMode_WaitFirstRelease)
    {
        if (pressed == false)
        {
            s_state = PowerMode_WaitSecondPress;
        }
    }

    else if (s_state == PowerMode_WaitSecondPress)
    {
        if (pressed == true)
        {
            s_state = PowerMode_WaitSecondRelease;
        }
    }

    else
    {
        if (pressed == false)
        {
            PowerModes_EnterShutdown();
        }
    }
}

/***** Static Functions ******************************************************/

static void PowerModes_EnterShutdown(void)
{
    /* Persist runtime before shutdown */
    Storage_PrepareShutdown();

    /* Sensor supply must be disabled before shutdown */
    Acceleration_PrepareShutdown();

    /* All LEDs must be off before shutdown */
    HAL_GPIO_WritePin(LED_D0_GPIO_Port, LED_D0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_D1_GPIO_Port, LED_D1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_D2_GPIO_Port, LED_D2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_D3_GPIO_Port, LED_D3_Pin, GPIO_PIN_RESET);

    s_state = PowerMode_WaitFirstRelease;

    (void)HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

    RTC->SCR = RTC_SCR_CWUTF;
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF2);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);

    HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_C, PWR_GPIO_BIT_13);
    HAL_PWREx_EnablePullUpPullDownConfig();

    HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2_HIGH);

    if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc,
                                    POWERMODES_WAKEUP_TIME_S,
                                    POWERMODES_RTC_CLOCK_DIVIDER) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_PWREx_EnterSHUTDOWNMode();

    HAL_NVIC_SystemReset();
}
