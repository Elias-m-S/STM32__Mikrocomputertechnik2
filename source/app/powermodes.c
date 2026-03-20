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

#define POWERMODES_WAKEUP_TIME_S      10U
#define POWERMODES_RTC_CLOCK_DIVIDER  RTC_WAKEUPCLOCK_CK_SPRE_16BITS

/***** Local Types ***********************************************************/

typedef enum
{
    PowerMode_WaitFirstRelease = 0, /*!< Wait for first button release */
    PowerMode_WaitSecondPress, /*!< Wait for second button press */
    PowerMode_WaitSecondRelease /*!< Wait for second button release -> shutdown */
} PowerMode_State;

/***** Static Function Prototypes ********************************************/

static void PowerModes_EnterShutdown(void);

/***** Static Variables ******************************************************/

static PowerMode_State s_state = PowerMode_WaitFirstRelease;

/***** Public Functions ******************************************************/

/*!
 * @brief Initialize power mode module and evaluate wakeup source.
 *
 */
void PowerModes_Init(void)
{
    bool wokeFromShutdown;
    bool wokeByButton;
    bool wokeByRtc;

    /* Disable RTC wakeup timer - reconfigured before next shutdown */
    (void) HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

    /* Read wakeup flags before clearing them */
    wokeFromShutdown = (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != 0U);
    wokeByButton = (__HAL_PWR_GET_FLAG(PWR_FLAG_WUF2) != 0U);
    wokeByRtc = ((RTC->SR & RTC_SR_WUTF) != 0U);

    /* Turn off all LEDs */
    HAL_GPIO_WritePin(LED_D0_GPIO_Port, LED_D0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_D1_GPIO_Port, LED_D1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_D2_GPIO_Port, LED_D2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_D3_GPIO_Port, LED_D3_Pin, GPIO_PIN_RESET);

    if ((wokeFromShutdown == true) && (wokeByButton == true))
    {
        /* Button wakeup: LED D2 on, skip first release */
        HAL_GPIO_WritePin(LED_D2_GPIO_Port, LED_D2_Pin, GPIO_PIN_SET);
        s_state = PowerMode_WaitSecondPress;
    }

    else if ((wokeFromShutdown == true) && (wokeByRtc == true))
    {
        /* RTC wakeup: LED D3 on */
        HAL_GPIO_WritePin(LED_D3_GPIO_Port, LED_D3_Pin, GPIO_PIN_SET);
        s_state = PowerMode_WaitFirstRelease;
    }

    else
    {
        /* Normal power-on or pin reset */
        s_state = PowerMode_WaitFirstRelease;
    }

    /* Clear all wakeup flags */
    RTC->SCR = RTC_SCR_CWUTF;
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF2);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
}

/*!
 * @brief Process a button press or release event (called from EXTI ISR).
 *
 */
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

    else /* PowerMode_WaitSecondRelease */
    {
        if (pressed == false)
        {
            PowerModes_EnterShutdown();
        }
    }
}

/***** Static Functions ******************************************************/

/*!
 * @brief Perform a clean transition into STM32G4 Shutdown mode.
 */
static void PowerModes_EnterShutdown(void)
{
    /* Persist runtime data and disable sensor supply */
    Storage_PrepareShutdown();
    Acceleration_Shutdown();

    /* Turn off all LEDs */
    HAL_GPIO_WritePin(LED_D0_GPIO_Port, LED_D0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_D1_GPIO_Port, LED_D1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_D2_GPIO_Port, LED_D2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_D3_GPIO_Port, LED_D3_Pin, GPIO_PIN_RESET);

    /* Reset state machine for the next wakeup session */
    s_state = PowerMode_WaitFirstRelease;

    /* Deactivate RTC wakeup timer and clear all flags */
    (void) HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
    RTC->SCR = RTC_SCR_CWUTF;
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF2);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);

    /* WKUP2 (PC13): pull-down keeps line LOW during Shutdown;
     * HIGH polarity wakes on button release (rising edge). */
    HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_C, PWR_GPIO_BIT_13);
    HAL_PWREx_EnablePullUpPullDownConfig();
    HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2_HIGH);

    /* Clear WUF2 again - EnableWakeUpPin sets it immediately because
     * PC13 is already HIGH when called from the release callback. */
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF2);

    /* Arm RTC wakeup timer (WUTIE required to exit Shutdown, see RM0440 Table 337) */
    if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc,
                                    POWERMODES_WAKEUP_TIME_S,
                                    POWERMODES_RTC_CLOCK_DIVIDER) != HAL_OK)
    {
        Error_Handler();
    }

    /* Enter Shutdown - SRAM content lost */
    HAL_PWREx_EnterSHUTDOWNMode();

    /* Should never be reached */
    HAL_NVIC_SystemReset();
}
