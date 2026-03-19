/******************************************************************************
 * @file    powermodes.c
 * @brief   System power state management and shutdown control
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

#define WAKEUP_TIMEOUT_SEC             10U
#define RTC_TIMER_PRESCALER            RTC_WAKEUPCLOCK_CK_SPRE_16BITS

/***** Local Types ***********************************************************/

typedef enum
{
    ButtonState_Idle = 0,
    ButtonState_Released,
    ButtonState_Pressed
} ButtonSequenceState;

/***** Static Function Prototypes ********************************************/

static void EnterLowPowerMode(void);
static void HandleWakeupIndicators(void);

/***** Static Variables ******************************************************/

static ButtonSequenceState g_buttonSequence = ButtonState_Idle;

/***** Public Functions ******************************************************/

/*!
 * @brief Initialize power mode management and detect wakeup source.
 */
/*!
 * @brief Initialize power mode management and detect wakeup source.
 */
void PowerModes_Init(void)
{
    /* Only reset button sequence on cold start, NOT on wakeup */
    if ((__HAL_PWR_GET_FLAG(PWR_FLAG_SB) == 0U) &&
            (__HAL_PWR_GET_FLAG(PWR_FLAG_WUF2) == 0U) &&
            ((RTC->SR & RTC_SR_WUTF) == 0U))
    {
        /* Cold start - reset button state */
        g_buttonSequence = ButtonState_Idle;
    }

    else
    {
        /* Wakeup - keep button state for consistency */
        g_buttonSequence = ButtonState_Idle;  /* Reset nach Wakeup, bereit für neuen 2-Click */
    }

    /* Disable any active RTC wakeup timer */
    (void)HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

    /* Clear all LED indicators first */
    HAL_GPIO_WritePin(LED_D0_GPIO_Port, LED_D0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_D1_GPIO_Port, LED_D1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_D2_GPIO_Port, LED_D2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_D3_GPIO_Port, LED_D3_Pin, GPIO_PIN_RESET);

    /* Set LED indicators based on wakeup source */
    HandleWakeupIndicators();

    /* Clear wakeup flags for next cycle */
    RTC->SCR = RTC_SCR_CWUTF;
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF2);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
}


/*!
 * @brief Process power mode cyclic operations.
 */
void PowerModes_Cyclic(void)
{
    /* Placeholder for future power management tasks */
}

/*!
 * @brief Handle button state change for power mode transitions.
 *
 * @param isPressed Current button press state.
 */
void PowerModes_ButtonEvent(bool isPressed)
{
    if (g_buttonSequence == ButtonState_Idle)
    {
        if (isPressed == false)
        {
            g_buttonSequence = ButtonState_Released;
        }
    }

    else if (g_buttonSequence == ButtonState_Released)
    {
        if (isPressed == true)
        {
            g_buttonSequence = ButtonState_Pressed;
        }
    }

    else if (g_buttonSequence == ButtonState_Pressed)
    {
        if (isPressed == false)
        {
            EnterLowPowerMode();
        }
    }
}

/***** Static Functions ******************************************************/

/*!
 * @brief Detect wakeup source and illuminate corresponding LED.
 */
static void HandleWakeupIndicators(void)
{
    bool systemWokeFromShutdown;
    bool wakeSourceButton;
    bool wakeSourceTimer;

    systemWokeFromShutdown = (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != 0U);
    wakeSourceButton = (__HAL_PWR_GET_FLAG(PWR_FLAG_WUF2) != 0U);
    wakeSourceTimer = ((RTC->SR & RTC_SR_WUTF) != 0U);

    if ((systemWokeFromShutdown == true) && (wakeSourceButton == true))
    {
        /* LED D2: Woken by button press */
        HAL_GPIO_WritePin(LED_D2_GPIO_Port, LED_D2_Pin, GPIO_PIN_SET);
    }

    else if ((systemWokeFromShutdown == true) && (wakeSourceTimer == true))
    {
        /* LED D3: Woken by RTC timeout */
        HAL_GPIO_WritePin(LED_D3_GPIO_Port, LED_D3_Pin, GPIO_PIN_SET);
    }

    else
    {
        /* Cold start or other wakeup source */
    }
}

/*!
 * @brief Gracefully shutdown system and enter low power state.
 *
 * Performs final cleanup, disables peripherals, and transitions to shutdown mode.
 */
static void EnterLowPowerMode(void)
{
    /* Save runtime metrics before power down */
    Storage_PrepareShutdown();

    /* Disable sensor power supply */
    Acceleration_PrepareShutdown();

    /* Turn off all indicator LEDs */
    HAL_GPIO_WritePin(LED_D0_GPIO_Port, LED_D0_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_D1_GPIO_Port, LED_D1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_D2_GPIO_Port, LED_D2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_D3_GPIO_Port, LED_D3_Pin, GPIO_PIN_RESET);

    /* Reset button state machine */
    g_buttonSequence = ButtonState_Idle;

    /* Deactivate any pending RTC wakeup */
    (void)HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);

    /* Clear all pending wakeup flags */
    RTC->SCR = RTC_SCR_CWUTF;
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF2);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);

    /* Configure GPIO pulldown on wakeup pin for stable voltage */
    HAL_PWREx_EnableGPIOPullDown(PWR_GPIO_C, PWR_GPIO_BIT_13);
    HAL_PWREx_EnablePullUpPullDownConfig();

    /* Enable wakeup from button on PWR_WAKEUP_PIN2 (high level) */
    HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2_HIGH);

    /* Arm RTC timer for automatic wakeup after timeout */
    if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc,
                                    WAKEUP_TIMEOUT_SEC,
                                    RTC_TIMER_PRESCALER) != HAL_OK)
    {
        Error_Handler();
    }

    /* Enter shutdown mode - system will not resume from this point */
    HAL_PWREx_EnterSHUTDOWNMode();

    /* Force system reset on wakeup */
    HAL_NVIC_SystemReset();
}
