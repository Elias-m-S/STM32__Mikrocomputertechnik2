/**
 * @file      run_state.c
 * @author    Christian Hildenbrand
 * @date      01.05.2023
 * @brief     Run state handling.
 */

/*******************************************************************************
 * Includes
 *******************************************************************************/

#include <stdbool.h>

#include "run_state.h"

#include "tim.h"
#include "stm32g4xx_ll_tim.h"
#include "gpio.h"
#include "powermodes.h"
#include "acceleration.h"
#include "processing.h"
#include "storage.h"
#include "main.h"

/*******************************************************************************
 * Defines
 *******************************************************************************/

/** @brief Toggle period in milliseconds for 1 Hz LED blinking. */
#define RUN_LED_TOGGLE   500U

/*******************************************************************************
 * Local Types and Typedefs
 *******************************************************************************/

/*******************************************************************************
 * Global Variables
 *******************************************************************************/

/*******************************************************************************
 * Static Function Prototypes
 *******************************************************************************/

static bool RunState_Cyclic_SelfTest(RunState const* pRunState, bool* pError);
static void RunState_Cyclic_Running(RunState* const pRunState);
static void RunState_Cyclic_Running_HandleLED_D1(uint32_t currentTick);
static void RunState_Cyclic_Error(RunState const* pRunState);

/*******************************************************************************
 * Static Variables
 *******************************************************************************/

/** @brief Last tick used for LED toggling in run mode. */
static uint32_t s_lastToggleTick = 0U;

/** @brief Last sampled state of switch SW2 for edge detection. */
static GPIO_PinState SW2_LAST_STATE = GPIO_PIN_RESET;

/*******************************************************************************
 * Functions
 *******************************************************************************/

/**
 * @brief Processes the self-test phase of the run state.
 *
 * @param pRunState Pointer to the run state instance.
 * @param pError Pointer to the error result flag.
 *
 * @return true if the self-test is finished, otherwise false.
 */
static bool RunState_Cyclic_SelfTest(RunState const* pRunState, bool* pError)
{
    bool finished = false;

    assert_param(pError != NULL);

    DrvCrc_Cyclic(pRunState->pCfg->pDrvCrc);

    if (DrvCrc_GetState(pRunState->pCfg->pDrvCrc) == DrvCrcState_Finished)
    {
        if (DrvCrc_IsValid(pRunState->pCfg->pDrvCrc) != false)
        {
            *pError = false;
        }

        else
        {
            *pError = true;
        }

        finished = true;
    }

    return finished;
}

/**
 * @brief Processes the running state.
 *
 * @param pRunState Pointer to the run state instance.
 */
static void RunState_Cyclic_Running(RunState* const pRunState)
{
    assert_param(pRunState != NULL);

    uint32_t tick = HAL_GetTick();
    RunState_Cyclic_Running_HandleLED_D1(tick);

    GPIO_PinState currentPin = HAL_GPIO_ReadPin(SW_2_GPIO_Port, SW_2_Pin);

    if (SW2_LAST_STATE != currentPin && currentPin == GPIO_PIN_RESET)
    {
        Acceleration_RequestConfigure();
    }

    SW2_LAST_STATE = currentPin;

    PowerModes_Cyclic();
    Processing_Cyclic();
    Acceleration_Cyclic();
    Storage_Cyclic();
}

/**
 * @brief Blinks LED D1 at 1 Hz in RUN mode.
 *
 * @param currentTick Current system tick in milliseconds.
 */
static void RunState_Cyclic_Running_HandleLED_D1(uint32_t currentTick)
{
    if ((currentTick - s_lastToggleTick) >= RUN_LED_TOGGLE)
    {
        s_lastToggleTick = currentTick;
        HAL_GPIO_TogglePin(LED_D1_GPIO_Port, LED_D1_Pin);
    }
}
/**
 * @brief Handles the error state.
 *
 * @param pRunState Pointer to the run state instance.
 */
static void RunState_Cyclic_Error(RunState const* pRunState)
{
    (void) pRunState;
    assert_param(0);
}

void RunState_Construct(RunState* const pThis, RunStateConfig const* const pCfg)
{
    assert_param(pThis != NULL);
    assert_param(pThis->constructed == false);
    assert_param(pCfg != NULL);

    pThis->pCfg = pCfg;

    pThis->initialized = false;
    pThis->constructed = true;
}

void RunState_Init(RunState* const pThis)
{
    assert_param(pThis != NULL);
    assert_param(pThis->initialized == false);
    assert_param(pThis->constructed == true);

    /* Initialize dependent modules */
    DrvCrc_Init(pThis->pCfg->pDrvCrc);

    PowerModes_Init(); /* Initialize power mode management (e.g., button state) */
    Processing_Init(); /* Initialize serial output module */

    pThis->data.state = RunState_SelfTest;
    pThis->data.cycleCounter = 0U;

    s_lastToggleTick = HAL_GetTick();
    SW2_LAST_STATE = HAL_GPIO_ReadPin(SW_2_GPIO_Port, SW_2_Pin);

    pThis->initialized = true;
}

void RunState_Cyclic(RunState* const pThis)
{
    assert_param(pThis != NULL);
    assert_param(pThis->constructed == true);
    assert_param(pThis->initialized == true);

    pThis->data.cycleCounter++;

    if (pThis->data.state == RunState_SelfTest)
    {
        bool error = false;

        if (RunState_Cyclic_SelfTest(pThis, &error))
        {
            if (error == true)
            {
                pThis->data.state = RunState_Error;
            }

            else
            {
                /*Self test passed - initialize storage */
                Storage_Init();
                Storage_MainStateInit();

                pThis->data.state = RunState_Running;
            }
        }
    }

    else if (pThis->data.state == RunState_Running)
    {
        RunState_Cyclic_Running(pThis);
    }

    else
    {
        RunState_Cyclic_Error(pThis);
    }
}
