/******************************************************************************
 * @file    powermodes.h
 * @brief   Power mode handling
 ******************************************************************************/

#ifndef POWERMODES_H_
#define POWERMODES_H_

/***** Includes **************************************************************/
#include <stdbool.h>

/***** Public Functions ******************************************************/

/*!
 * @brief Initialize power mode module
 */
void PowerModes_Init(void);

/*!
 * @brief Process cyclic power mode logic
 */
void PowerModes_Cyclic(void);

/*!
 * @brief Process button press or release event
 *
 * @param pressed true if button is pressed, false if button is released
 */
void PowerModes_ButtonEvent(bool pressed);

#endif /* POWERMODES_H_ */
