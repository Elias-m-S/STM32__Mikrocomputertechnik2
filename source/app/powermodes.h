/******************************************************************************
 * @file    powermodes.h
 * @brief   Power mode handling
 ******************************************************************************/

#ifndef POWERMODES_H_
#define POWERMODES_H_

#include <stdbool.h>

void PowerModes_Init(void);

void PowerModes_ButtonEvent(bool pressed);

#endif /* POWERMODES_H_ */
