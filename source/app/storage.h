/**
 * @file      storage.h
 *
 * @brief     Header file for persistent storage handling
 */

#ifndef STORAGE_H_
#define STORAGE_H_

/*******************************************************************************
 * Includes
 *******************************************************************************/

#include <stdbool.h>
#include <stdint.h>

/*******************************************************************************
 * Exported Functions
 *******************************************************************************/

/*!
 * @brief Initialize logical storage module state
 */
void Storage_Init(void);

/*!
 * @brief Initialize EEPROM emulation and load persisted values
 */
void Storage_MainStateInit(void);

/*!
 * @brief Cyclic processing of runtime accumulation and persistence
 */
void Storage_Cyclic(void);

/*!
 * @brief Persist pending runtime before shutdown
 */
void Storage_PrepareShutdown(void);

#endif /* STORAGE_H_ */
