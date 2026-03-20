/**
 * @file      acceleration.h
 * @brief     Header file for acceleration sensor handling.
 */

#ifndef ACCELERATION_H_
#define ACCELERATION_H_

/*******************************************************************************
 * Includes
 *******************************************************************************/
#include <stdbool.h>
#include <stdint.h>

/*******************************************************************************
 * Exported Types and Typedefs
 *******************************************************************************/

/**
 * @brief Raw data container for accelerometer and gyroscope values.
 */
typedef struct
{
    /** @brief Raw accelerometer X-axis value. */
    int16_t accelXRaw;

    /** @brief Raw accelerometer Y-axis value. */
    int16_t accelYRaw;

    /** @brief Raw accelerometer Z-axis value. */
    int16_t accelZRaw;

    /** @brief Raw gyroscope X-axis value. */
    int16_t gyroXRaw;

    /** @brief Raw gyroscope Y-axis value. */
    int16_t gyroYRaw;

    /** @brief Raw gyroscope Z-axis value. */
    int16_t gyroZRaw;

    /** @brief Indicates whether the data is valid. */
    bool valid;

    /** @brief Counts the number of acquired samples. */
    uint32_t sampleCounter;
} Acceleration_Data;

/*******************************************************************************
 * Exported Functions
 *******************************************************************************/

/**
 * @brief Initializes the logical acceleration module state.
 */
void Acceleration_Init(void);

/**
 * @brief Cyclic processing of the acceleration module including button handling and DMA state machine.
 */
void Acceleration_Cyclic(void);

/**
 * @brief Disables sensor supply before shutdown and resets internal state.
 */
void Acceleration_Shutdown(void);

/**
 * @brief Provides access to the latest measurement data.
 * @return Pointer to the internal measurement structure.
 */
Acceleration_Data const* Acceleration_GetData(void);

#endif /* ACCELERATION_H_ */
