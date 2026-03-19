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
    /** Raw accelerometer X-axis value. */
    int16_t accelXRaw;

    /** Raw accelerometer Y-axis value. */
    int16_t accelYRaw;

    /** Raw accelerometer Z-axis value. */
    int16_t accelZRaw;

    /** Raw gyroscope X-axis value. */
    int16_t gyroXRaw;

    /** Raw gyroscope Y-axis value. */
    int16_t gyroYRaw;

    /** Raw gyroscope Z-axis value. */
    int16_t gyroZRaw;

    /** Indicates whether the data is valid. */
    bool valid;

    /** Counts the number of acquired samples. */
    uint32_t sampleCounter;

} Acceleration_Data;

/*******************************************************************************
* Exported Functions
*******************************************************************************/

/**
 * @brief Initialize logical acceleration module state.
 */
void Acceleration_Init(void);

/**
 * @brief Enable sensor supply from main state initialization context.
 */
void Acceleration_MainStateInit(void);

/**
 * @brief Cyclic processing of the acceleration module.
 */
void Acceleration_Cyclic(void);

/**
 * @brief Request one-time sensor configuration.
 */
void Acceleration_RequestConfigure(void);

/**
 * @brief Disable sensor supply before shutdown.
 */
void Acceleration_PrepareShutdown(void);

/**
 * @brief Check whether sensor configuration is finished.
 *
 * @return true if the sensor is configured, otherwise false.
 */
bool Acceleration_IsConfigured(void);

/**
 * @brief Get latest raw sensor data.
 *
 * @return Pointer to the internal data structure.
 */
Acceleration_Data const* Acceleration_GetData(void);

#endif /* ACCELERATION_H_ */
