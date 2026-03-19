/**
 * @file      processing.c
 * @brief     Data processing and UART output for inertial data.
 */

/*******************************************************************************
* Includes
*******************************************************************************/

#include "processing.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "acceleration.h"
#include "usart.h"

/*******************************************************************************
* Defines
*******************************************************************************/

/** @brief Size of the UART transmit buffer in bytes. */
#define PROCESSING_TX_BUFFER_SIZE           80U

/*******************************************************************************
* Local Types and Typedefs
*******************************************************************************/

/** @brief Internal runtime context of the processing module. */
typedef struct
{
    /** Indicates whether the module was initialized. */
    bool initialized;
    /** Indicates whether a UART DMA transmission is active. */
    bool txBusy;
    /** Stores the last transmitted sample counter. */
    uint32_t lastSampleCounter;
    /** UART transmit buffer. */
    uint8_t txBuffer[PROCESSING_TX_BUFFER_SIZE];
} Processing_Context;

/*******************************************************************************
* Static Function Prototypes
*******************************************************************************/

static int32_t Processing_ConvertGyroRawToDpsInt(int16_t rawValue);
static int32_t Processing_ConvertAccelRawToTenthsMs2(int16_t rawValue);

static uint32_t Processing_GetAbsU32(int32_t value);
static void Processing_AppendChar(uint8_t* const pBuffer,
                                  uint16_t* const pIndex,
                                  char character);
static void Processing_AppendText(uint8_t* const pBuffer,
                                  uint16_t* const pIndex,
                                  char const* pText);
static void Processing_AppendSignedInt3(uint8_t* const pBuffer,
                                        uint16_t* const pIndex,
                                        int32_t value);
static void Processing_AppendSignedFixed1(uint8_t* const pBuffer,
        uint16_t* const pIndex,
        int32_t tenthsValue);

/*******************************************************************************
* Static Variables
*******************************************************************************/

/** @brief Global context of the processing module. */
static Processing_Context s_processing;

/*******************************************************************************
* Functions
*******************************************************************************/

/**
 * @brief Converts a raw gyroscope value to a rounded integer in deg/s.
 *
 * @param rawValue Raw gyroscope sensor value.
 *
 * @return Converted gyroscope value in deg/s.
 */
static int32_t Processing_ConvertGyroRawToDpsInt(int16_t rawValue)
{
    int32_t scaledValue;
    int32_t result;

    /* Gyro sensitivity for +-1000 deg/s: 32.8 LSB/(deg/s)
     * Rounded integer result:
     * deg/s = raw / 32.8 = raw * 10 / 328
     */
    scaledValue = ((int32_t)rawValue) * 10;

    if (scaledValue >= 0)
    {
        result = (scaledValue + 164) / 328;
    }

    else
    {
        result = (scaledValue - 164) / 328;
    }

    if (result > 999)
    {
        result = 999;
    }

    else if (result < -999)
    {
        result = -999;
    }

    else
    {
        /* Value already within valid range */
    }

    return result;
}

/**
 * @brief Converts a raw accelerometer value to tenths of m/s^2.
 *
 * @param rawValue Raw accelerometer sensor value.
 *
 * @return Converted acceleration value in 0.1 m/s^2.
 */
static int32_t Processing_ConvertAccelRawToTenthsMs2(int16_t rawValue)
{
    int32_t scaledValue;
    int32_t result;

    /* Accel sensitivity for +-16 g: 2048 LSB/g
     * 1 g = 9.80665 m/s^2
     * For one decimal place:
     * value_0p1 = raw * 9.80665 * 10 / 2048
     *           ~ raw * 98.0665 / 2048
     * Integer approximation with rounding:
     *           ~ raw * 98 / 2048
     */
    scaledValue = ((int32_t)rawValue) * 98;

    if (scaledValue >= 0)
    {
        result = (scaledValue + 1024) / 2048;
    }

    else
    {
        result = (scaledValue - 1024) / 2048;
    }

    if (result > 999)
    {
        result = 999;
    }

    else if (result < -999)
    {
        result = -999;
    }

    else
    {
        /* Value already within valid range */
    }

    return result;
}

/**
 * @brief Returns the absolute value of a signed 32-bit integer as unsigned.
 *
 * @param value Signed input value.
 *
 * @return Absolute value as uint32_t.
 */
static uint32_t Processing_GetAbsU32(int32_t value)
{
    if (value < 0)
    {
        return (uint32_t)(-value);
    }

    return (uint32_t)value;
}

/**
 * @brief Appends one character to the transmit buffer.
 *
 * @param pBuffer Pointer to the target buffer.
 * @param pIndex Pointer to the current write index.
 * @param character Character to append.
 */
static void Processing_AppendChar(uint8_t* const pBuffer,
                                  uint16_t* const pIndex,
                                  char character)
{
    if (*pIndex >= (PROCESSING_TX_BUFFER_SIZE - 1U))
    {
        Error_Handler();
    }

    pBuffer[*pIndex] = (uint8_t)character;
    (*pIndex)++;
}

/**
 * @brief Appends a zero-terminated text string to the transmit buffer.
 *
 * @param pBuffer Pointer to the target buffer.
 * @param pIndex Pointer to the current write index.
 * @param pText Pointer to the text string.
 */
static void Processing_AppendText(uint8_t* const pBuffer,
                                  uint16_t* const pIndex,
                                  char const* pText)
{
    uint16_t i = 0U;

    while (pText[i] != '\0')
    {
        Processing_AppendChar(pBuffer, pIndex, pText[i]);
        i++;
    }
}

/**
 * @brief Appends a signed integer with sign and three digits.
 *
 * @param pBuffer Pointer to the target buffer.
 * @param pIndex Pointer to the current write index.
 * @param value Signed value to append.
 */
static void Processing_AppendSignedInt3(uint8_t* const pBuffer,
                                        uint16_t* const pIndex,
                                        int32_t value)
{
    uint32_t magnitude;

    if (value >= 0)
    {
        Processing_AppendChar(pBuffer, pIndex, '+');
        magnitude = (uint32_t)value;
    }

    else
    {
        Processing_AppendChar(pBuffer, pIndex, '-');
        magnitude = Processing_GetAbsU32(value);
    }

    if (magnitude > 999U)
    {
        magnitude = 999U;
    }

    Processing_AppendChar(pBuffer, pIndex, (char)('0' + ((magnitude / 100U) % 10U)));
    Processing_AppendChar(pBuffer, pIndex, (char)('0' + ((magnitude / 10U) % 10U)));
    Processing_AppendChar(pBuffer, pIndex, (char)('0' + (magnitude % 10U)));
}

/**
 * @brief Appends a signed fixed-point value with one decimal place.
 *
 * @param pBuffer Pointer to the target buffer.
 * @param pIndex Pointer to the current write index.
 * @param tenthsValue Signed value in tenths.
 */
static void Processing_AppendSignedFixed1(uint8_t* const pBuffer,
        uint16_t* const pIndex,
        int32_t tenthsValue)
{
    uint32_t magnitude;
    uint32_t integerPart;
    uint32_t fractionPart;

    if (tenthsValue >= 0)
    {
        Processing_AppendChar(pBuffer, pIndex, '+');
        magnitude = (uint32_t)tenthsValue;
    }

    else
    {
        Processing_AppendChar(pBuffer, pIndex, '-');
        magnitude = Processing_GetAbsU32(tenthsValue);
    }

    if (magnitude > 999U)
    {
        magnitude = 999U;
    }

    integerPart = magnitude / 10U;
    fractionPart = magnitude % 10U;

    Processing_AppendChar(pBuffer, pIndex, (char)('0' + ((integerPart / 10U) % 10U)));
    Processing_AppendChar(pBuffer, pIndex, (char)('0' + (integerPart % 10U)));
    Processing_AppendChar(pBuffer, pIndex, '.');
    Processing_AppendChar(pBuffer, pIndex, (char)('0' + fractionPart));
}

/** @brief Initializes the processing module. */
void Processing_Init(void)
{
    (void)memset(&s_processing, 0, sizeof(s_processing));
    s_processing.initialized = true;
}

/** @brief Cyclic processing of sensor conversion and UART output. */
void Processing_Cyclic(void)
{
    Acceleration_Data const* pAccelerationData;
    int32_t gyroX_dps;
    int32_t gyroY_dps;
    int32_t gyroZ_dps;
    int32_t accelX_tenthsMs2;
    int32_t accelY_tenthsMs2;
    int32_t accelZ_tenthsMs2;
    uint16_t index = 0U;

    if (s_processing.initialized == false)
    {
        return;
    }

    if (s_processing.txBusy == true)
    {
        return;
    }

    if (Acceleration_IsConfigured() == false)
    {
        return;
    }

    pAccelerationData = Acceleration_GetData();

    if (pAccelerationData->valid == false)
    {
        return;
    }

    if (pAccelerationData->sampleCounter == s_processing.lastSampleCounter)
    {
        return;
    }

    gyroX_dps = Processing_ConvertGyroRawToDpsInt(pAccelerationData->gyroXRaw);
    gyroY_dps = Processing_ConvertGyroRawToDpsInt(pAccelerationData->gyroYRaw);
    gyroZ_dps = Processing_ConvertGyroRawToDpsInt(pAccelerationData->gyroZRaw);

    accelX_tenthsMs2 = Processing_ConvertAccelRawToTenthsMs2(pAccelerationData->accelXRaw);
    accelY_tenthsMs2 = Processing_ConvertAccelRawToTenthsMs2(pAccelerationData->accelYRaw);
    accelZ_tenthsMs2 = Processing_ConvertAccelRawToTenthsMs2(pAccelerationData->accelZRaw);

    Processing_AppendText(s_processing.txBuffer, &index, "Drehrate: ");
    Processing_AppendSignedInt3(s_processing.txBuffer, &index, gyroX_dps);
    Processing_AppendChar(s_processing.txBuffer, &index, ' ');
    Processing_AppendSignedInt3(s_processing.txBuffer, &index, gyroY_dps);
    Processing_AppendChar(s_processing.txBuffer, &index, ' ');
    Processing_AppendSignedInt3(s_processing.txBuffer, &index, gyroZ_dps);

    Processing_AppendText(s_processing.txBuffer, &index, "; Linear: ");
    Processing_AppendSignedFixed1(s_processing.txBuffer, &index, accelX_tenthsMs2);
    Processing_AppendText(s_processing.txBuffer, &index, ", ");
    Processing_AppendSignedFixed1(s_processing.txBuffer, &index, accelY_tenthsMs2);
    Processing_AppendText(s_processing.txBuffer, &index, ", ");
    Processing_AppendSignedFixed1(s_processing.txBuffer, &index, accelZ_tenthsMs2);
    Processing_AppendText(s_processing.txBuffer, &index, "\r\n");

    if (HAL_UART_Transmit_DMA(&hlpuart1, s_processing.txBuffer, index) != HAL_OK)
    {
        Error_Handler();
    }

    s_processing.txBusy = true;
    s_processing.lastSampleCounter = pAccelerationData->sampleCounter;
}

/**
 * @brief HAL callback for completed UART DMA transmissions.
 *
 * @param huart Pointer to the UART handle.
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    if ((huart != NULL) && (huart->Instance == LPUART1))
    {
        s_processing.txBusy = false;
    }
}
