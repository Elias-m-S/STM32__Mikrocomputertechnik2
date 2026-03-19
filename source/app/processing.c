/**
 * @file      processing.c
 * @brief     Sensor data formatting and serial transmission module.
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

/** @brief Maximum size of formatted output buffer in bytes. */
#define OUTPUT_BUFFER_CAPACITY              80U

/** @brief Maximum representable acceleration magnitude (99.9 m/s^2). */
#define ACCEL_MAX_MAGNITUDE_TENTHS          999U

/*******************************************************************************
* Local Types and Typedefs
*******************************************************************************/

/** @brief Serial transmission engine context. */
typedef struct
{
    /** Module operational status flag. */
    bool ready;
    /** Indicates active serial DMA transmission. */
    bool transmission_active;
    /** Last processed measurement identifier. */
    uint32_t prior_measurement_id;
    /** Formatted output byte sequence. */
    uint8_t output_sequence[OUTPUT_BUFFER_CAPACITY];
} SerialOutput_Context;

/*******************************************************************************
* Static Function Prototypes
*******************************************************************************/

static int32_t SerialOutput_GyroScaling(int16_t sensor_raw);
static int32_t SerialOutput_AccelScaling(int16_t sensor_raw);

static uint32_t SerialOutput_Magnitude(int32_t signed_value);
static void SerialOutput_WriteByte(uint8_t* const target,
                                   uint16_t* const offset,
                                   uint8_t byte_value);
static void SerialOutput_WriteString(uint8_t* const target,
                                     uint16_t* const offset,
                                     char const* source);
static void SerialOutput_WriteSignedInteger(uint8_t* const target,
        uint16_t* const offset,
        int32_t number);
static void SerialOutput_WriteSignedDecimal(uint8_t* const target,
        uint16_t* const offset,
        int32_t decimal_value);

/*******************************************************************************
* Static Variables
*******************************************************************************/

/** @brief Module runtime state. */
static SerialOutput_Context s_serial_output;

/*******************************************************************************
* Functions
*******************************************************************************/

/**
 * @brief Transforms raw gyroscope reading to integer degrees per second.
 *
 * @param sensor_raw Raw gyroscope measurement.
 *
 * @return Scaled gyroscope value in deg/s with rounding.
 */
static int32_t SerialOutput_GyroScaling(int16_t sensor_raw)
{
    int32_t scaled;
    int32_t converted;

    /* Gyroscope sensitivity: +-1000 deg/s → 32.8 LSB/(deg/s)
     * Conversion: deg/s = raw / 32.8 = raw * 10 / 328
     */
    scaled = ((int32_t)sensor_raw) * 10;

    if (scaled >= 0)
    {
        converted = (scaled + 164) / 328;
    }

    else
    {
        converted = (scaled - 164) / 328;
    }

    if (converted > 999)
    {
        converted = 999;
    }

    else if (converted < -999)
    {
        converted = -999;
    }

    return converted;
}

/**
 * @brief Transforms raw accelerometer reading to tenths of m/s^2.
 *
 * @param sensor_raw Raw accelerometer measurement.
 *
 * @return Scaled acceleration value in 0.1 m/s^2 with clamping.
 */
static int32_t SerialOutput_AccelScaling(int16_t sensor_raw)
{
    int32_t scaled;
    int32_t converted;

    /* Accelerometer sensitivity: +-16 g → 2048 LSB/g
     * 1 g = 9.80665 m/s^2
     * Conversion: value_0p1 = raw * 98 / 2048 (integer approximation)
     */
    scaled = ((int32_t)sensor_raw) * 98;

    if (scaled >= 0)
    {
        converted = (scaled + 1024) / 2048;
    }

    else
    {
        converted = (scaled - 1024) / 2048;
    }

    if (converted > 999)
    {
        converted = 999;
    }

    else if (converted < -999)
    {
        converted = -999;
    }

    return converted;
}

/**
 * @brief Computes absolute value of signed integer as unsigned type.
 *
 * @param signed_value Input value.
 *
 * @return Absolute value.
 */
static uint32_t SerialOutput_Magnitude(int32_t signed_value)
{
    if (signed_value < 0)
    {
        return (uint32_t)(-signed_value);
    }

    return (uint32_t)signed_value;
}

/**
 * @brief Writes single byte to output buffer at specified offset.
 *
 * @param target Output buffer pointer.
 * @param offset Current write position pointer.
 * @param byte_value Byte to write.
 */
static void SerialOutput_WriteByte(uint8_t* const target,
                                   uint16_t* const offset,
                                   uint8_t byte_value)
{
    if (*offset >= (OUTPUT_BUFFER_CAPACITY - 1U))
    {
        Error_Handler();
    }

    target[*offset] = byte_value;
    (*offset)++;
}

/**
 * @brief Writes null-terminated string to output buffer.
 *
 * @param target Output buffer pointer.
 * @param offset Current write position pointer.
 * @param source Source string pointer.
 */
static void SerialOutput_WriteString(uint8_t* const target,
                                     uint16_t* const offset,
                                     char const* source)
{
    uint16_t idx = 0U;

    while (source[idx] != '\0')
    {
        SerialOutput_WriteByte(target, offset, (uint8_t)source[idx]);
        idx++;
    }
}

/**
 * @brief Writes signed integer with sign and three digits.
 *
 * @param target Output buffer pointer.
 * @param offset Current write position pointer.
 * @param number Value to format.
 */
static void SerialOutput_WriteSignedInteger(uint8_t* const target,
        uint16_t* const offset,
        int32_t number)
{
    uint32_t magnitude;

    if (number >= 0)
    {
        SerialOutput_WriteByte(target, offset, (uint8_t)'+');
        magnitude = (uint32_t)number;
    }

    else
    {
        SerialOutput_WriteByte(target, offset, (uint8_t)'-');
        magnitude = SerialOutput_Magnitude(number);
    }

    if (magnitude > 999U)
    {
        magnitude = 999U;
    }

    SerialOutput_WriteByte(target, offset, (uint8_t)('0' + ((magnitude / 100U) % 10U)));
    SerialOutput_WriteByte(target, offset, (uint8_t)('0' + ((magnitude / 10U) % 10U)));
    SerialOutput_WriteByte(target, offset, (uint8_t)('0' + (magnitude % 10U)));
}

/**
 * @brief Writes signed decimal with one fractional digit.
 *
 * @param target Output buffer pointer.
 * @param offset Current write position pointer.
 * @param decimal_value Value in tenths to format.
 */
static void SerialOutput_WriteSignedDecimal(uint8_t* const target,
        uint16_t* const offset,
        int32_t decimal_value)
{
    uint32_t magnitude;
    uint32_t whole_part;
    uint32_t fractional_part;

    if (decimal_value >= 0)
    {
        SerialOutput_WriteByte(target, offset, (uint8_t)'+');
        magnitude = (uint32_t)decimal_value;
    }

    else
    {
        SerialOutput_WriteByte(target, offset, (uint8_t)'-');
        magnitude = SerialOutput_Magnitude(decimal_value);
    }

    if (magnitude > ACCEL_MAX_MAGNITUDE_TENTHS)
    {
        magnitude = ACCEL_MAX_MAGNITUDE_TENTHS;
    }

    whole_part = magnitude / 10U;
    fractional_part = magnitude % 10U;

    SerialOutput_WriteByte(target, offset, (uint8_t)('0' + ((whole_part / 10U) % 10U)));
    SerialOutput_WriteByte(target, offset, (uint8_t)('0' + (whole_part % 10U)));
    SerialOutput_WriteByte(target, offset, (uint8_t)'.');
    SerialOutput_WriteByte(target, offset, (uint8_t)('0' + fractional_part));
}

/** @brief Initializes serial output module. */
void Processing_Init(void)
{
    (void)memset(&s_serial_output, 0, sizeof(s_serial_output));
    s_serial_output.ready = true;
}

/** @brief Cyclic processing: reads sensor data, formats, and transmits via DMA. */
void Processing_Cyclic(void)
{
    Acceleration_Data const* sensor_dataset;
    int32_t angular_rate_x;
    int32_t angular_rate_y;
    int32_t angular_rate_z;
    int32_t linear_accel_x;
    int32_t linear_accel_y;
    int32_t linear_accel_z;
    uint16_t buffer_position = 0U;

    if (s_serial_output.ready == false)
    {
        return;
    }

    if (s_serial_output.transmission_active == true)
    {
        return;
    }

    if (Acceleration_IsConfigured() == false)
    {
        return;
    }

    sensor_dataset = Acceleration_GetData();

    if (sensor_dataset->valid == false)
    {
        return;
    }

    if (sensor_dataset->sampleCounter == s_serial_output.prior_measurement_id)
    {
        return;
    }

    angular_rate_x = SerialOutput_GyroScaling(sensor_dataset->gyroXRaw);
    angular_rate_y = SerialOutput_GyroScaling(sensor_dataset->gyroYRaw);
    angular_rate_z = SerialOutput_GyroScaling(sensor_dataset->gyroZRaw);

    linear_accel_x = SerialOutput_AccelScaling(sensor_dataset->accelXRaw);
    linear_accel_y = SerialOutput_AccelScaling(sensor_dataset->accelYRaw);
    linear_accel_z = SerialOutput_AccelScaling(sensor_dataset->accelZRaw);

    SerialOutput_WriteString(s_serial_output.output_sequence, &buffer_position, "Drehrate: ");
    SerialOutput_WriteSignedInteger(s_serial_output.output_sequence, &buffer_position, angular_rate_x);
    SerialOutput_WriteByte(s_serial_output.output_sequence, &buffer_position, (uint8_t)' ');
    SerialOutput_WriteSignedInteger(s_serial_output.output_sequence, &buffer_position, angular_rate_y);
    SerialOutput_WriteByte(s_serial_output.output_sequence, &buffer_position, (uint8_t)' ');
    SerialOutput_WriteSignedInteger(s_serial_output.output_sequence, &buffer_position, angular_rate_z);

    SerialOutput_WriteString(s_serial_output.output_sequence, &buffer_position, "; Linear: ");
    SerialOutput_WriteSignedDecimal(s_serial_output.output_sequence, &buffer_position, linear_accel_x);
    SerialOutput_WriteString(s_serial_output.output_sequence, &buffer_position, ", ");
    SerialOutput_WriteSignedDecimal(s_serial_output.output_sequence, &buffer_position, linear_accel_y);
    SerialOutput_WriteString(s_serial_output.output_sequence, &buffer_position, ", ");
    SerialOutput_WriteSignedDecimal(s_serial_output.output_sequence, &buffer_position, linear_accel_z);
    SerialOutput_WriteString(s_serial_output.output_sequence, &buffer_position, "\r\n");

    if (HAL_UART_Transmit_DMA(&hlpuart1, s_serial_output.output_sequence, buffer_position) != HAL_OK)
    {
        Error_Handler();
    }

    s_serial_output.transmission_active = true;
    s_serial_output.prior_measurement_id = sensor_dataset->sampleCounter;
}

/**
 * @brief HAL callback: UART DMA transmission completion handler.
 *
 * @param huart UART interface handle.
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    if ((huart != NULL) && (huart->Instance == LPUART1))
    {
        s_serial_output.transmission_active = false;
    }
}
