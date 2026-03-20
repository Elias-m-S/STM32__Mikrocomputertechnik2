/**
 * @file      processing.c
 * @brief     Sensor data formatting and serial transmission module.
 */

#include "processing.h"

#include <stdbool.h> /*Standard libraries dont seem forbidden */
#include <stdint.h>
#include <string.h>

#include "acceleration.h"
#include "usart.h"

/** @brief Max buffer size. */
#define OUTPUT_BUFFER_CAPACITY              80U

/** @brief Max accel value. */
#define ACCEL_MAX_MAGNITUDE_TENTHS          999U

/** @brief Serial context. */
typedef struct
{
    /** @brief Init flag. */
    bool ready;
    /** @brief TX active flag. */
    bool transmission_active;
    /** @brief Last sample ID. */
    uint32_t prior_measurement_id;
    /** @brief TX buffer. */
    uint8_t output_sequence[OUTPUT_BUFFER_CAPACITY];
} SerialOutput_Context;

/** @brief Global context. */
static SerialOutput_Context s_serial_output;

/**
 * @brief Scales gyro data.
 * @param sensor_raw Raw gyro.
 * @return Scaled gyro.
 */
static int32_t SerialOutput_GyroScaling(int16_t sensor_raw)
{
    int32_t scaled = ((int32_t) sensor_raw) * 10;
    int32_t converted = (scaled >= 0) ? ((scaled + 164) / 328) : ((scaled - 164) / 328);

    if (converted > 999)
    {
        return 999;
    }

    if (converted < -999)
    {
        return -999;
    }

    return converted;
}

/**
 * @brief Scales accel data.
 * @param sensor_raw Raw accel.
 * @return Scaled accel.
 */
static int32_t SerialOutput_AccelScaling(int16_t sensor_raw)
{
    int32_t scaled = ((int32_t) sensor_raw) * 98;
    int32_t converted = (scaled >= 0) ? ((scaled + 1024) / 2048) : ((scaled - 1024) / 2048);

    if (converted > 999)
    {
        return 999;
    }

    if (converted < -999)
    {
        return -999;
    }

    return converted;
}

/**
 * @brief Gets absolute value.
 * @param signed_value Input.
 * @return Magnitude.
 */
static uint32_t SerialOutput_Magnitude(int32_t signed_value)
{
    return (signed_value < 0) ? (uint32_t)(-signed_value) : (uint32_t)signed_value;
}

/**
 * @brief Writes byte.
 * @param target Buffer.
 * @param offset Offset.
 * @param byte_value Data.
 */
static void SerialOutput_WriteByte(uint8_t* const target, uint16_t* const offset, uint8_t byte_value)
{
    if (*offset >= (OUTPUT_BUFFER_CAPACITY - 1U))
    {
        Error_Handler();
    }

    target[*offset] = byte_value;
    (*offset)++;
}

/**
 * @brief Writes string.
 * @param target Buffer.
 * @param offset Offset.
 * @param source String.
 */
static void SerialOutput_WriteString(uint8_t* const target, uint16_t* const offset, char const* source)
{
    uint16_t idx = 0U;

    while (source[idx] != '\0')
    {
        SerialOutput_WriteByte(target, offset, (uint8_t) source[idx]);
        idx++;
    }
}

/**
 * @brief Writes int.
 * @param target Buffer.
 * @param offset Offset.
 * @param number Data.
 */
static void SerialOutput_WriteSignedInteger(uint8_t* const target, uint16_t* const offset, int32_t number)
{
    uint32_t magnitude;

    if (number >= 0)
    {
        SerialOutput_WriteByte(target, offset, (uint8_t) '+');
        magnitude = (uint32_t) number;
    }

    else
    {
        SerialOutput_WriteByte(target, offset, (uint8_t) '-');
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
 * @brief Writes decimal.
 * @param target Buffer.
 * @param offset Offset.
 * @param decimal_value Data.
 */
static void SerialOutput_WriteSignedDecimal(uint8_t* const target, uint16_t* const offset, int32_t decimal_value)
{
    uint32_t magnitude;

    if (decimal_value >= 0)
    {
        SerialOutput_WriteByte(target, offset, (uint8_t) '+');
        magnitude = (uint32_t) decimal_value;
    }

    else
    {
        SerialOutput_WriteByte(target, offset, (uint8_t) '-');
        magnitude = SerialOutput_Magnitude(decimal_value);
    }

    if (magnitude > ACCEL_MAX_MAGNITUDE_TENTHS)
    {
        magnitude = ACCEL_MAX_MAGNITUDE_TENTHS;
    }

    uint32_t whole_part = magnitude / 10U;
    uint32_t fractional_part = magnitude % 10U;

    SerialOutput_WriteByte(target, offset, (uint8_t)('0' + ((whole_part / 10U) % 10U)));
    SerialOutput_WriteByte(target, offset, (uint8_t)('0' + (whole_part % 10U)));
    SerialOutput_WriteByte(target, offset, (uint8_t) '.');
    SerialOutput_WriteByte(target, offset, (uint8_t)('0' + fractional_part));
}

/**
 * @brief Initialize processing module.
 */
void Processing_Init(void)
{
    (void) memset(&s_serial_output, 0, sizeof(s_serial_output));
    s_serial_output.ready = true;
}

/**
 * @brief Cyclic processing of sensor data.
 */
void Processing_Cyclic(void)
{
    if (!s_serial_output.ready || s_serial_output.transmission_active)
    {
        return;
    }

    Acceleration_Data const* sensor_dataset = Acceleration_GetData();

    if (sensor_dataset->valid == false || sensor_dataset->sampleCounter == s_serial_output.prior_measurement_id)
    {
        return;
    }

    int32_t angular_rate_x = SerialOutput_GyroScaling(sensor_dataset->gyroXRaw);
    int32_t angular_rate_y = SerialOutput_GyroScaling(sensor_dataset->gyroYRaw);
    int32_t angular_rate_z = SerialOutput_GyroScaling(sensor_dataset->gyroZRaw);

    int32_t linear_accel_x = SerialOutput_AccelScaling(sensor_dataset->accelXRaw);
    int32_t linear_accel_y = SerialOutput_AccelScaling(sensor_dataset->accelYRaw);
    int32_t linear_accel_z = SerialOutput_AccelScaling(sensor_dataset->accelZRaw);

    uint16_t buffer_position = 0U;

    SerialOutput_WriteString(s_serial_output.output_sequence, &buffer_position, "Drehrate: ");
    SerialOutput_WriteSignedInteger(s_serial_output.output_sequence, &buffer_position, angular_rate_x);
    SerialOutput_WriteByte(s_serial_output.output_sequence, &buffer_position, (uint8_t) ' ');
    SerialOutput_WriteSignedInteger(s_serial_output.output_sequence, &buffer_position, angular_rate_y);
    SerialOutput_WriteByte(s_serial_output.output_sequence, &buffer_position, (uint8_t) ' ');
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
 * @brief UART TX complete.
 * @param huart UART handle.
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
    if ((huart != NULL) && (huart->Instance == LPUART1))
    {
        s_serial_output.transmission_active = false;
    }
}
