/**
 * @file      acceleration.c
 * @brief     IMU sensor handling for GY-521.
 */

#include "acceleration.h"
#include <string.h>
#include "main.h"
#include "gpio.h"
#include "i2c.h"

/*******************************************************************************
 * Defines
 *******************************************************************************/

/** @brief I2C slave address. */
#define IMU_I2C_ADDR                     (0x68U << 1U)

/** @brief Accel start register. */
#define IMU_REG_ACCEL_DATA_START         0x3BU

/** @brief Gyro start register. */
#define IMU_REG_GYRO_DATA_START          0x43U

/** @brief DLPF config register. */
#define IMU_REG_DLPF_CONFIG              0x1AU

/** @brief Gyro range register. */
#define IMU_REG_GYRO_RANGE_CONFIG        0x1BU

/** @brief Accel range register. */
#define IMU_REG_ACCEL_RANGE_CONFIG       0x1CU

/** @brief Power control register. */
#define IMU_REG_PWR_CONTROL              0x6BU

/** @brief VCC GPIO port. */
#define IMU_SUPPLY_PORT                  ACCELEROMETER_VCC_GPIO_Port

/** @brief VCC GPIO pin. */
#define IMU_SUPPLY_PIN                   ACCELEROMETER_VCC_Pin

/** @brief Sample interval ms. */
#define IMU_SAMPLE_INTERVAL_MS           1000U

/** @brief Config register count. */
#define IMU_CONFIG_REGISTER_COUNT        4U

/*******************************************************************************
 * Local Types and Typedefs
 *******************************************************************************/

/** @brief IMU module states. */
typedef enum
{
    ImuState_Powered_Off = 0, /**< @brief Power off. */
    ImuState_Ready, /**< @brief Ready. */
    ImuState_Configuring, /**< @brief Configuring. */
    ImuState_Fetching_Accel, /**< @brief Fetching accel. */
    ImuState_Fetching_Gyro /**< @brief Fetching gyro. */
} ImuState_t;

/** @brief IMU module context. */
typedef struct
{
    ImuState_t state; /**< @brief Current state. */
    bool power_active; /**< @brief Power flag. */
    bool config_done; /**< @brief Config flag. */
    volatile bool write_finished; /**< @brief TX done flag. */
    volatile bool read_finished; /**< @brief RX done flag. */
    uint8_t register_index; /**< @brief Config index. */
    uint8_t register_value; /**< @brief Config value. */
    uint8_t accel_raw_bytes[6]; /**< @brief Accel buffer. */
    uint8_t gyro_raw_bytes[6]; /**< @brief Gyro buffer. */
    uint32_t measurement_timestamp; /**< @brief Last sample tick. */
    Acceleration_Data measurement; /**< @brief Sensor data. */
} ImuContext_t;

/*******************************************************************************
 * Static Variables
 *******************************************************************************/

/** @brief Global IMU context. */
static ImuContext_t s_imu_context;

/** @brief Last SW2 state. */
static GPIO_PinState s_sw2_last_state = GPIO_PIN_RESET;

/*******************************************************************************
 * Static Function Prototypes
 *******************************************************************************/

static bool Imu_InitiateRegisterWrite(uint8_t reg_index);
static bool Imu_InitiateAccelRetrieval(void);
static bool Imu_InitiateGyroRetrieval(void);
static void Imu_TransformAccelBytes(void);
static void Imu_TransformGyroBytes(void);

/*******************************************************************************
 * Functions
 *******************************************************************************/

/**
 * @brief Init IMU module.
 */
void Acceleration_Init(void)
{
    memset(&s_imu_context, 0, sizeof(ImuContext_t));
    s_sw2_last_state = HAL_GPIO_ReadPin(SW_2_GPIO_Port, SW_2_Pin);

    HAL_GPIO_WritePin(IMU_SUPPLY_PORT, IMU_SUPPLY_PIN, GPIO_PIN_SET);
    s_imu_context.power_active = true;
    s_imu_context.state = ImuState_Ready;
    s_imu_context.measurement_timestamp = HAL_GetTick();
}

/**
 * @brief Cyclic IMU task.
 */
void Acceleration_Cyclic(void)
{
    if (!s_imu_context.power_active)
    {
        return;
    }

    uint32_t current_timestamp = HAL_GetTick();

    GPIO_PinState current_sw2 = HAL_GPIO_ReadPin(SW_2_GPIO_Port, SW_2_Pin);

    if ((s_sw2_last_state != current_sw2) && (current_sw2 == GPIO_PIN_RESET))
    {
        if (s_imu_context.state == ImuState_Ready
                && !s_imu_context.config_done)
        {
            s_imu_context.state = ImuState_Configuring;
            s_imu_context.register_index = 0U;
            Imu_InitiateRegisterWrite(s_imu_context.register_index);
        }
    }

    s_sw2_last_state = current_sw2;

    switch (s_imu_context.state)
    {
        case ImuState_Configuring:
            if (s_imu_context.write_finished)
            {
                s_imu_context.write_finished = false;
                s_imu_context.register_index++;

                if (s_imu_context.register_index < IMU_CONFIG_REGISTER_COUNT)
                {
                    Imu_InitiateRegisterWrite(s_imu_context.register_index);
                }

                else
                {
                    s_imu_context.config_done = true;
                    s_imu_context.state = ImuState_Fetching_Accel;
                    HAL_GPIO_WritePin(LED_D0_GPIO_Port, LED_D0_Pin, GPIO_PIN_SET);
                }
            }

            break;

        case ImuState_Fetching_Accel:
            if (!s_imu_context.read_finished
                    && (current_timestamp - s_imu_context.measurement_timestamp
                        >= IMU_SAMPLE_INTERVAL_MS))
            {
                s_imu_context.measurement_timestamp = current_timestamp;
                Imu_InitiateAccelRetrieval();
            }

            else if (s_imu_context.read_finished)
            {
                s_imu_context.read_finished = false;
                Imu_TransformAccelBytes();
                s_imu_context.state = ImuState_Fetching_Gyro;
                Imu_InitiateGyroRetrieval();
            }

            break;

        case ImuState_Fetching_Gyro:
            if (s_imu_context.read_finished)
            {
                s_imu_context.read_finished = false;
                Imu_TransformGyroBytes();
                s_imu_context.measurement.valid = true;
                s_imu_context.measurement.sampleCounter++;
                s_imu_context.state = ImuState_Fetching_Accel;
            }

            break;

        default:
            break;
    }
}

/**
 * @brief Shutdown IMU.
 */
void Acceleration_Shutdown(void)
{
    HAL_GPIO_WritePin(IMU_SUPPLY_PORT, IMU_SUPPLY_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_D0_GPIO_Port, LED_D0_Pin, GPIO_PIN_RESET);
    s_imu_context.power_active = false;
    s_imu_context.config_done = false;
    s_imu_context.measurement.valid = false;
    s_imu_context.state = ImuState_Powered_Off;
}

/**
 * @brief Get IMU data.
 * @return Data pointer.
 */
Acceleration_Data const* Acceleration_GetData(void)
{
    return &s_imu_context.measurement;
}

/**
 * @brief I2C TX done.
 * @param hi2c I2C handle.
 */
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    if (hi2c->Instance == I2C3)
    {
        s_imu_context.write_finished = true;
    }
}

/**
 * @brief I2C RX done.
 * @param hi2c I2C handle.
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    if (hi2c->Instance == I2C3)
    {
        s_imu_context.read_finished = true;
    }
}

/**
 * @brief Write config.
 * @param reg_index Index.
 * @return Success.
 */
static bool Imu_InitiateRegisterWrite(uint8_t reg_index)
{
    HAL_StatusTypeDef transfer_status = HAL_ERROR;

    if (reg_index == 0U)
    {
        s_imu_context.register_value = 0x01U;
        transfer_status = HAL_I2C_Mem_Write_DMA(&hi2c3, IMU_I2C_ADDR,
                                                IMU_REG_PWR_CONTROL, I2C_MEMADD_SIZE_8BIT,
                                                &s_imu_context.register_value, 1U);
    }

    else if (reg_index == 1U)
    {
        s_imu_context.register_value = 0x05U;
        transfer_status = HAL_I2C_Mem_Write_DMA(&hi2c3, IMU_I2C_ADDR,
                                                IMU_REG_DLPF_CONFIG, I2C_MEMADD_SIZE_8BIT,
                                                &s_imu_context.register_value, 1U);
    }

    else if (reg_index == 2U)
    {
        s_imu_context.register_value = 0x10U;
        transfer_status = HAL_I2C_Mem_Write_DMA(&hi2c3, IMU_I2C_ADDR,
                                                IMU_REG_GYRO_RANGE_CONFIG, I2C_MEMADD_SIZE_8BIT,
                                                &s_imu_context.register_value, 1U);
    }

    else if (reg_index == 3U)
    {
        s_imu_context.register_value = 0x18U;
        transfer_status = HAL_I2C_Mem_Write_DMA(&hi2c3, IMU_I2C_ADDR,
                                                IMU_REG_ACCEL_RANGE_CONFIG, I2C_MEMADD_SIZE_8BIT,
                                                &s_imu_context.register_value, 1U);
    }

    return (transfer_status == HAL_OK);
}

/**
 * @brief Read accel.
 * @return Success.
 */
static bool Imu_InitiateAccelRetrieval(void)
{
    return (HAL_I2C_Mem_Read_DMA(&hi2c3, IMU_I2C_ADDR, IMU_REG_ACCEL_DATA_START,
                                 I2C_MEMADD_SIZE_8BIT, s_imu_context.accel_raw_bytes, 6U) == HAL_OK);
}

/**
 * @brief Read gyro.
 * @return Success.
 */
static bool Imu_InitiateGyroRetrieval(void)
{
    return (HAL_I2C_Mem_Read_DMA(&hi2c3, IMU_I2C_ADDR, IMU_REG_GYRO_DATA_START,
                                 I2C_MEMADD_SIZE_8BIT, s_imu_context.gyro_raw_bytes, 6U) == HAL_OK);
}

/**
 * @brief Parse accel bytes.
 */
static void Imu_TransformAccelBytes(void)
{
    s_imu_context.measurement.accelXRaw =
        (int16_t)((s_imu_context.accel_raw_bytes[0] << 8)
                  | s_imu_context.accel_raw_bytes[1]);
    s_imu_context.measurement.accelYRaw =
        (int16_t)((s_imu_context.accel_raw_bytes[2] << 8)
                  | s_imu_context.accel_raw_bytes[3]);
    s_imu_context.measurement.accelZRaw =
        (int16_t)((s_imu_context.accel_raw_bytes[4] << 8)
                  | s_imu_context.accel_raw_bytes[5]);
}

/**
 * @brief Parse gyro bytes.
 */
static void Imu_TransformGyroBytes(void)
{
    s_imu_context.measurement.gyroXRaw =
        (int16_t)((s_imu_context.gyro_raw_bytes[0] << 8)
                  | s_imu_context.gyro_raw_bytes[1]);
    s_imu_context.measurement.gyroYRaw =
        (int16_t)((s_imu_context.gyro_raw_bytes[2] << 8)
                  | s_imu_context.gyro_raw_bytes[3]);
    s_imu_context.measurement.gyroZRaw =
        (int16_t)((s_imu_context.gyro_raw_bytes[4] << 8)
                  | s_imu_context.gyro_raw_bytes[5]);
}
