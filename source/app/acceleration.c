/**
 * @file      acceleration.c
 * @brief     Inertial measurement unit sensor handling for GY-521.
 */

/*******************************************************************************
* Includes
*******************************************************************************/

#include "acceleration.h"

#include <string.h>

#include "main.h"
#include "gpio.h"
#include "i2c.h"

/*******************************************************************************
* Defines
*******************************************************************************/

/** @brief I2C slave address of the MPU-6050 sensor. */
#define IMU_I2C_ADDR                     (0x68U << 1U)

/** @brief First register address for accelerometer measurements. */
#define IMU_REG_ACCEL_DATA_START         0x3BU

/** @brief First register address for gyroscope measurements. */
#define IMU_REG_GYRO_DATA_START          0x43U

/** @brief Digital low-pass filter configuration register. */
#define IMU_REG_DLPF_CONFIG              0x1AU

/** @brief Rotational rate full-scale configuration register. */
#define IMU_REG_GYRO_RANGE_CONFIG        0x1BU

/** @brief Linear acceleration full-scale configuration register. */
#define IMU_REG_ACCEL_RANGE_CONFIG       0x1CU

/** @brief Power control and clock selection register. */
#define IMU_REG_PWR_CONTROL              0x6BU

/** @brief Port for power supply control. */
#define IMU_SUPPLY_PORT                  GPIOC

/** @brief Pin for power supply control. */
#define IMU_SUPPLY_PIN                   GPIO_PIN_5

/** @brief Sampling interval in milliseconds. */
#define IMU_SAMPLE_INTERVAL_MS           1000U

/** @brief Number of configuration register writes. */
#define IMU_CONFIG_REGISTER_COUNT        4U

/*******************************************************************************
* Local Types and Typedefs
*******************************************************************************/

/** @brief State machine states for sensor operation. */
typedef enum
{
    /** Sensor powered off. */
    ImuState_Powered_Off = 0,
    /** Sensor ready and idle. */
    ImuState_Ready,
    /** Configuration sequence in progress. */
    ImuState_Configuring,
    /** Accelerometer data retrieval in progress. */
    ImuState_Fetching_Accel,
    /** Gyroscope data retrieval in progress. */
    ImuState_Fetching_Gyro
} ImuState_t;

/** @brief Module operational context and state variables. */
typedef struct
{
    /** Current operational state. */
    ImuState_t state;

    /** Power supply status flag. */
    bool power_active;
    /** Pending configuration request flag. */
    bool config_pending;
    /** Configuration completion status flag. */
    bool config_done;

    /** DMA write transfer completion flag. */
    volatile bool write_finished;
    /** DMA read transfer completion flag. */
    volatile bool read_finished;

    /** Current configuration register index. */
    uint8_t register_index;
    /** Temporary storage for register write value. */
    uint8_t register_value;

    /** Buffer for accelerometer raw byte data. */
    uint8_t accel_raw_bytes[6];
    /** Buffer for gyroscope raw byte data. */
    uint8_t gyro_raw_bytes[6];

    /** Timestamp of most recent measurement. */
    uint32_t measurement_timestamp;

    /** Latest measurement data. */
    Acceleration_Data measurement;

} ImuContext_t;

/*******************************************************************************
* Static Function Prototypes
*******************************************************************************/

static bool Imu_InitiateRegisterWrite(uint8_t reg_index);
static bool Imu_InitiateAccelRetrieval(void);
static bool Imu_InitiateGyroRetrieval(void);
static void Imu_TransformAccelBytes(void);
static void Imu_TransformGyroBytes(void);

/*******************************************************************************
* Static Variables
*******************************************************************************/

/** @brief Module runtime context. */
static ImuContext_t s_imu_context;

/*******************************************************************************
* Functions
*******************************************************************************/

/**
 * @brief Initiates a DMA-based register write operation.
 *
 * @param reg_index Index of configuration step.
 *
 * @return true on successful transfer initiation, false otherwise.
 */
static bool Imu_InitiateRegisterWrite(uint8_t reg_index)
{
    HAL_StatusTypeDef transfer_status = HAL_ERROR;

    if (reg_index == 0U)
    {
        s_imu_context.register_value = 0x01U;
        transfer_status = HAL_I2C_Mem_Write_DMA(&hi2c3,
                                                IMU_I2C_ADDR,
                                                IMU_REG_PWR_CONTROL,
                                                I2C_MEMADD_SIZE_8BIT,
                                                &s_imu_context.register_value,
                                                1U);
    }

    else if (reg_index == 1U)
    {
        s_imu_context.register_value = 0x05U;
        transfer_status = HAL_I2C_Mem_Write_DMA(&hi2c3,
                                                IMU_I2C_ADDR,
                                                IMU_REG_DLPF_CONFIG,
                                                I2C_MEMADD_SIZE_8BIT,
                                                &s_imu_context.register_value,
                                                1U);
    }

    else if (reg_index == 2U)
    {
        s_imu_context.register_value = 0x10U;
        transfer_status = HAL_I2C_Mem_Write_DMA(&hi2c3,
                                                IMU_I2C_ADDR,
                                                IMU_REG_GYRO_RANGE_CONFIG,
                                                I2C_MEMADD_SIZE_8BIT,
                                                &s_imu_context.register_value,
                                                1U);
    }

    else if (reg_index == 3U)
    {
        s_imu_context.register_value = 0x18U;
        transfer_status = HAL_I2C_Mem_Write_DMA(&hi2c3,
                                                IMU_I2C_ADDR,
                                                IMU_REG_ACCEL_RANGE_CONFIG,
                                                I2C_MEMADD_SIZE_8BIT,
                                                &s_imu_context.register_value,
                                                1U);
    }

    else
    {
        /* Undefined configuration register */
    }

    if (transfer_status == HAL_OK)
    {
        s_imu_context.write_finished = false;
        s_imu_context.state = ImuState_Configuring;
        return true;
    }

    Error_Handler();
    return false;
}

/**
 * @brief Initiates DMA-based accelerometer data retrieval.
 *
 * @return true on successful transfer initiation, false otherwise.
 */
static bool Imu_InitiateAccelRetrieval(void)
{
    HAL_StatusTypeDef transfer_status;

    transfer_status = HAL_I2C_Mem_Read_DMA(&hi2c3,
                                           IMU_I2C_ADDR,
                                           IMU_REG_ACCEL_DATA_START,
                                           I2C_MEMADD_SIZE_8BIT,
                                           s_imu_context.accel_raw_bytes,
                                           sizeof(s_imu_context.accel_raw_bytes));

    if (transfer_status == HAL_OK)
    {
        s_imu_context.read_finished = false;
        s_imu_context.state = ImuState_Fetching_Accel;
        return true;
    }

    Error_Handler();
    return false;
}

/**
 * @brief Initiates DMA-based gyroscope data retrieval.
 *
 * @return true on successful transfer initiation, false otherwise.
 */
static bool Imu_InitiateGyroRetrieval(void)
{
    HAL_StatusTypeDef transfer_status;

    transfer_status = HAL_I2C_Mem_Read_DMA(&hi2c3,
                                           IMU_I2C_ADDR,
                                           IMU_REG_GYRO_DATA_START,
                                           I2C_MEMADD_SIZE_8BIT,
                                           s_imu_context.gyro_raw_bytes,
                                           sizeof(s_imu_context.gyro_raw_bytes));

    if (transfer_status == HAL_OK)
    {
        s_imu_context.read_finished = false;
        s_imu_context.state = ImuState_Fetching_Gyro;
        return true;
    }

    Error_Handler();
    return false;
}

/** @brief Converts accelerometer byte buffer to signed integer values. */
static void Imu_TransformAccelBytes(void)
{
    s_imu_context.measurement.accelXRaw =
        (int16_t)((((uint16_t)s_imu_context.accel_raw_bytes[0]) << 8U)
                  | ((uint16_t)s_imu_context.accel_raw_bytes[1]));
    s_imu_context.measurement.accelYRaw =
        (int16_t)((((uint16_t)s_imu_context.accel_raw_bytes[2]) << 8U)
                  | ((uint16_t)s_imu_context.accel_raw_bytes[3]));
    s_imu_context.measurement.accelZRaw =
        (int16_t)((((uint16_t)s_imu_context.accel_raw_bytes[4]) << 8U)
                  | ((uint16_t)s_imu_context.accel_raw_bytes[5]));
}

/** @brief Converts gyroscope byte buffer to signed integer values. */
static void Imu_TransformGyroBytes(void)
{
    s_imu_context.measurement.gyroXRaw =
        (int16_t)((((uint16_t)s_imu_context.gyro_raw_bytes[0]) << 8U)
                  | ((uint16_t)s_imu_context.gyro_raw_bytes[1]));
    s_imu_context.measurement.gyroYRaw =
        (int16_t)((((uint16_t)s_imu_context.gyro_raw_bytes[2]) << 8U)
                  | ((uint16_t)s_imu_context.gyro_raw_bytes[3]));
    s_imu_context.measurement.gyroZRaw =
        (int16_t)((((uint16_t)s_imu_context.gyro_raw_bytes[4]) << 8U)
                  | ((uint16_t)s_imu_context.gyro_raw_bytes[5]));
}

/** @brief Resets module context to initial state. */
void Acceleration_Init(void)
{
    (void)memset(&s_imu_context, 0, sizeof(s_imu_context));
    s_imu_context.state = ImuState_Powered_Off;
}

/** @brief Enables power supply and prepares sensor for operation. */
void Acceleration_MainStateInit(void)
{
    HAL_GPIO_WritePin(IMU_SUPPLY_PORT, IMU_SUPPLY_PIN, GPIO_PIN_SET);

    s_imu_context.power_active = true;
    s_imu_context.state = ImuState_Ready;
    s_imu_context.measurement_timestamp = HAL_GetTick();
}

/** @brief Main cyclic processing routine for sensor management. */
void Acceleration_Cyclic(void)
{
    uint32_t current_timestamp;

    if (s_imu_context.power_active == false)
    {
        return;
    }

    current_timestamp = HAL_GetTick();

    if ((s_imu_context.state == ImuState_Ready)
            && (s_imu_context.config_pending == true)
            && (s_imu_context.config_done == false))
    {
        s_imu_context.config_pending = false;
        s_imu_context.register_index = 0U;
        s_imu_context.measurement.valid = false;

        HAL_GPIO_WritePin(LED_D0_GPIO_Port, LED_D0_Pin, GPIO_PIN_RESET);

        (void)Imu_InitiateRegisterWrite(s_imu_context.register_index);
    }

    else if (s_imu_context.state == ImuState_Configuring)
    {
        if (s_imu_context.write_finished == true)
        {
            s_imu_context.write_finished = false;
            s_imu_context.register_index++;

            if (s_imu_context.register_index < IMU_CONFIG_REGISTER_COUNT)
            {
                (void)Imu_InitiateRegisterWrite(s_imu_context.register_index);
            }

            else
            {
                s_imu_context.config_done = true;
                s_imu_context.state = ImuState_Ready;
                s_imu_context.measurement_timestamp = current_timestamp;

                HAL_GPIO_WritePin(LED_D0_GPIO_Port, LED_D0_Pin, GPIO_PIN_SET);
            }
        }
    }

    else if ((s_imu_context.state == ImuState_Ready)
             && (s_imu_context.config_done == true))
    {
        if ((current_timestamp - s_imu_context.measurement_timestamp) >= IMU_SAMPLE_INTERVAL_MS)
        {
            s_imu_context.measurement_timestamp = current_timestamp;
            (void)Imu_InitiateAccelRetrieval();
        }
    }

    else if (s_imu_context.state == ImuState_Fetching_Accel)
    {
        if (s_imu_context.read_finished == true)
        {
            s_imu_context.read_finished = false;
            Imu_TransformAccelBytes();
            (void)Imu_InitiateGyroRetrieval();
        }
    }

    else if (s_imu_context.state == ImuState_Fetching_Gyro)
    {
        if (s_imu_context.read_finished == true)
        {
            s_imu_context.read_finished = false;
            Imu_TransformGyroBytes();

            s_imu_context.measurement.valid = true;
            s_imu_context.measurement.sampleCounter++;
            s_imu_context.state = ImuState_Ready;
        }
    }

    else
    {
        /* Idle state */
    }
}

/** @brief Signals that sensor configuration should be performed. */
void Acceleration_RequestConfigure(void)
{
    if (s_imu_context.power_active == false)
    {
        return;
    }

    if (s_imu_context.config_done == false)
    {
        s_imu_context.config_pending = true;
    }
}

/** @brief Disables power and resets all operational flags. */
void Acceleration_PrepareShutdown(void)
{
    HAL_GPIO_WritePin(IMU_SUPPLY_PORT, IMU_SUPPLY_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_D0_GPIO_Port, LED_D0_Pin, GPIO_PIN_RESET);

    s_imu_context.power_active = false;
    s_imu_context.config_pending = false;
    s_imu_context.config_done = false;
    s_imu_context.measurement.valid = false;
    s_imu_context.state = ImuState_Powered_Off;
}

/**
 * @brief Returns configuration status.
 *
 * @return true if sensor configuration is complete, false otherwise.
 */
bool Acceleration_IsConfigured(void)
{
    return s_imu_context.config_done;
}

/**
 * @brief Provides access to latest measurement data.
 *
 * @return Pointer to internal measurement structure.
 */
Acceleration_Data const* Acceleration_GetData(void)
{
    return &s_imu_context.measurement;
}

/**
 * @brief HAL callback triggered on DMA write completion.
 *
 * @param hi2c Pointer to I2C interface handle.
 */
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    if ((hi2c != NULL) && (hi2c->Instance == I2C3))
    {
        s_imu_context.write_finished = true;
    }
}

/**
 * @brief HAL callback triggered on DMA read completion.
 *
 * @param hi2c Pointer to I2C interface handle.
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    if ((hi2c != NULL) && (hi2c->Instance == I2C3))
    {
        s_imu_context.read_finished = true;
    }
}
