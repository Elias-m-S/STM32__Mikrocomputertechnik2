/**
 * @file      acceleration.c
 * @brief     Acceleration sensor handling for GY-521.
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

/** @brief I2C address of the MPU-6050 sensor in HAL format. */
#define ACC_I2C_ADDRESS                  (0x68U << 1U)

/** @brief Start register for accelerometer raw data. */
#define ACC_REG_ACCEL_XOUT_H             0x3BU

/** @brief Start register for gyroscope raw data. */
#define ACC_REG_GYRO_XOUT_H              0x43U

/** @brief Configuration register address. */
#define ACC_REG_CONFIG                   0x1AU

/** @brief Gyroscope configuration register address. */
#define ACC_REG_GYRO_CONFIG              0x1BU

/** @brief Accelerometer configuration register address. */
#define ACC_REG_ACCEL_CONFIG             0x1CU

/** @brief Power management register address. */
#define ACC_REG_PWR_MGMT_1               0x6BU

/** @brief GPIO port used to power the sensor. */
#define ACC_PWR_GPIO_Port                GPIOC

/** @brief GPIO pin used to power the sensor. */
#define ACC_PWR_Pin                      GPIO_PIN_5

/** @brief Sensor read interval in milliseconds. */
#define ACC_READ_PERIOD_MS               1000U

/** @brief Number of configuration steps. */
#define ACC_CONFIG_STEP_COUNT            4U

/*******************************************************************************
* Local Types and Typedefs
*******************************************************************************/

/** @brief Internal state of the acceleration module. */
typedef enum
{
    /** Sensor is powered off. */
    AccelerationState_Off = 0,
    /** Sensor is idle and ready. */
    AccelerationState_Idle,
    /** Configuration transfer is active. */
    AccelerationState_ConfigBusy,
    /** Accelerometer read transfer is active. */
    AccelerationState_ReadAccelBusy,
    /** Gyroscope read transfer is active. */
    AccelerationState_ReadGyroBusy
} AccelerationState_t;

/** @brief Internal runtime context of the acceleration module. */
typedef struct
{
    /** Current module state. */
    AccelerationState_t state;

    /** Indicates whether sensor power is enabled. */
    bool powerEnabled;
    /** Indicates that configuration was requested. */
    bool configureRequested;
    /** Indicates that configuration is complete. */
    bool configured;

    /** Set when DMA transmit finished. */
    volatile bool txDone;
    /** Set when DMA receive finished. */
    volatile bool rxDone;

    /** Current configuration step. */
    uint8_t configStep;
    /** Temporary transmit byte for register writes. */
    uint8_t txValue;

    /** Raw accelerometer receive buffer. */
    uint8_t accelBuffer[6];
    /** Raw gyroscope receive buffer. */
    uint8_t gyroBuffer[6];

    /** Tick of the last sensor read. */
    uint32_t lastReadTick;

    /** Latest sensor data. */
    Acceleration_Data data;

} Acceleration_Context_t;

/*******************************************************************************
* Static Function Prototypes
*******************************************************************************/

static bool Acceleration_StartConfigStep(uint8_t step);
static bool Acceleration_StartAccelRead(void);
static bool Acceleration_StartGyroRead(void);
static void Acceleration_ParseAccel(void);
static void Acceleration_ParseGyro(void);

/*******************************************************************************
* Static Variables
*******************************************************************************/

/** @brief Global context of the acceleration module. */
static Acceleration_Context_t s_acceleration;

/*******************************************************************************
* Functions
*******************************************************************************/

/**
 * @brief Starts one configuration step of the sensor.
 *
 * @param step Configuration step index.
 *
 * @return true if the transfer was started successfully, otherwise false.
 */
static bool Acceleration_StartConfigStep(uint8_t step)
{
    HAL_StatusTypeDef status = HAL_ERROR;

    if (step == 0U)
    {
        s_acceleration.txValue = 0x01U;
        status = HAL_I2C_Mem_Write_DMA(&hi2c3,
                                       ACC_I2C_ADDRESS,
                                       ACC_REG_PWR_MGMT_1,
                                       I2C_MEMADD_SIZE_8BIT,
                                       &s_acceleration.txValue,
                                       1U);
    }

    else if (step == 1U)
    {
        s_acceleration.txValue = 0x05U;
        status = HAL_I2C_Mem_Write_DMA(&hi2c3,
                                       ACC_I2C_ADDRESS,
                                       ACC_REG_CONFIG,
                                       I2C_MEMADD_SIZE_8BIT,
                                       &s_acceleration.txValue,
                                       1U);
    }

    else if (step == 2U)
    {
        s_acceleration.txValue = 0x10U;
        status = HAL_I2C_Mem_Write_DMA(&hi2c3,
                                       ACC_I2C_ADDRESS,
                                       ACC_REG_GYRO_CONFIG,
                                       I2C_MEMADD_SIZE_8BIT,
                                       &s_acceleration.txValue,
                                       1U);
    }

    else if (step == 3U)
    {
        s_acceleration.txValue = 0x18U;
        status = HAL_I2C_Mem_Write_DMA(&hi2c3,
                                       ACC_I2C_ADDRESS,
                                       ACC_REG_ACCEL_CONFIG,
                                       I2C_MEMADD_SIZE_8BIT,
                                       &s_acceleration.txValue,
                                       1U);
    }

    else
    {
        /* Invalid configuration step */
    }

    if (status == HAL_OK)
    {
        s_acceleration.txDone = false;
        s_acceleration.state = AccelerationState_ConfigBusy;
        return true;
    }

    Error_Handler();
    return false;
}

/**
 * @brief Starts a DMA read of accelerometer raw data.
 *
 * @return true if the transfer was started successfully, otherwise false.
 */
static bool Acceleration_StartAccelRead(void)
{
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read_DMA(&hi2c3,
                                  ACC_I2C_ADDRESS,
                                  ACC_REG_ACCEL_XOUT_H,
                                  I2C_MEMADD_SIZE_8BIT,
                                  s_acceleration.accelBuffer,
                                  sizeof(s_acceleration.accelBuffer));

    if (status == HAL_OK)
    {
        s_acceleration.rxDone = false;
        s_acceleration.state = AccelerationState_ReadAccelBusy;
        return true;
    }

    Error_Handler();
    return false;
}

/**
 * @brief Starts a DMA read of gyroscope raw data.
 *
 * @return true if the transfer was started successfully, otherwise false.
 */
static bool Acceleration_StartGyroRead(void)
{
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read_DMA(&hi2c3,
                                  ACC_I2C_ADDRESS,
                                  ACC_REG_GYRO_XOUT_H,
                                  I2C_MEMADD_SIZE_8BIT,
                                  s_acceleration.gyroBuffer,
                                  sizeof(s_acceleration.gyroBuffer));

    if (status == HAL_OK)
    {
        s_acceleration.rxDone = false;
        s_acceleration.state = AccelerationState_ReadGyroBusy;
        return true;
    }

    Error_Handler();
    return false;
}

/** @brief Parses raw accelerometer bytes into signed axis values. */
static void Acceleration_ParseAccel(void)
{
    s_acceleration.data.accelXRaw = (int16_t)((((uint16_t)s_acceleration.accelBuffer[0]) << 8U)
                                    | ((uint16_t)s_acceleration.accelBuffer[1]));
    s_acceleration.data.accelYRaw = (int16_t)((((uint16_t)s_acceleration.accelBuffer[2]) << 8U)
                                    | ((uint16_t)s_acceleration.accelBuffer[3]));
    s_acceleration.data.accelZRaw = (int16_t)((((uint16_t)s_acceleration.accelBuffer[4]) << 8U)
                                    | ((uint16_t)s_acceleration.accelBuffer[5]));
}

/** @brief Parses raw gyroscope bytes into signed axis values. */
static void Acceleration_ParseGyro(void)
{
    s_acceleration.data.gyroXRaw = (int16_t)((((uint16_t)s_acceleration.gyroBuffer[0]) << 8U)
                                   | ((uint16_t)s_acceleration.gyroBuffer[1]));
    s_acceleration.data.gyroYRaw = (int16_t)((((uint16_t)s_acceleration.gyroBuffer[2]) << 8U)
                                   | ((uint16_t)s_acceleration.gyroBuffer[3]));
    s_acceleration.data.gyroZRaw = (int16_t)((((uint16_t)s_acceleration.gyroBuffer[4]) << 8U)
                                   | ((uint16_t)s_acceleration.gyroBuffer[5]));
}

/** @brief Initializes the acceleration module context. */
void Acceleration_Init(void)
{
    (void)memset(&s_acceleration, 0, sizeof(s_acceleration));
    s_acceleration.state = AccelerationState_Off;
}

/** @brief Enables the sensor supply and prepares the module for operation. */
void Acceleration_MainStateInit(void)
{
    HAL_GPIO_WritePin(ACC_PWR_GPIO_Port, ACC_PWR_Pin, GPIO_PIN_SET);

    s_acceleration.powerEnabled = true;
    s_acceleration.state = AccelerationState_Idle;
    s_acceleration.lastReadTick = HAL_GetTick();
}

/** @brief Cyclic state handling of the acceleration module. */
void Acceleration_Cyclic(void)
{
    uint32_t currentTick;

    if (s_acceleration.powerEnabled == false)
    {
        return;
    }

    currentTick = HAL_GetTick();

    if ((s_acceleration.state == AccelerationState_Idle)
            && (s_acceleration.configureRequested == true)
            && (s_acceleration.configured == false))
    {
        s_acceleration.configureRequested = false;
        s_acceleration.configStep = 0U;
        s_acceleration.data.valid = false;

        HAL_GPIO_WritePin(LED_D0_GPIO_Port, LED_D0_Pin, GPIO_PIN_RESET);

        (void)Acceleration_StartConfigStep(s_acceleration.configStep);
    }

    else if (s_acceleration.state == AccelerationState_ConfigBusy)
    {
        if (s_acceleration.txDone == true)
        {
            s_acceleration.txDone = false;
            s_acceleration.configStep++;

            if (s_acceleration.configStep < ACC_CONFIG_STEP_COUNT)
            {
                (void)Acceleration_StartConfigStep(s_acceleration.configStep);
            }

            else
            {
                s_acceleration.configured = true;
                s_acceleration.state = AccelerationState_Idle;
                s_acceleration.lastReadTick = currentTick;

                HAL_GPIO_WritePin(LED_D0_GPIO_Port, LED_D0_Pin, GPIO_PIN_SET);
            }
        }
    }

    else if ((s_acceleration.state == AccelerationState_Idle)
             && (s_acceleration.configured == true))
    {
        if ((currentTick - s_acceleration.lastReadTick) >= ACC_READ_PERIOD_MS)
        {
            s_acceleration.lastReadTick = currentTick;
            (void)Acceleration_StartAccelRead();
        }
    }

    else if (s_acceleration.state == AccelerationState_ReadAccelBusy)
    {
        if (s_acceleration.rxDone == true)
        {
            s_acceleration.rxDone = false;
            Acceleration_ParseAccel();
            (void)Acceleration_StartGyroRead();
        }
    }

    else if (s_acceleration.state == AccelerationState_ReadGyroBusy)
    {
        if (s_acceleration.rxDone == true)
        {
            s_acceleration.rxDone = false;
            Acceleration_ParseGyro();

            s_acceleration.data.valid = true;
            s_acceleration.data.sampleCounter++;
            s_acceleration.state = AccelerationState_Idle;
        }
    }

    else
    {
        /* No action required */
    }
}

/** @brief Requests sensor configuration. */
void Acceleration_RequestConfigure(void)
{
    if (s_acceleration.powerEnabled == false)
    {
        return;
    }

    if (s_acceleration.configured == false)
    {
        s_acceleration.configureRequested = true;
    }
}

/** @brief Powers down the sensor and resets the module state. */
void Acceleration_PrepareShutdown(void)
{
    HAL_GPIO_WritePin(ACC_PWR_GPIO_Port, ACC_PWR_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_D0_GPIO_Port, LED_D0_Pin, GPIO_PIN_RESET);

    s_acceleration.powerEnabled = false;
    s_acceleration.configureRequested = false;
    s_acceleration.configured = false;
    s_acceleration.data.valid = false;
    s_acceleration.state = AccelerationState_Off;
}

/**
 * @brief Returns the configuration status of the sensor.
 *
 * @return true if the sensor is configured, otherwise false.
 */
bool Acceleration_IsConfigured(void)
{
    return s_acceleration.configured;
}

/**
 * @brief Returns the latest sensor data.
 *
 * @return Pointer to the internal data structure.
 */
Acceleration_Data const* Acceleration_GetData(void)
{
    return &s_acceleration.data;
}

/**
 * @brief HAL callback for completed I2C memory write DMA transfers.
 *
 * @param hi2c Pointer to the I2C handle.
 */
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    if ((hi2c != NULL) && (hi2c->Instance == I2C3))
    {
        s_acceleration.txDone = true;
    }
}

/**
 * @brief HAL callback for completed I2C memory read DMA transfers.
 *
 * @param hi2c Pointer to the I2C handle.
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c)
{
    if ((hi2c != NULL) && (hi2c->Instance == I2C3))
    {
        s_acceleration.rxDone = true;
    }
}
