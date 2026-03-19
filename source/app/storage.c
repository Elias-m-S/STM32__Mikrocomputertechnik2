/**
 * @file      storage.c
 * @brief     Persistent storage handling using STM EEPROM emulation.
 */

/*******************************************************************************
* Includes
*******************************************************************************/

#include "storage.h"

#include <string.h>

#include "eeprom_emul.h"
#include "eeprom_emul_types.h"
#include "crc.h"

/*******************************************************************************
* Defines
*******************************************************************************/

/** @brief EEPROM identifier for the startup counter. */
#define STORAGE_ID_STARTUP_COUNTER        1U

/** @brief EEPROM identifier for the accumulated runtime in milliseconds. */
#define STORAGE_ID_RUNTIME_MS             2U

/** @brief Period in milliseconds for persisting runtime to non-volatile memory. */
#define STORAGE_PERSIST_PERIOD_MS         5000UL

/*******************************************************************************
* Local Types and Typedefs
*******************************************************************************/

/** @brief Internal runtime context of the storage module. */
typedef struct
{
    /** Indicates whether the module was initialized. */
    bool initialized;
    /** Indicates whether the EEPROM emulation is ready. */
    bool nvmReady;

    /** Persisted startup counter value. */
    uint32_t startupCounter;
    /** Accumulated runtime in milliseconds. */
    uint32_t accumulatedRuntimeMs;

    /** Tick of the last runtime update. */
    uint32_t lastTick;
    /** Tick of the last persist operation. */
    uint32_t lastPersistTick;
} Storage_Context;

/*******************************************************************************
* Static Function Prototypes
*******************************************************************************/

static void Storage_InitPartitionIfRequired(void);
static uint32_t Storage_ReadVariable32(uint16_t identifier);
static void Storage_WriteVariable32(uint16_t identifier, uint32_t value);

/*******************************************************************************
* Static Variables
*******************************************************************************/

/** @brief Global context of the storage module. */
static Storage_Context s_storage;

/*******************************************************************************
* Functions
*******************************************************************************/

/** @brief Initializes and repairs the EEPROM emulation partition if required. */
static void Storage_InitPartitionIfRequired(void)
{
    EE_Status status;

    __HAL_RCC_CRC_CLK_ENABLE();
    HAL_CRC_DeInit(&hcrc);
    HAL_CRC_Init(&hcrc);

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        Error_Handler();
    }

    status = EE_Init(EE_CONDITIONAL_ERASE);

    if (status == EE_CLEANUP_REQUIRED)
    {
        status = EE_CleanUp();

        if (status != EE_OK)
        {
            (void)HAL_FLASH_Lock();
            Error_Handler();
        }
    }

    else if (status != EE_OK)
    {
        status = EE_Format(EE_FORCED_ERASE);

        if (status != EE_OK)
        {
            (void)HAL_FLASH_Lock();
            Error_Handler();
        }

        status = EE_Init(EE_CONDITIONAL_ERASE);

        if (status == EE_CLEANUP_REQUIRED)
        {
            status = EE_CleanUp();
        }

        if (status != EE_OK)
        {
            (void)HAL_FLASH_Lock();
            Error_Handler();
        }
    }

    else
    {
        /* Partition already valid */
    }

    HAL_CRC_DeInit(&hcrc);
    HAL_CRC_Init(&hcrc);

    if (HAL_FLASH_Lock() != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief Reads a 32-bit value from EEPROM emulation.
 *
 * @param identifier Virtual EEPROM variable identifier.
 *
 * @return Stored value, or 0 if no value exists.
 */
static uint32_t Storage_ReadVariable32(uint16_t identifier)
{
    EE_Status status;
    uint32_t value = 0UL;

    status = EE_ReadVariable32bits(identifier, &value);

    if (status == EE_OK)
    {
        return value;
    }

    if (status == EE_NO_DATA)
    {
        return 0UL;
    }

    Error_Handler();
    return 0UL;
}

/**
 * @brief Writes a 32-bit value to EEPROM emulation.
 *
 * @param identifier Virtual EEPROM variable identifier.
 * @param value Value to write.
 */
static void Storage_WriteVariable32(uint16_t identifier, uint32_t value)
{
    EE_Status status;

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        Error_Handler();
    }

    status = EE_WriteVariable32bits(identifier, value);

    if (status == EE_CLEANUP_REQUIRED)
    {
        status = EE_CleanUp();

        if (status != EE_OK)
        {
            (void)HAL_FLASH_Lock();
            Error_Handler();
        }

        status = EE_WriteVariable32bits(identifier, value);
    }

    if (HAL_FLASH_Lock() != HAL_OK)
    {
        Error_Handler();
    }

    if (status != EE_OK)
    {
        Error_Handler();
    }
}

/** @brief Initializes the storage module context. */
void Storage_Init(void)
{
    (void)memset(&s_storage, 0, sizeof(s_storage));
    s_storage.initialized = true;
}

/** @brief Initializes persistent storage for the active main state. */
void Storage_MainStateInit(void)
{
    uint32_t currentTick;

    if (s_storage.initialized == false)
    {
        Error_Handler();
    }

    Storage_InitPartitionIfRequired();

    s_storage.startupCounter = Storage_ReadVariable32(STORAGE_ID_STARTUP_COUNTER);
    s_storage.accumulatedRuntimeMs = Storage_ReadVariable32(STORAGE_ID_RUNTIME_MS);

    s_storage.startupCounter++;
    Storage_WriteVariable32(STORAGE_ID_STARTUP_COUNTER, s_storage.startupCounter);

    currentTick = HAL_GetTick();
    s_storage.lastTick = currentTick;
    s_storage.lastPersistTick = currentTick;
    s_storage.nvmReady = true;
}

/** @brief Cyclic runtime accumulation and periodic persistence. */
void Storage_Cyclic(void)
{
    uint32_t currentTick;
    uint32_t deltaTick;

    if ((s_storage.initialized == false) || (s_storage.nvmReady == false))
    {
        return;
    }

    currentTick = HAL_GetTick();
    deltaTick = currentTick - s_storage.lastTick;
    s_storage.lastTick = currentTick;

    s_storage.accumulatedRuntimeMs += deltaTick;

    if ((currentTick - s_storage.lastPersistTick) >= STORAGE_PERSIST_PERIOD_MS)
    {
        Storage_WriteVariable32(STORAGE_ID_RUNTIME_MS, s_storage.accumulatedRuntimeMs);
        s_storage.lastPersistTick = currentTick;
    }
}

/** @brief Writes the current runtime value before shutdown. */
void Storage_PrepareShutdown(void)
{
    if ((s_storage.initialized == false) || (s_storage.nvmReady == false))
    {
        return;
    }

    Storage_WriteVariable32(STORAGE_ID_RUNTIME_MS, s_storage.accumulatedRuntimeMs);
    s_storage.lastPersistTick = HAL_GetTick();
}

/**
 * @brief Returns the persisted startup counter.
 *
 * @return Startup counter value.
 */
uint32_t Storage_GetStartupCounter(void)
{
    return s_storage.startupCounter;
}

/**
 * @brief Returns the accumulated runtime in milliseconds.
 *
 * @return Accumulated runtime value.
 */
uint32_t Storage_GetAccumulatedRuntimeMs(void)
{
    return s_storage.accumulatedRuntimeMs;
}
