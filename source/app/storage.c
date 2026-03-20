/**
 * @file      storage.c
 * @brief     Non-volatile data persistence using STM EEPROM emulation layer.
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

/** @brief Virtual EEPROM key for boot sequence counter. */
#define NVRAM_KEY_BOOT_COUNT              1U

/** @brief Virtual EEPROM key for cumulative execution time. */
#define NVRAM_KEY_EXEC_TIME_MILLIS        2U

/** @brief Flush interval for runtime metrics to non-volatile storage. */
#define NVRAM_FLUSH_INTERVAL_MS           5000UL

/*******************************************************************************
 * Local Types and Typedefs
 *******************************************************************************/

/** @brief Persistent data manager context. */
typedef struct
{
    /** Module initialization status flag. */
    bool isReady;
    /** Non-volatile memory interface availability. */
    bool memoryAvailable;

    /** Current boot counter value. */
    uint32_t bootCount;
    /** Current execution time accumulator. */
    uint32_t execTimeMs;

    /** Previous HAL_GetTick() snapshot. */
    uint32_t prevSystemTick;
    /** Last time metrics were flushed to NVM. */
    uint32_t prevFlushTick;
} PersistentData_Manager;

/*******************************************************************************
 * Static Function Prototypes
 *******************************************************************************/

static void PersistentData_InitializePartition(void);
static uint32_t PersistentData_FetchValue(uint16_t key);
static void PersistentData_StoreValue(uint16_t key, uint32_t val);

/*******************************************************************************
 * Static Variables
 *******************************************************************************/

/** @brief Global persistent data manager instance. */
static PersistentData_Manager g_persistMgr;

/*******************************************************************************
 * Functions
 *******************************************************************************/

/** @brief Initializes the EEPROM partition if needed or repairs corruption. */
static void PersistentData_InitializePartition(void)
{
    EE_Status initStatus;

    __HAL_RCC_CRC_CLK_ENABLE();
    HAL_CRC_DeInit(&hcrc);
    HAL_CRC_Init(&hcrc);

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        Error_Handler();
    }

    initStatus = EE_Init(EE_CONDITIONAL_ERASE);

    if (initStatus == EE_CLEANUP_REQUIRED)
    {
        initStatus = EE_CleanUp();

        if (initStatus != EE_OK)
        {
            (void) HAL_FLASH_Lock();
            Error_Handler();
        }
    }

    else if (initStatus != EE_OK)
    {
        initStatus = EE_Format(EE_FORCED_ERASE);

        if (initStatus != EE_OK)
        {
            (void) HAL_FLASH_Lock();
            Error_Handler();
        }

        initStatus = EE_Init(EE_CONDITIONAL_ERASE);

        if (initStatus == EE_CLEANUP_REQUIRED)
        {
            initStatus = EE_CleanUp();
        }

        if (initStatus != EE_OK)
        {
            (void) HAL_FLASH_Lock();
            Error_Handler();
        }
    }

    HAL_CRC_DeInit(&hcrc);
    HAL_CRC_Init(&hcrc);

    if (HAL_FLASH_Lock() != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief Retrieves a 32-bit value from virtual EEPROM.
 *
 * @param key Virtual EEPROM variable identifier.
 *
 * @return Retrieved value, 0 if key does not exist.
 */
static uint32_t PersistentData_FetchValue(uint16_t key)
{
    EE_Status fetchStatus;
    uint32_t fetchedVal = 0UL;

    fetchStatus = EE_ReadVariable32bits(key, &fetchedVal);

    if (fetchStatus == EE_OK)
    {
        return fetchedVal;
    }

    if (fetchStatus == EE_NO_DATA)
    {
        return 0UL;
    }

    Error_Handler();
    return 0UL;
}

/**
 * @brief Persists a 32-bit value to virtual EEPROM.
 *
 * @param key Virtual EEPROM variable identifier.
 * @param val Value to persist.
 */
static void PersistentData_StoreValue(uint16_t key, uint32_t val)
{
    EE_Status storeStatus;

    if (HAL_FLASH_Unlock() != HAL_OK)
    {
        Error_Handler();
    }

    storeStatus = EE_WriteVariable32bits(key, val);

    if (storeStatus == EE_CLEANUP_REQUIRED)
    {
        storeStatus = EE_CleanUp();

        if (storeStatus != EE_OK)
        {
            (void) HAL_FLASH_Lock();
            Error_Handler();
        }

        storeStatus = EE_WriteVariable32bits(key, val);
    }

    if (HAL_FLASH_Lock() != HAL_OK)
    {
        Error_Handler();
    }

    if (storeStatus != EE_OK)
    {
        Error_Handler();
    }
}

/** @brief Prepares persistent data manager for operation. */
void Storage_Init(void)
{
    (void) memset(&g_persistMgr, 0, sizeof(g_persistMgr));
    g_persistMgr.isReady = true;
}

/** @brief Loads metrics from NVM and initializes runtime tracking. */
void Storage_MainStateInit(void)
{
    uint32_t nowTick;

    if (g_persistMgr.isReady == false)
    {
        Error_Handler();
    }

    PersistentData_InitializePartition();

    g_persistMgr.bootCount = PersistentData_FetchValue(NVRAM_KEY_BOOT_COUNT);
    g_persistMgr.execTimeMs = PersistentData_FetchValue(
                                  NVRAM_KEY_EXEC_TIME_MILLIS);

    g_persistMgr.bootCount++;
    PersistentData_StoreValue(NVRAM_KEY_BOOT_COUNT, g_persistMgr.bootCount);

    nowTick = HAL_GetTick();
    g_persistMgr.prevSystemTick = nowTick;
    g_persistMgr.prevFlushTick = nowTick;
    g_persistMgr.memoryAvailable = true;
}

/** @brief Accumulates runtime and flushes to NVM at periodic intervals. */
void Storage_Cyclic(void)
{
    uint32_t nowTick;
    uint32_t elapsedMs;

    if ((g_persistMgr.isReady == false)
            || (g_persistMgr.memoryAvailable == false))
    {
        return;
    }

    nowTick = HAL_GetTick();
    elapsedMs = nowTick - g_persistMgr.prevSystemTick;
    g_persistMgr.prevSystemTick = nowTick;

    g_persistMgr.execTimeMs += elapsedMs;

    if ((nowTick - g_persistMgr.prevFlushTick) >= NVRAM_FLUSH_INTERVAL_MS)
    {
        PersistentData_StoreValue(NVRAM_KEY_EXEC_TIME_MILLIS,
                                  g_persistMgr.execTimeMs);
        g_persistMgr.prevFlushTick = nowTick;
    }
}

/** @brief Flushes accumulated runtime to NVM before system shutdown. */
void Storage_PrepareShutdown(void)
{
    if ((g_persistMgr.isReady == false)
            || (g_persistMgr.memoryAvailable == false))
    {
        return;
    }

    PersistentData_StoreValue(NVRAM_KEY_EXEC_TIME_MILLIS,
                              g_persistMgr.execTimeMs);
    g_persistMgr.prevFlushTick = HAL_GetTick();
}

