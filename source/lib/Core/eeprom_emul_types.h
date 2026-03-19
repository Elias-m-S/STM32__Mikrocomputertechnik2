/**
  ******************************************************************************
  * @file    eeprom_emul_types.h
  * @author  MCD Application Team
  * @brief   This file contains all the functions prototypes for the EEPROM
  *          emulation firmware library.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __EEPROM_EMUL_TYPES_H
#define __EEPROM_EMUL_TYPES_H

/** @addtogroup EEPROM_Emulation
  * @{
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup EEPROM_Exported_Constants EEPROM Exported Constants
  * @{
  */

/** @defgroup Exported_Other_Constants Exported Other Constants
  * @{
  */

/**
  * @brief  EE Status enum definition.
  */
/* Define of the return value */
typedef enum
{
    /** Operation completed successfully. */
    EE_OK = 0U,

    /** Flash erase operation failed. */
    EE_ERASE_ERROR,
    /** Flash write operation failed. */
    EE_WRITE_ERROR,
    /** No active page was found. */
    EE_ERROR_NOACTIVE_PAGE,
    /** No erased page was found. */
    EE_ERROR_NOERASE_PAGE,
    /** No page in erasing state was found. */
    EE_ERROR_NOERASING_PAGE,
    /** No active, receive or valid page was found. */
    EE_ERROR_NOACTIVE_NORECEIVE_NOVALID_PAGE,
    /** Requested virtual variable has no stored data. */
    EE_NO_DATA,
    /** Virtual address is invalid. */
    EE_INVALID_VIRTUALADDRESS,
    /** Page state is invalid. */
    EE_INVALID_PAGE,
    /** Page sequence is inconsistent. */
    EE_INVALID_PAGE_SEQUENCE,
    /** EEPROM element is invalid. */
    EE_INVALID_ELEMENT,
    /** Page transfer operation failed. */
    EE_TRANSFER_ERROR,
    /** Delete operation failed. */
    EE_DELETE_ERROR,
    /** Flash bank configuration is invalid. */
    EE_INVALID_BANK_CFG,

    /** No suitable page was found internally. */
    EE_NO_PAGE_FOUND,
    /** Page is not fully erased. */
    EE_PAGE_NOTERASED,
    /** Page is fully erased. */
    EE_PAGE_ERASED,
    /** Page is full. */
    EE_PAGE_FULL,

    /** Cleanup is required before continuing. */
    EE_CLEANUP_REQUIRED = 0x100U,

#ifdef DUALCORE_FLASH_SHARING
    /** Flash is currently used by CPU2. */
    EE_FLASH_USED,
    /** Hardware semaphore timeout occurred. */
    EE_SEM_TIMEOUT,
#endif

} EE_Status;

/*! Type of page erasing */
typedef enum
{
    /*! pages to erase are erased unconditionally */
    EE_FORCED_ERASE,
    /*! pages to erase are erased only if not fully erased */
    EE_CONDITIONAL_ERASE
} EE_Erase_type;

#if (defined DUALCORE_FLASH_SHARING) || (defined FLASH_LINES_128B)
/* Type of write operations:
       EE_TRANSFER         --> Used by WriteDoubleWord to know when the operation ongoing is a transfer
       EE_SIMPLE_WRITE     --> Used by WriteDoubleWord to know when the operation ongoing is a simple writing */
typedef enum
{
    EE_TRANSFER,
    EE_SIMPLE_WRITE,
    EE_SET_PAGE,
    EE_INIT_WRITE
} EE_Write_type;
#endif

/* Masks of EE_Status return codes */
#define EE_STATUSMASK_ERROR   (uint16_t)0x00FFU /*!< Mask on EE_Status return code, selecting error codes */
#define EE_STATUSMASK_CLEANUP (uint16_t)0x0100U /*!< Mask on EE_Status return code, selecting cleanup request codes */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#endif /* __EEPROM_EMUL_TYPES_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
