/**
  ******************************************************************************
  * @file    eeprom_emul_conf.h
  * @author  MCD Application Team
  * @brief   EEPROM emulation configuration file.
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

#ifndef __EEPROM_EMUL_CONF_H
#define __EEPROM_EMUL_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* Configuration of eeprom emulation in flash */
#define START_PAGE_ADDRESS      0x08040000U /*!< First page of flash bank 2 */
#define CYCLES_NUMBER           1U          /*!< Minimal configuration */
#define GUARD_PAGES_NUMBER      0U          /*!< Exactly two pages in total */

/* Configuration of crc calculation for eeprom emulation in flash */
/** @brief CRC polynomial length used by EEPROM emulation. */
#define CRC_POLYNOMIAL_LENGTH   LL_CRC_POLYLENGTH_16B

/** @brief CRC polynomial value used by EEPROM emulation. */
#define CRC_POLYNOMIAL_VALUE    0x8005U

/* Number of variables stored in emulated EEPROM */
#define NB_OF_VARIABLES         2U          /*!< ID1 startup counter, ID2 runtime */

#ifdef __cplusplus
}
#endif

#endif /* __EEPROM_EMUL_CONF_H */
