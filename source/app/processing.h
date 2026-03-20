/**
 * @file      processing.h
 *
 * @brief     Header file for data processing and UART output
 */

#ifndef PROCESSING_H_
#define PROCESSING_H_

/*******************************************************************************
 * Includes
 *******************************************************************************/

/*******************************************************************************
 * Exported Functions
 *******************************************************************************/

/*!
 * @brief Initialize processing module
 */
void Processing_Init(void);

/*!
 * @brief Cyclic processing of sensor data formatting and UART output
 */
void Processing_Cyclic(void);

#endif /* PROCESSING_H_ */
