/*
 * RF_V3_config.h
 *
 *  Created on: Nov 17, 2023
 *      Author: Norana
 */

/*                                                  Replace with your PINS                              */

#ifndef HAL_RF_V3_RF_V3_CONFIG_H_
#define HAL_RF_V3_RF_V3_CONFIG_H_

#define CSN_PORT PORTA
#define CSN_PIN  PIN0

#define CE_PORT PORTA
#define CE_PIN  PIN1 //OUTPUT_2MHZ_PUSH_PULL

#define CLK_PORT PORTA
#define CLK_PIN  PIN5 // AF_2MHZ_PUSH_PULL

#define MOSI_PORT PORTA
#define MOSI_PIN  PIN7 // AF_2MHZ_PUSH_PULL

#define MISO_PORT PORTA
#define MISO_PIN  PIN6 // INPUT_FLOATING


/*
 * Define the SPI_NUMBER used with RF
 * options :
 * SPI_2 / SPI_1
 */
#define RF_SPI_NUMBER   SPI_1
#define RF_SPI_EN		SPI1_EN
#define RF_SPI_PORT		PORTA
#define RF_SPI_PORT_EN	PORTA_EN

/*************** UART debugging ********/
#define RF_DebugUART	USART_1
#define RF_DebugUART_EN USART1_EN
#define DebugUART_PORT		PORTA
#define DebugUART_TX			PIN9 // PORTA
#define DebugUART_RX			PIN10 // PORTA
/**************************************/
#endif /* HAL_RF_V3_RF_V3_CONFIG_H_ */
