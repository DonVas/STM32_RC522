/*
 * MFRC522Config.h
 *
 *  Created on: Feb 19, 2021
 *      Author: vasil003
 */

#ifndef INC_MFRC522CONFIG_H_
#define INC_MFRC522CONFIG_H_

#define CS_PORT 				RC522_CS_GPIO_Port
#define CS_PIN  				RC522_CS_Pin

#define CS_LOW			     	HAL_GPIO_WritePin(CS_PORT,CS_PIN,GPIO_PIN_RESET);
#define CS_HIGH			      	HAL_GPIO_WritePin(CS_PORT,CS_PIN,GPIO_PIN_SET);
#define CS_INPUT      	      	PIN_INPUT(CS_PORT,CS_PIN);
#define CS_OUTPUT 		      	PIN_OUTPUT(CS_PORT,CS_PIN);

#define RESET_PORT 		      	RC522_Reset_GPIO_Port
#define RESET_PIN  		      	RC522_Reset_Pin

#define RESET_LOW		      	HAL_GPIO_WritePin(RESET_PORT,RESET_PIN,GPIO_PIN_RESET);
#define RESET_HIGH		      	HAL_GPIO_WritePin(RESET_PORT,RESET_PIN,GPIO_PIN_SET);
#define RESET_INPUT   	      	PIN_INPUT(RESET_PORT,RESET_PIN);
#define RESET_OUTPUT  	      	PIN_OUTPUT(RESET_PORT,RESET_PIN);
#define RESET_GET_VALUE()     	(HAL_GPIO_ReadPin(RESET_PORT,RESET_PIN))

#define delayMicroseconds(x) DWT_Delay_us(x);
#define delay(x) DWT_Delay_ms(x);

#endif /* INC_MFRC522CONFIG_H_ */
