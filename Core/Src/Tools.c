/*
 * Tools.c
 *
 *  Created on: Feb 18, 2021
 *      Author: vasil003
 */
#include "Tools.h"
#include "usbd_cdc_if.h"
#include "stdio.h"

extern SPI_HandleTypeDef hspi1;
extern USBD_HandleTypeDef hUsbDeviceFS;

serial Serial = {
				.print = print,
				.println = println,
				.isConnected = FALSE,
				};

spi SPI = {   .recive = HAL_SPI_Receive,
			  .transfer = HAL_SPI_Transmit,
		      .transmitRecive = HAL_SPI_TransmitReceive
		  };

/*Delay functions*/
#if (HWTIMER == 1 & SYSDELAY == 0)
#define TIMER  TIM2
static TIM_HandleTypeDef HTIMx;
volatile uint32_t gu32_ticks = 0;
#endif

#if (SYSDELAY == 1 & HWTIMER == 0)
uint32_t DWT_Delay_Init(void)
{
    /* Disable TRC */
    CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk; // ~0x01000000;
    /* Enable TRC */
    CoreDebug->DEMCR |=  CoreDebug_DEMCR_TRCENA_Msk; // 0x01000000;

    /* Disable clock cycle counter */
    DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk; //~0x00000001;
    /* Enable  clock cycle counter */
    DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk; //0x00000001;

    /* Reset the clock cycle counter value */
    DWT->CYCCNT = 0;

    /* 3 NO OPERATION instructions */
    __ASM volatile ("NOP");
    __ASM volatile ("NOP");
    __ASM volatile ("NOP");

    /* Check if clock cycle counter has started */
    if(DWT->CYCCNT)
    {
       return 0; /*clock cycle counter started*/
    }
    else
    {
      return 1; /*clock cycle counter not started*/
    }
}
#elif (HWTIMER == 1 & SYSDELAY == 0)
void TimerDelay_Init(void)
{
	gu32_ticks = (HAL_RCC_GetHCLKFreq() / 1000000);


    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    HTIMx.Instance = TIMER;
    HTIMx.Init.Prescaler = gu32_ticks-1;
    HTIMx.Init.CounterMode = TIM_COUNTERMODE_UP;
    HTIMx.Init.Period = 65535;
    HTIMx.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HTIMx.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&HTIMx) != HAL_OK)
    {
      Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&HTIMx, &sClockSourceConfig) != HAL_OK)
    {
      Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&HTIMx, &sMasterConfig) != HAL_OK)
    {
      Error_Handler();
    }

    HAL_TIM_Base_Start(&HTIMx);

}

void delay_us(volatile uint16_t au16_us)
{
	HTIMx.Instance->CNT = 0;
	while (HTIMx.Instance->CNT < au16_us);
}

void delay_ms(volatile uint16_t au16_ms)
{
	while(au16_ms > 0)
	{
		HTIMx.Instance->CNT = 0;
		au16_ms--;
		while (HTIMx.Instance->CNT < 1000);
	}
}
#endif

/*Pin I/O functions*/
void PIN_INPUT (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	  GPIO_InitTypeDef GPIO_InitStruct;
	  GPIO_InitStruct.Pin = GPIO_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void PIN_OUTPUT (GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
	 GPIO_InitTypeDef GPIO_InitStruct;

	  GPIO_InitStruct.Pin = GPIO_Pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

/*Serial functions*/
void print(const char* text, strDef det){

	DWT_Delay_ms(1);
	if(serialIsConnected()){
		if(det == STR)
		{
			CDC_Transmit_FS((uint8_t*) text, strlen(text));
		}
		else if(det == HEX)
		{
			char buff[100];

			sprintf(buff, "%X", (unsigned int)text);
			CDC_Transmit_FS((uint8_t*) buff, strlen(buff));
		}
		else if(det == DEC){
			char buff[strlen(text)];

			sprintf(buff, "%u", (unsigned int)text);
			CDC_Transmit_FS((uint8_t*) buff, strlen(buff));
		}
	}
}

void println(const char* text, strDef det){

	DWT_Delay_ms(1);
	if(serialIsConnected()){
		if(det == STR)
		{
			char buff[100];

			sprintf(buff, "%s\r\n",text);
			CDC_Transmit_FS((uint8_t*) buff, strlen(buff));
		}
		else if(det == HEX)
		{
			char buff[100];

			sprintf(buff, "%02X\r\n", (unsigned int)text);
			CDC_Transmit_FS((uint8_t*) buff, strlen(buff));
		}
		else if(det == DEC){
			char buff[strlen(text)];

			sprintf(buff, "%u\r\n", (unsigned int)text);
			CDC_Transmit_FS((uint8_t*) buff, strlen(buff));
		}
	}
}

boolean serialIsConnected()
{
	if(hUsbDeviceFS.dev_state == CDC_SET_CONTROL_LINE_STATE)
	{
//    	if(req->wValue &0x0001 != 0)
//    	{
//    		Serial.isConnected = TRUE;
//        	return TRUE;
//    	}
//    	else
//    	{
//    		Serial.isConnected = FALSE;
//    		return FALSE;
//    	}
		Serial.isConnected = TRUE;
		return TRUE;
	}
	Serial.isConnected = FALSE;
	return FALSE;
}
