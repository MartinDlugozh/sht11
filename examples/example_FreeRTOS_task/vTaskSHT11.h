/*
 * vTaskSHT11.h
 *
 *  Created on: Feb 5, 2018
 *      Author: Dr. Saldon
 */

#ifndef VTASKSHT11_H_
#define VTASKSHT11_H_

/*-----------------------------------------------------------------------------
MACRO SECTION
-----------------------------------------------------------------------------*/
#define SHT11_SAMPLING_PERIOD 		500 		// [ms]

/*-----------------------------------------------------------------------------
INCLUDE SECTION
-----------------------------------------------------------------------------*/
// FreeRTOS includes
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "sht11_drv_sensibus/sht11_drv_sensibus.h"

/*-----------------------------------------------------------------------------
GLOBAL VARIABLES SECTION
-----------------------------------------------------------------------------*/
SemaphoreHandle_t xMutex_SHT11_Data;

/*-----------------------------------------------------------------------------
HEADER SECTION
-----------------------------------------------------------------------------*/
void vSHT11_sample(void *pvParameters);

/*-----------------------------------------------------------------------------
IMPLEMENTATION SECTION
-----------------------------------------------------------------------------*/

/**
 * OS Task: SHT11 sample
 */
void vSHT11_sample(void *pvParameters)
{
	TickType_t pxPreviousWakeTime;
	xMutex_SHT11_Data = xSemaphoreCreateMutex();

	vTaskDelay(15); // max delay before startup
	SHT11_Init(SHT11_PORT);

	pxPreviousWakeTime = xTaskGetTickCount();
	do{
		if(xSemaphoreTake(xMutex_SHT11_Data, portMAX_DELAY) == pdTRUE){
			SHT11_StartTemperature();
			T_READ:
			vTaskDelay(_SHT11.delay);
			if(SHT11_ReadTemperature() > 0){
				SHT11_CalcTemp(_SHT11.raw_temperature);
				station.SHT11_data.temperature = _SHT11.temperature;
			}else{
				if(_SHT11.reset_cnt < 3){
					_SHT11.reset_cnt++;
//					uart_send_str_ln(USART1, "TMP_R_ERR");
					goto T_READ;
				}else{
					_SHT11.reset_cnt = 0;
					SHT11_ConnectionReset();
				}
			}

			SHT11_StartHumidity();
			H_READ:
			vTaskDelay(_SHT11.delay);
			if(SHT11_ReadHumidity() > 0){
				SHT11_CalcHumidity(_SHT11.raw_humidity, _SHT11.temperature);
				station.SHT11_data.humidity = _SHT11.humidity;
				station.SHT11_data.dewpoint = SHT11_CalcDewPoint(_SHT11.humidity, _SHT11.temperature);
			}else{
				if(_SHT11.reset_cnt < 3){
					_SHT11.reset_cnt++;
//					uart_send_str_ln(USART1, "HUM_R_ERR");
					goto H_READ;
				}else{
					_SHT11.reset_cnt = 0;
					SHT11_ConnectionReset();
				}
			}

			xSemaphoreGive(xMutex_SHT11_Data);
		}

		vTaskDelayUntil(&pxPreviousWakeTime, SHT11_SAMPLING_PERIOD);
	}while(1);
	vTaskDelete(NULL);
}

#endif /* VTASKSHT11_H_ */
