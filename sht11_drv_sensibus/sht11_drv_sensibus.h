/*
 * SHT11_drv_sensibus.h
 *
 *  Created on: Feb 5, 2018
 *      Author: Dr. Saldon
 */

#ifndef SHT11_DRV_SENSIBUS_H_
#define SHT11_DRV_SENSIBUS_H_

/*-----------------------------------------------------------------------------
INCLUDE SECTION
-----------------------------------------------------------------------------*/
// General STM32 includes
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

/*-----------------------------------------------------------------------------
MACRO SECTION
-----------------------------------------------------------------------------*/
#define SHT11_PORT 				GPIOB
#define SHT11_DATA_PIN 			GPIO_Pin_11
#define SHT11_SCK_PIN 			GPIO_Pin_10

#define SHT11_DATA_IO 			SHT11_PORT, SHT11_DATA_PIN
#define SHT11_SCK_IO 			SHT11_PORT, SHT11_SCK_PIN

#define SHT11_SET_DATA_HIGH() 	{GPIO_SetBits	(SHT11_DATA_IO);}
#define SHT11_SET_DATA_LOW()	{GPIO_ResetBits	(SHT11_DATA_IO);}
#define SHT11_SET_SCK_HIGH() 	{GPIO_SetBits	(SHT11_SCK_IO);}
#define SHT11_SET_SCK_LOW()	 	{GPIO_ResetBits	(SHT11_SCK_IO);}

//adr  command  r/w
#define STATUS_REG_W  0x06   //000   0011    0
#define STATUS_REG_R  0x07   //000   0011    1
#define MEASURE_TEMP  0x03   //000   0001    1
#define MEASURE_HUMI  0x05   //000   0010    1
#define RESETa        0x1e   //000   1111    0

// SHT11 Measurment delays
#define SHT11_M_DELAY_MS_SHORT		20
#define SHT11_M_DELAY_MS_MEDIUM		80
#define SHT11_M_DELAY_MS_LONG		320

#define SHT11_M_DELAY_8_BIT			SHT11_M_DELAY_MS_SHORT
#define SHT11_M_DELAY_12_BIT		SHT11_M_DELAY_MS_MEDIUM
#define SHT11_M_DELAY_14_BIT		SHT11_M_DELAY_MS_LONG

// SHT11 Measurment resolution
#define SHT11_M_RES_LOW 			0	// Status Register Bits - 0bxxxxxxx1
#define SHT11_M_RES_HIGH 			1	// Status Register Bits - 0bxxxxxxx0

// Temperature & humidity equation constants
#define C1		(float)(-2.0468)        	// for 3V sensors
#define C2h   	(float)(0.0367)        		// for 3V sensors, 12-bit precision
#define C3h  	(float)(-0.0000015955)     	// for 3V sensors, 12-bit precision
#define C2l   	(float)(0.5872)        		// for 3V sensors, 8-bit precision
#define C3l  	(float)(-0.00040845)     	// for 3V sensors, 8-bit precision

#define T1    	(float)(-39.6)        		// for 3V sensors
#define T2h   	(float)(0.01)       		// for 3V sensors, 12-bit precision
#define T2l   	(float)(0.04)       		// for 3V sensors, 8-bit precision

#define NACK 	0
#define ACK     1

/*-----------------------------------------------------------------------------
GLOBAL VARIABLES SECTION
-----------------------------------------------------------------------------*/

typedef struct {
	uint16_t raw_temperature;		// temperature readout
	uint16_t raw_humidity;			// humidity readout
	float temperature;				// true temperature
	float humidity;					// true humidity
	float dewpoint;					// true dew point
	uint8_t checksum;				// checksum (received)
	uint8_t resolution;				// measurment resolution
	uint16_t delay; 				// measurment delay [ms]
	uint8_t reset_cnt;				// sensor lack counter
} SHT11_t;

SHT11_t _SHT11;

/*-----------------------------------------------------------------------------
HEADER SECTION
-----------------------------------------------------------------------------*/
void nop_delay(uint8_t ticks);
void SHT11_SetDataIn(void);
void SHT11_SetDataOut(void);
void SHT11_SetSckOut(void);
void SHT11_TransmissionStart(void);
void SHT11_ConnectionReset(void);
uint8_t SHT11_ReadByte(uint8_t ack);
uint8_t SHT11_WriteByte(uint8_t value);
uint8_t SHT11_SoftReset(void);
uint8_t SHT11_ReadSR(void);
uint8_t SHT11_WriteSR(uint8_t value);
void SHT11_Init(GPIO_TypeDef * port);
uint8_t SHT11_StartTemperature(void);
uint8_t SHT11_StartHumidity(void);
uint16_t SHT11_ReadTemperature(void);
uint16_t SHT11_ReadHumidity(void);
float SHT11_CalcTemp(uint16_t raw_temperature);
float SHT11_CalcHumidity(uint16_t raw_humidity, float temperature);
float SHT11_CalcDewPoint(float humidity, float temperature);

#endif /* SHT11_DRV_SENSIBUS_H_*/
