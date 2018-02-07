/*
 * sht11_drv_sensibus.c
 *
 *  Created on: Feb 5, 2018
 *      Author: Dr. Saldon
 */

/*-----------------------------------------------------------------------------
MACRO SECTION
-----------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------
INCLUDE SECTION
-----------------------------------------------------------------------------*/
#include <math.h>
#include "sht11_drv_sensibus/sht11_drv_sensibus.h"

/*-----------------------------------------------------------------------------
GLOBAL VARIABLES SECTION
-----------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------
HEADER SECTION
-----------------------------------------------------------------------------*/


/*-----------------------------------------------------------------------------
IMPLEMENTATION SECTION
-----------------------------------------------------------------------------*/

/**
 * Damned delay
 *
 * input:
 * 		us - delay microseconds
 */
void nop_delay(uint8_t us)
{
    for(uint16_t i = (us*72); i > 0; i--)  // us * (SYSTEM_CORE_CLOCK / 1 000 000)
    {
    	asm("nop");
    }
}

/**
 * SHT11_SetDataIn
 *
 * Set the operating mode for the SHT11_DATA_PIN as input
 */
void SHT11_SetDataIn(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = SHT11_DATA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SHT11_PORT, &GPIO_InitStructure);
}

/**
 * SHT11_SetDataOut
 *
 * Set the operating mode for the SHT11_DATA_PIN as output
 */
void SHT11_SetDataOut(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = SHT11_DATA_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(SHT11_PORT, &GPIO_InitStructure);
}

/**
 * SHT11_SetSckOut
 *
 * Set the operating mode for the SHT11_SCK_PIN as output
 */
void SHT11_SetSckOut(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = SHT11_SCK_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(SHT11_PORT, &GPIO_InitStructure);
}

/**
 * SHT11_TransmissionStart
 *
 * Emit "Transmission Start" (TS) sequence
 */
void SHT11_TransmissionStart(void)
{
	SHT11_SET_SCK_LOW(); 				//SCK=0;                   //Initial state
	SHT11_SetDataIn();					// DATA_TRIS = 1;   //pullup resistor brings DATA pin high
	SHT11_SET_DATA_HIGH();
	SHT11_SetDataOut();					//DATA_TRIS=0;
	nop_delay(1);
	SHT11_SET_SCK_HIGH(); 				//SCK=1;
	nop_delay(1);
	SHT11_SET_DATA_LOW();    			// DATA_WR=0; DATA_TRIS=0;
	nop_delay(1);
	SHT11_SET_SCK_LOW();				//SCK=0;
	nop_delay(5);
	SHT11_SET_SCK_HIGH();				//SCK=1;
	nop_delay(1);
	SHT11_SET_DATA_HIGH();
	nop_delay(1);
	SHT11_SET_SCK_LOW();				//SCK=0;
	SHT11_SetDataIn();
}

/**
 * SHT11_ConnectionReset
 *
 * Reset connection if communication with the SHT11 is lost.
 * This sequence resets the communication interface only. The status
 * register preserves its content.
 */
void SHT11_ConnectionReset(void)
{
	SHT11_SetDataOut();     			//DATA_TRIS  = 0;    //set data pin an output
	SHT11_SET_DATA_HIGH();    			//DATA_WR     = 1;   //set data pin high
	SHT11_SetSckOut();      			//SCK_TRIS = 0;    	 //set CLK pin an output low
	SHT11_SET_SCK_LOW();   				// SCK   = 0;

	for(uint8_t i = 0; i < 9; i++) 		//9 SCK cycles for connection reset sequence
	{
		SHT11_SET_SCK_HIGH();   		//SCK=1;
		nop_delay(1);
		SHT11_SET_SCK_LOW();  			//SCK=0;
		nop_delay(1);
	}
	SHT11_TransmissionStart();
}

/**
 * SHT11_ReadByte
 *
 * Read byte from sensor and send acknowledge if "ack" is true
 *
 * input:
 * 		ack - "acknowledge" send:
 * 			ACK = 1 - send "ack"
 * 			NACK = 0 - skip
 * return:
 * 		result - received byte
 */
uint8_t SHT11_ReadByte(uint8_t ack)
{
	uint8_t result = 0;
	SHT11_SetDataIn(); 					//DATA_TRIS = 1; 		//set DATA line an input
	SHT11_SET_SCK_LOW();				//SCK = 0;
	for (uint8_t i = 8; i > 0; i--)     //shift bit for masking
	{
		result <<= 1;
		SHT11_SET_SCK_HIGH();			//SCK=1;   				//clk for SENSI-BUS
		nop_delay(2);
		result |= GPIO_ReadInputDataBit(SHT11_PORT, SHT11_DATA_PIN);  //read bit
		SHT11_SET_SCK_LOW();			//SCK=0;
		nop_delay(2);
	}

	if(ack == ACK)
	{
		// Pull DATA line low for "ack"
		SHT11_SetDataOut();				// DATA_TRIS = 0;
		SHT11_SET_DATA_LOW();			//DATA_WR = 0;
		nop_delay(2);
		SHT11_SET_SCK_HIGH();			//SCK=1;
		nop_delay(5);      				//pulse-width approx. 5 us
		SHT11_SET_SCK_LOW();			//SCK=0;
		nop_delay(2);
		SHT11_SetDataIn();				// DATA_TRIS = 1;       //release DATA-line
	}else{
		// Pull DATA line high for skiping "ack"
		SHT11_SetDataOut();				// DATA_TRIS = 0;
		SHT11_SET_DATA_HIGH();			//DATA_WR = 0;
		nop_delay(2);
		SHT11_SET_SCK_HIGH();			//SCK=1;
		nop_delay(5);      				//pulse-width approx. 5 us
		SHT11_SET_SCK_LOW();			//SCK=0;
		nop_delay(2);
		SHT11_SetDataIn();				// DATA_TRIS = 1;      //release DATA-line
	}
    return result;
}

/**
 * SHT11_WriteByte
 *
 * Write byte to sensor and check for acknowledge
 *
 * input:
 * 		value - byte to send
 * return:
 * 		ACK = 1 	- proper reception of the value
 *		NACK = 0 	- reception failure
 */
uint8_t SHT11_WriteByte(uint8_t value)
{
	uint8_t ack = 0;
	uint8_t mask = 0x80;

	SHT11_SetDataOut();					//DATA_TRIS = 0;
	for (uint8_t i = 8; i > 0; i--)
	{
		if (value & mask){
			SHT11_SET_DATA_HIGH();		// DATA_WR=1;     //masking value with mask , write to SENSI-BUS
		}else{
			SHT11_SET_DATA_LOW(); 		//DATA_WR=0;
		}
		nop_delay(2);
		SHT11_SET_SCK_HIGH();			//SCK=1;          //clk for SENSI-BUS
		nop_delay(5);            		//pulse-width approx. 5 us
		SHT11_SET_SCK_LOW();			//SCK=0;
		nop_delay(1);
		mask >>= 1;                     // Shift mask for next data bit
	}
	SHT11_SetDataIn();					//DATA_TRIS=1;    //release DATA-line, let SHT11 sensor controls DATA line

	SHT11_SET_SCK_HIGH();				//SCK=1;
	nop_delay(5);						//clk #9 for ack

	if(GPIO_ReadInputDataBit(SHT11_PORT, SHT11_DATA_PIN) == 0){
		ack = ACK;
	}
	SHT11_SET_SCK_LOW();				//SCK=0;

	return ack;                        	//error=0 in case of no acknowledge
}

/**
 * SHT11_SoftReset
 *
 * Do Software reset (set STATUS_REGISTER to its default value)
 *
 * return:
 * 		ACK = 1 	- proper reception of the command
 *		NACK = 0 	- command reception failure
 */
uint8_t SHT11_SoftReset(void)
{
  SHT11_ConnectionReset();				//reset communication
  if(SHT11_WriteByte(RESETa)){			//send RESET-command to sensor
	  return ACK;
  }
  return NACK;                       		//error=0 in case of no response from the sensor
}

/**
 * SHT11_ReadSR
 *
 * Read status register
 *
 * return:
 * 		_temp_regval - status register value
 *		NACK = 0 	- command reception failure
 */
uint8_t SHT11_ReadSR(void)
{
	uint8_t _temp_regval = 0;
	if(SHT11_WriteByte(STATUS_REG_R)){
		_temp_regval = SHT11_ReadByte(ACK);
		if(_temp_regval > 0){
			_SHT11.checksum = SHT11_ReadByte(ACK);
			return _temp_regval;
		}
	}
	return NACK;
}

/**
 * SHT11_WriteSR
 *
 * Write status register
 *
 * return:
 * 		ACK = 1 	- proper reception of the command
 *		NACK = 0 	- command reception failure
 */
uint8_t SHT11_WriteSR(uint8_t value)
{
	if(SHT11_WriteByte(STATUS_REG_W)){
		if(SHT11_WriteByte(value)){
			return ACK;
		}
	}
	return NACK;
}

/**
 * SHT11_Init
 *
 * Initialize sensor interface
 */
void SHT11_Init(GPIO_TypeDef * port)
{
	uint16_t _temp;

	if(port == GPIOA){
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	}
	if(port == GPIOB){
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	}
	if(port == GPIOC){
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	}

	SHT11_SetSckOut();
	SHT11_SetDataIn();
	_SHT11.delay = SHT11_M_DELAY_14_BIT;
	_SHT11.resolution = SHT11_M_RES_HIGH;
	_SHT11.raw_temperature = 0;
	_SHT11.raw_humidity = 0;
	_SHT11.checksum = 0;

	_temp = SHT11_ReadSR();
	if(_temp && 0b10000000){
		// low resolution
		_SHT11.resolution = SHT11_M_RES_LOW;
	}else{
		// high resolution
		_SHT11.resolution = SHT11_M_RES_HIGH;
	}
}

/**
 * SHT11_StartTemperature
 *
 * Start temperature measurment and set appropriate delay
 *
 * return:
 * 		ACK = 1 	- proper reception of the command
 *		NACK = 0 	- command reception failure
 */
uint8_t SHT11_StartTemperature(void)
{
	_SHT11.reset_cnt = 0;
	SHT11_TransmissionStart();
	if(SHT11_WriteByte(MEASURE_TEMP)){
		_SHT11.delay = SHT11_M_DELAY_14_BIT;
		return ACK;
	}
	return NACK;
}

/**
 * SHT11_StartHumidity
 *
 * Start humidity measurment and set appropriate delay
 *
 * return:
 * 		ACK = 1 	- proper reception of the command
 *		NACK = 0 	- command reception failure
 */
uint8_t SHT11_StartHumidity(void)
{
	_SHT11.reset_cnt = 0;
	SHT11_TransmissionStart();
	if(SHT11_WriteByte(MEASURE_HUMI)){
		_SHT11.delay = SHT11_M_DELAY_12_BIT;
		return ACK;
	}
	return NACK;
}

/**
 * SHT11_ReadTemperature
 *
 * return:
 * 		raw_temperature	- in case of successful data reception
 *		zero - in case of data reception failure
 */
uint16_t SHT11_ReadTemperature(void)
{
	uint16_t _msb;
	uint16_t _lsb;
	uint16_t _temp;

	if(GPIO_ReadInputDataBit(SHT11_PORT, SHT11_DATA_PIN) == 0){
		_msb = SHT11_ReadByte(ACK);     //read the first byte (MSB)
		_lsb = SHT11_ReadByte(ACK);      //read the second byte (LSB)
		_SHT11.checksum  = SHT11_ReadByte(ACK);    //read checksum
		_temp = (_msb << 8) | (_lsb);
		_SHT11.raw_temperature = _temp;
		return _temp;
	}
	return 0;
}

/**
 * SHT11_ReadHumidity
 *
 * return:
 * 		raw_humidity - in case of successful data reception
 *		zero - in case of data reception failure
 */
uint16_t SHT11_ReadHumidity(void)
{
	uint16_t _msb;
	uint16_t _lsb;
	uint16_t _temp;

	if(GPIO_ReadInputDataBit(SHT11_PORT, SHT11_DATA_PIN) == 0){
		_msb = SHT11_ReadByte(ACK);     //read the first byte (MSB)
		_lsb = SHT11_ReadByte(ACK);      //read the second byte (LSB)
		_SHT11.checksum  = SHT11_ReadByte(ACK);    //read checksum
		_temp = (_msb << 8) | (_lsb);
		_SHT11.raw_humidity= _temp;
		return _temp;
	}
	return 0;
}

/**
 * SHT11_CalcTemp
 *
 * Calculates temperature in degC from raw sensor data.
 * Should be called after SHT11_ReadTemperature()
 *
 * input:
 * 		raw_temperature - result of measurment; _SHT11.raw_temperature can be passed
 * return:
 * 		temperature - calculated real temperature in degC
 */
float SHT11_CalcTemp(uint16_t raw_temperature)
{
	float _temp_t;
	if (_SHT11.resolution == SHT11_M_RES_LOW){
		_temp_t = T1 + T2l * (float)raw_temperature;
	}else{
		_temp_t = T1 + T2h * (float)raw_temperature;
	}
	_SHT11.temperature = _temp_t;
	return _temp_t;
}

/**
 * SHT11_CalcHumidity
 *
 * Calculates relative humidity from raw sensor data
 * (with temperature compensation)
 *
 * input:
 * 		raw_humidity - result of measurment; _SHT11.raw_humidity can be passed
 * 		temperature - calculated real temperature in degC; _SHT11.temperature can be passed
 * return:
 * 		humidity - calculated real humidity in %
 */
float SHT11_CalcHumidity(uint16_t raw_humidity, float temperature)
{
	float _temp_h;
	if (_SHT11.resolution == SHT11_M_RES_LOW){
		_temp_h = C1 + C2l * raw_humidity + C3l * raw_humidity * raw_humidity;
		if(temperature > 50){
			_temp_h = (temperature - 25.0) * (T1 + T2l * raw_humidity) + _temp_h;
		}
	}else{
		_temp_h = C1 + C2h * raw_humidity + C3h * raw_humidity * raw_humidity;
		if(temperature > 50){
			_temp_h = (temperature - 25.0) * (T1 + T2h * raw_humidity) + _temp_h;
		}
	}

//	if (_temp_h > 100.0) _temp_h = 100.0;		// Uncomment if you want to limit output values with their phisical ranges
//	if (_temp_h < 0.1) _temp_h = 0.1;
	_SHT11.humidity = _temp_h;
	return _temp_h;
}

/**
 * SHT11_CalcDewPoint
 *
 * Calculates dew point in degC
 *
 * input:
 * 		humidity - calculated real humidity in %; _SHT11.humidity can be passed
 * 		temperature - calculated real temperature in degC; _SHT11.temperature can be passed
 * return:
 * 		dewpoint - calculated real dew point temperature in degC
 */
float SHT11_CalcDewPoint(float humidity, float temperature)
{
  float _temp_k;
  _temp_k = log(humidity/100) + (17.62 * temperature) / (243.12 + temperature);
  _temp_k = 243.12 * _temp_k / (17.62 - _temp_k);
  _SHT11.dewpoint = _temp_k;
  return _temp_k;
}
