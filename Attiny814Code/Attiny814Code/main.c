#include <atmel_start.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <math.h>
#include "touch.h"
#include "touch_api_ptc.h"

#define EDGE_THRESDHOLD					30
#define FILTER_THRESDHOLD				20
#define FINGER_ON_MINIMUM_TIME_MS		70
#define FINGER_ON_MAXIMUM_TIME_MS		1000

typedef enum
{
	FINGER_ON_DETECT = 0,
	FINGER_OFF_DETECT,
}SensorStateDef;

typedef enum
{
	ON = 0,
	OFF,
}RadiotubeStateDef;

volatile SensorStateDef SensorState;
volatile RadiotubeStateDef RadiotubeState;

volatile uint8_t measeurePeriod = 32;

volatile uint16_t fingerOnCnt = 0;
uint8_t radiotubeCnt = 0;
uint32_t RadiotubeOnTime = 0;

extern volatile uint8_t measurement_done_touch;

volatile uint8_t edgeFreezeStart = 0;
volatile uint8_t edgeDetectFreeze = 0;
uint16_t edgeFreezeCnt = 0;

/* filter variable */
int16_t filteredDeltaValue = 0;
int16_t preFilteredDeltaValue = 0;
int16_t preDebugFilteredDeltaValue = 0;
int16_t debugFilteredDeltaValue = 0;

void Radiotube_Handle(void)
{
	if (RadiotubeState == OFF)
	{
		RadiotubeState = ON;
		IO1_set_level(true);
		_delay_ms(30);
		IO1_set_level(false);
		_delay_ms(10);
		measeurePeriod = 16;
		edgeDetectFreeze = 1;
	}
	else
	{
		RadiotubeState = OFF;
		IO2_set_level(true);
		_delay_ms(30);
		IO2_set_level(false);
		_delay_ms(10);
		measeurePeriod = 32;
		edgeDetectFreeze = 1;
	}
}

void MCU_GoToSleep(int mode)
{
	// Set sleep mode to Power Down mode
	set_sleep_mode(mode);
	sleep_enable();
	sleep_cpu();
	sleep_disable();
}

void LowBattery(void)
{
	IO1_set_level(true);
	_delay_ms(30);
	IO1_set_level(false);
	MCU_GoToSleep(SLEEP_MODE_PWR_DOWN);
}


void RTC_CallBack(void)
{
	/* count the time when the  finger on */
	if (SensorState == FINGER_OFF_DETECT)
		fingerOnCnt++;
	
	/* freeze the edge detection for 500 ms after open the radiotube */
	if(edgeDetectFreeze == 1)
		edgeFreezeCnt++;

	if (edgeFreezeCnt > 500)
	{
		edgeFreezeCnt = 0;
		edgeDetectFreeze = 0;
	}
	
	/* radiotube will close automatically 
		when it open more than 3 mins */
	if(RadiotubeState == ON)
	{
		RadiotubeOnTime++;
		if(RadiotubeOnTime > 180000)
		{
			RadiotubeOnTime = 0;
			Radiotube_Handle();
		}
	}
}


int16_t TOUCH_DeltaSmoothing(int16_t curDelta)
{
	uint16_t temp;
	
	preDebugFilteredDeltaValue = debugFilteredDeltaValue;
	
	if (abs(curDelta - debugFilteredDeltaValue) > FILTER_THRESDHOLD)
		debugFilteredDeltaValue = (debugFilteredDeltaValue * 1 + curDelta * 9)/10;
	else
		debugFilteredDeltaValue = (debugFilteredDeltaValue * 9 + curDelta)/10;
		
	temp = debugFilteredDeltaValue - preDebugFilteredDeltaValue;
	//temp = debugFilteredDeltaValue;
	
	if (edgeDetectFreeze == 1)
		temp = 0;
		
	return temp;
}

static uint8_t TOUCH_DeltaEdgeDetct(void)
{
	int16_t curDelta;
	int16_t tempDelta;
	uint8_t edgeStatus = 0;
	
	curDelta = get_sensor_node_signal(0);
	curDelta -= get_sensor_node_reference(0);
	
	tempDelta = abs(curDelta - filteredDeltaValue);
	
	preFilteredDeltaValue = filteredDeltaValue;
	
	if (tempDelta > FILTER_THRESDHOLD)
		filteredDeltaValue = (filteredDeltaValue * 1 + curDelta * 9)/10;
	else
		filteredDeltaValue = (filteredDeltaValue * 9 + curDelta)/10;
	
	tempDelta = abs(filteredDeltaValue - preFilteredDeltaValue);
	
	if(filteredDeltaValue > preFilteredDeltaValue)
	{
		if (tempDelta > EDGE_THRESDHOLD)
			edgeStatus = 1;
	}
	else if(filteredDeltaValue < preFilteredDeltaValue)
	{
		if (tempDelta > EDGE_THRESDHOLD)
			edgeStatus = 2;
	}
	return edgeStatus;
}

static uint8_t TOUCH_TouchDetect(void)
{
	uint8_t keyStatus = 0;
	uint8_t edgeStatus = 0;
	
	
	///* Does acquisition and post-processing */
	touch_process();
	
	
	if (measurement_done_touch == 0)
		return keyStatus;
		
	measurement_done_touch = 0;
	
	
		
	edgeStatus = TOUCH_DeltaEdgeDetct();
	
	if (edgeDetectFreeze == 1)
		return keyStatus;

	switch(SensorState)
	{
		case FINGER_ON_DETECT:
			if (edgeStatus == 1)
				SensorState = FINGER_OFF_DETECT;
		break;
		
		case FINGER_OFF_DETECT:
			if(edgeStatus == 2)
			{
				if(fingerOnCnt > FINGER_ON_MINIMUM_TIME_MS)
					keyStatus = 1;
					
				fingerOnCnt = 0;
				SensorState = FINGER_ON_DETECT;
			}
			else if(edgeStatus == 1)
				fingerOnCnt = 0;
			else if (fingerOnCnt > FINGER_ON_MAXIMUM_TIME_MS)
			{
				fingerOnCnt = 0;
				SensorState = FINGER_ON_DETECT;
			}
			break;
	}
	return keyStatus;
}

static void Radiotube_Test(void)
{
	while (1)
	{
		IO1_set_level(true);
		_delay_ms(30);
		IO1_set_level(false);
		_delay_ms(1000);
		IO2_set_level(true);
		_delay_ms(30);
		IO2_set_level(false);
		_delay_ms(1000);
	}
}



int main(void)
{
	
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
	
	SensorState = FINGER_ON_DETECT;
	
	/* the inital state of radiotube should be chosed */
	IO2_set_level(true);
	_delay_ms(30);
	IO2_set_level(false);
	
	RadiotubeState = OFF;
	//MCU_GoToSleep(SLEEP_MODE_PWR_DOWN);
	//Radiotube_Test();
	
	/* Replace with your application code */
	while(1) 
	{
		//wdt_reset();
		
		if(TOUCH_TouchDetect() == 1)
			Radiotube_Handle();
	
		//MCU_GoToSleep(SLEEP_MODE_STANDBY);
		MCU_GoToSleep(SLEEP_MODE_IDLE);
	}
}


