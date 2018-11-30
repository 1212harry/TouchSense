#include <atmel_start.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <math.h>
#include "touch.h"
#include "touch_api_ptc.h"

uint8_t EDGE_THRESDHOLD = 70;
uint8_t FILTER_THRESDHOLD = 20;
#define STRONG_EDGE_THRESHOLD			70
#define WEAK_EDGE_THRESHOLD				20
#define FINGER_ON_MINIMUM_TIME_MS		2
#define FINGER_ON_MAXIMUM_TIME_MS		31

//#define SNR_CAL_CNT    20

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

volatile SensorStateDef SensorState = FINGER_ON_DETECT;
volatile RadiotubeStateDef RadiotubeState = OFF;

volatile uint8_t measeurePeriod = 1;

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

volatile uint8_t measureBusyFlag = 0;

void TOUCH_SetMeasureBusyFlag(void)
{
	measureBusyFlag = 1;
}

void Radiotube_Handle(void)
{
	if (RadiotubeState == OFF)
	{
		RadiotubeState = ON;
		IO1_set_level(true);
		_delay_ms(30);
		IO1_set_level(false);
		_delay_ms(10);
		measeurePeriod = 1;
		edgeDetectFreeze = 1;
	}
	else
	{
		RadiotubeState = OFF;
		IO2_set_level(true);
		_delay_ms(30);
		IO2_set_level(false);
		_delay_ms(10);
		measeurePeriod = 1;
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
	while (1)
	{
	}
}


void RTC_CallBack(void)
{
	/* count the time when the  finger on */
	if (SensorState == FINGER_OFF_DETECT)
		fingerOnCnt++;
	
	/* freeze the edge detection for 500 ms after open the radiotube */
	if(edgeDetectFreeze == 1)
		edgeFreezeCnt++;

	if (edgeFreezeCnt > 16)
	{
		edgeFreezeCnt = 0;
		edgeDetectFreeze = 0;
	}
	
	/* radiotube will close automatically 
		when it open more than 3 mins */
	if(RadiotubeState == ON)
	{
		RadiotubeOnTime++;
		if(RadiotubeOnTime > 5625)
		{
			RadiotubeOnTime = 0;
			Radiotube_Handle();
		}
	}
}


int16_t TOUCH_DeltaSmoothing(int16_t curDelta)
{
	int16_t tempDelta;
	
	tempDelta = curDelta - preDebugFilteredDeltaValue;
	
	if (abs(tempDelta) >= STRONG_EDGE_THRESHOLD)
	{
		/* this is an strong edge */
		debugFilteredDeltaValue = curDelta;
	}
	else if (abs(tempDelta) >= WEAK_EDGE_THRESHOLD)
	{
		/* this is an weak edge */
		debugFilteredDeltaValue = (preDebugFilteredDeltaValue * 5 + curDelta * 5)/10;
	}
	else
	{
		/* this should be suppressed */
		debugFilteredDeltaValue = (preDebugFilteredDeltaValue * 9 + curDelta)/10;
	}
	
	preDebugFilteredDeltaValue = debugFilteredDeltaValue;
	
	if (edgeDetectFreeze == 1)
		tempDelta = 0;
		
	return tempDelta;
}
int16_t edgeGroup[4];
uint8_t edgeGroupPtr = 0;

static void TOUCH_ClearEdgeGroup(void)
{
	uint8_t i;
	
	edgeGroupPtr = 0;
	for (i = 0; i < 4; i++)
		edgeGroup[edgeGroupPtr] = 0;
}

static uint8_t TOUCH_DeltaEdgeDetct(void)
{
	int16_t curDelta;
	int16_t tempDelta;
	uint8_t edgeStatus = 0,i;
	int16_t edgeGroupSum = 0;
	
	curDelta = get_sensor_node_signal(0);
	curDelta -= get_sensor_node_reference(0);
	
	tempDelta = curDelta - preFilteredDeltaValue;
	
	if (abs(tempDelta) >= STRONG_EDGE_THRESHOLD)
	{
		/* this is an strong edge */
		if(tempDelta > 0)
			edgeStatus = 1;
		else 
			edgeStatus = 2;
		
		filteredDeltaValue = curDelta;
		TOUCH_ClearEdgeGroup();
	}
	else if (abs(tempDelta) >= WEAK_EDGE_THRESHOLD)
	{
		/* this is an weak edge */
		
		filteredDeltaValue = (preFilteredDeltaValue * 5 + curDelta * 5)/10;
		
		/* if the sum of continuous four edges exceed the 
			strong threshold, an effective status should
			be return.*/
		edgeGroup[edgeGroupPtr++] = tempDelta;
		if (edgeGroupPtr >= 4)
			edgeGroupPtr = 0;
		
		for (i = 0; i < 4; i++)
			edgeGroupSum += edgeGroup[i];
			
		if (abs(edgeGroupSum) >= STRONG_EDGE_THRESHOLD)
		{
			TOUCH_ClearEdgeGroup();
			
			if (edgeGroupSum > 0)
				edgeStatus = 1; 
			else
				edgeStatus = 2;
		}
	}
	else
	{
		/* this should be suppressed */
		filteredDeltaValue = (preFilteredDeltaValue * 9 + curDelta)/10;
		
		edgeGroup[edgeGroupPtr++] = 0;
		if (edgeGroupPtr >= 4)
			edgeGroupPtr = 0;
	}
	
	preFilteredDeltaValue = filteredDeltaValue;
	
	return edgeStatus;
}

//int16_t signalTouched[SNR_CAL_CNT];
//int16_t signalUntouched[SNR_CAL_CNT];
//int16_t signalRaw[SNR_CAL_CNT];
//uint8_t signalPtr = 0;
//uint8_t signalTouchedPtr = 0;
//uint8_t signalUntouchedPtr = 0;
//
//int16_t TOUCH_SNR(void)
//{
	//int32_t temp = 0;
	//int16_t SNR,touchStrength, noise;
	//uint16_t signalTouchAvg = 0, signalUntouchAvg = 0;
	//uint8_t i;
//
	//signalRaw[signalPtr] = get_sensor_node_signal(0);
	//signalRaw[signalPtr] -= get_sensor_node_reference(0);
	//
	//if (signalRaw[signalPtr] >= EDGE_THRESDHOLD)
	//{
		//signalTouched[signalTouchedPtr++] = signalRaw[signalPtr];
		//if(signalTouchedPtr >= SNR_CAL_CNT)
			//signalTouchedPtr = 0;
	//}
	//else
	//{
		//signalUntouched[signalUntouchedPtr++] = signalRaw[signalPtr];
		//if(signalUntouchedPtr >= SNR_CAL_CNT)
			//signalUntouchedPtr = 0;
	//}
	//
	//signalPtr++;
	//if (signalPtr >= SNR_CAL_CNT)
		//signalPtr = 0;
	//
	//for (i = 0; i < SNR_CAL_CNT; i++)
	//{
		//signalTouchAvg += signalTouched[i];
		//signalUntouchAvg += signalUntouched[i];
	//}
	//
	//signalTouchAvg = signalTouchAvg/SNR_CAL_CNT;
	//signalUntouchAvg = signalUntouchAvg/SNR_CAL_CNT;
	//
	//touchStrength = signalTouchAvg - signalUntouchAvg;
	//
	//for (i = 0; i < SNR_CAL_CNT; i++)
		//temp += (signalUntouched[i] - signalUntouchAvg) * (signalUntouched[i] - signalUntouchAvg);
	//
	//
	//noise = sqrt((double)(temp/SNR_CAL_CNT));
	//
	//if(noise == 0)
		//noise = 1;
		//
	//SNR = touchStrength/noise;
	////SNR = touchStrength;
	//return SNR;
//}

static uint8_t TOUCH_TouchDetect(void)
{
	uint8_t keyStatus = 0;
	uint8_t edgeStatus = 0;
	
	
	///* Does acquisition and post-processing */
	touch_process();
	
	if (measurement_done_touch == 0)
		return keyStatus;
		
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
	
	/* one cycle of measurement is done */
	measurement_done_touch = 0;
	measureBusyFlag = 0;
	return keyStatus;
}

//static void Radiotube_Test(void)
//{
	//while (1)
	//{
		//IO1_set_level(true);
		//_delay_ms(30);
		//IO1_set_level(false);
		//_delay_ms(1000);
		//IO2_set_level(true);
		//_delay_ms(30);
		//IO2_set_level(false);
		//_delay_ms(1000);
	//}
//}


int main(void)
{
	
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
	
	/* the inital state of radiotube should be closed */
	IO2_set_level(true);
	_delay_ms(30);
	IO2_set_level(false);
	
	//Radiotube_Test();
	
	/* Replace with your application code */
	while(1) 
	{
		wdt_reset();
		
		if(TOUCH_TouchDetect() == 1)
			Radiotube_Handle();
		
		if (measureBusyFlag == 0)
			MCU_GoToSleep(SLEEP_MODE_PWR_DOWN);
		
			
		//MCU_GoToSleep(SLEEP_MODE_IDLE);
	}
}


