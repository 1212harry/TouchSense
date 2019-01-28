#include <atmel_start.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <math.h>
#include "touch.h"
#include "touch_api_ptc.h"
#include "driver_init.h"

#define EDGE_NONE				0
#define EDGE_RISING				1
#define EDGE_FALLING			2

#define RTC_WAKE_UP_TIME							32
#define FINGER_ON_MINIMUM_TIME_MS(TIME)				(uint16_t)(TIME/RTC_WAKE_UP_TIME)
#define FINGER_ON_MAXIMUM_TIME_MS(TIME)				(uint16_t)(TIME/RTC_WAKE_UP_TIME)
#define RADIOTUBE_FREEZE_TIME_MS(TIME)				(uint16_t)(TIME/RTC_WAKE_UP_TIME)		
#define RADIOTUBE_AUTO_CLOSE_TIME_MIN(TIME)			(uint32_t)((TIME * 60000)/RTC_WAKE_UP_TIME)
#define AC_CHECK_TIME_MS(TIME)						(uint16_t)(TIME/RTC_WAKE_UP_TIME)	

uint16_t STRONG_EDGE_THRESHOLD = 50;
uint16_t noiseTolerance = 25;
uint8_t noiseCnt = 0;

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

volatile uint8_t measeurePeriod = RTC_WAKE_UP_TIME;

volatile uint16_t fingerOnCnt = 0;
uint8_t radiotubeCnt = 0;
uint32_t RadiotubeOnTime = 0;

extern volatile uint8_t measurement_done_touch;
volatile uint8_t measureBusyFlag = 0;

volatile uint8_t edgeFreezeStart = 0;
volatile uint8_t edgeDetectFreeze = 0;
uint16_t edgeFreezeCnt = 0;

/* filter variable */
int16_t filteredDeltaValue = 0;

uint8_t lowBatteryWarming = 0;
uint16_t AC_TimeCnt = 0;

int16_t TOUCH_GetTouchSignal(void)
{
	return STRONG_EDGE_THRESHOLD;
}

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
		edgeDetectFreeze = 1;
		
		if (lowBatteryWarming == 1)
		{
			while (1)
			{
				wdt_reset();
			}
		}
	}
	else
	{
		RadiotubeState = OFF;
		IO2_set_level(true);
		_delay_ms(30);
		IO2_set_level(false);
		edgeDetectFreeze = 1;
		RadiotubeOnTime = 0;
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


void RTC_CallBack(void)
{
	/* monitor the battery charge every second */
	/* if the charge of battery below 1.5v, go to the low battery mode */
	AC_TimeCnt++;
	if (AC_TimeCnt >= AC_CHECK_TIME_MS(1000) && lowBatteryWarming == 0)
	{
		AC_TimeCnt = 0;
		PA6_set_level(true);
		_delay_ms(2);
		AC_0_init();
		_delay_ms(2);
		
		if ((AC0.STATUS & AC_STATE_bm) == 0)
			lowBatteryWarming = 1;
		
		AC_0_Disable();
		PA6_set_level(false);
	}
	
	/* count the time when the  finger on */
	if (SensorState == FINGER_OFF_DETECT)
		fingerOnCnt++; 
	
	/* freeze the edge detection for 100 ms after open the radiotube */
	if(edgeDetectFreeze == 1)
		edgeFreezeCnt++;

	if (edgeFreezeCnt > RADIOTUBE_FREEZE_TIME_MS(100))
	{
		edgeFreezeCnt = 0;
		edgeDetectFreeze = 0;
	}
	
	/* radiotube will close automatically 
		when it open more than 3 mins */
	if(RadiotubeState == ON)
	{
		RadiotubeOnTime++;
		if(RadiotubeOnTime > RADIOTUBE_AUTO_CLOSE_TIME_MIN(3))
		{
			RadiotubeOnTime = 0;
			Radiotube_Handle();
		}
	}
}


int16_t TOUCH_DeltaSmoothing(int16_t curDelta)
{		
	if (edgeDetectFreeze == 1)
		return 0;
	else
		return abs(curDelta - filteredDeltaValue);
}

static uint8_t TOUCH_DeltaEdgeDetct(void)
{
	int16_t curDelta;
	int16_t deltaDerivativeAbs,deltaDerivative;
	uint8_t edgeStatus = EDGE_NONE;
	
	curDelta = get_sensor_node_signal(0);
	curDelta -= get_sensor_node_reference(0);
	
	deltaDerivative = curDelta - filteredDeltaValue;
	deltaDerivativeAbs = abs(deltaDerivative);
	filteredDeltaValue = curDelta;
	
	if (deltaDerivativeAbs >= STRONG_EDGE_THRESHOLD)
	{
		/* this is an strong edge */
		if(deltaDerivative > 0)
			edgeStatus = EDGE_RISING;
		else 
			edgeStatus = EDGE_FALLING;
	}
	else if (deltaDerivativeAbs >= noiseTolerance)
	{
		/* if the amplitude of noise exceed the noise tolerance,
			the edge threshold should go up.*/
		STRONG_EDGE_THRESHOLD++;
		noiseCnt = 0;
	}
	else
	{
		/* if the fluctuation of noise within the noise tolerance for 3 second,
			the edge threshold should go down.*/
		noiseCnt++;
		if (noiseCnt >= 100)
		{
			STRONG_EDGE_THRESHOLD--;
			noiseCnt = 0;
		}
	}
	
	if (STRONG_EDGE_THRESHOLD >= 80)
		STRONG_EDGE_THRESHOLD = 80;
	else if (STRONG_EDGE_THRESHOLD <= 35)
		STRONG_EDGE_THRESHOLD = 35;
	
	noiseTolerance = STRONG_EDGE_THRESHOLD/2;
	
	return edgeStatus;
}

static uint8_t TOUCH_TouchDetect(void)
{
	uint8_t keyStatus = 0;
	uint8_t edgeStatus = EDGE_NONE;
	
	///* Does acquisition and post-processing */
	touch_process();
	
	if (measurement_done_touch == 0)
		return keyStatus;
		
	if (edgeDetectFreeze == 1)
		return keyStatus;
		
	edgeStatus = TOUCH_DeltaEdgeDetct();
	
	switch(SensorState)
	{
		case FINGER_ON_DETECT:
			if (edgeStatus == EDGE_RISING)
				SensorState = FINGER_OFF_DETECT;
		break;
		
		case FINGER_OFF_DETECT:
			/* state will roll back if rising edge appears. */
			if (edgeStatus == EDGE_RISING)
				fingerOnCnt = 0;
			/* the time duration of effective touch should between 70ms to 500ms */
			else if (fingerOnCnt >= FINGER_ON_MAXIMUM_TIME_MS(500))
			{
				fingerOnCnt = 0;
				SensorState = FINGER_ON_DETECT;
			}
			else if (edgeStatus == EDGE_FALLING)
			{
				if (fingerOnCnt >= FINGER_ON_MINIMUM_TIME_MS(70))
					keyStatus = 1;
					
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
		
	//Radiotube_Test();
	
	/* Replace with your application code */
	while(1) 
	{
		wdt_reset();
		
		if(TOUCH_TouchDetect() == 1)
			Radiotube_Handle();
		
		if (measureBusyFlag == 0)
		{
#ifdef _DEBUG
			MCU_GoToSleep(SLEEP_MODE_IDLE);
#else
			MCU_GoToSleep(SLEEP_MODE_PWR_DOWN);
#endif
		}
	}
}


