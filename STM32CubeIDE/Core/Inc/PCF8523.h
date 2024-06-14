/*
 * PCF8563.h
 *
 *	The MIT License.
 *  Created on: 5.09.2019
 *      Author: Mateusz Salamon
 *		Contact: mateusz@msalamon.pl
 *
 *      Website: https://msalamon.pl/dalsze-zmagania-z-rtc-ds1307-i-pcf8563-na-stm32/
 *      GitHub: https://github.com/lamik/PCF8563_RTC_STM32_HAL
 */

#ifndef _PCF8523_H_
#define _PCF8523_H_

#ifdef __cplusplus
extern "C" {
#endif
// comment this out if you already have a JeeLabs' DateTime class in your code
// or if you don't need DateTime functionality
#define PCF8523_INCLUDE_DATETIME_CLASS

// comment this out if you don't need DateTime functionality
#define PCF8523_INCLUDE_DATETIME_METHODS


#define PCF8523_ADDRESS				    (0x68<<1)
#define PCF8523_I2C_TIMEOUT				100

#define PCF8523_CONTROL_1			0x00
#define PCF8523_CONTROL_2			0x01
#define PCF8523_CONTROL_3			0x02
#define PCF8523_SECONDS				0x03
#define PCF8523_MINUTES				0x04
#define PCF8523_HOURS				0x05
#define PCF8523_DAYS				0x06
#define PCF8523_WEEKDAYS			0x07
#define PCF8523_MONTHS				0x08
#define PCF8523_YEARS				0x09
#define PCF8523_MINUTE_ALARM		0x0A
#define PCF8523_HOUR_ALARM			0x0B
#define PCF8523_DAY_ALARM			0x0C
#define PCF8523_WEEKDAY_ALARM		0x0D
#define PCF8523_OFFSET				0x0E
#define PCF8523_TMR_CLKOUT_CTRL		0x0F
#define PCF8523_TMR_A_FREQ_CTRL		0x10
#define PCF8523_TMR_A_REG			0x11
#define PCF8523_TMR_B_FREQ_CTRL		0x12
#define PCF8523_TMR_B_REG			0x13

#define PCF8523_CONTROL_1_CAP_SEL_BIT	7
#define PCF8523_CONTROL_1_T_BIT			6
#define PCF8523_CONTROL_1_STOP_BIT		5
#define PCF8523_CONTROL_1_SR_BIT		4
#define PCF8523_CONTROL_1_1224_BIT		3
#define PCF8523_CONTROL_1_SIE_BIT		2
#define PCF8523_CONTROL_1_AIE_BIT		1
#define PCF8523_CONTROL_1CIE_BIT		0

#define PCF8523_CONTROL_2_WTAF_BIT		7
#define PCF8523_CONTROL_2_CTAF_BIT		6
#define PCF8523_CONTROL_2_CTBF_BIT		5
#define PCF8523_CONTROL_2_SF_BIT		4
#define PCF8523_CONTROL_2_AF_BIT 		3
#define PCF8523_CONTROL_2_WTAIE_BIT		2
#define PCF8523_CONTROL_2_CTAIE_BIT		1
#define PCF8523_CONTROL_2_CTBIE_BIT		0

#define PCF8523_SECONDS_OS_BIT			7
#define PCF8523_SECONDS_10_BIT       	6
#define PCF8523_SECONDS_10_LENGTH   	3
#define PCF8523_SECONDS_1_BIT        	3
#define PCF8523_SECONDS_1_LENGTH     	4

#define PCF8523_MINUTES_10_BIT       	6
#define PCF8523_MINUTES_10_LENGTH    	3
#define PCF8523_MINUTES_1_BIT        	3
#define PCF8523_MINUTES_1_LENGTH     	4

#define PCF8523_HOURS_MODE_BIT  	    3 // 0 = 24-hour mode, 1 = 12-hour mode
#define PCF8523_HOURS_AMPM_BIT      	5 // 2nd HOURS_10 bit if in 24-hour mode
#define PCF8523_HOURS_10_BIT        	4
#define PCF8523_HOURS_1_BIT          	3
#define PCF8523_HOURS_1_LENGTH       	4

#define PCF8523_WEEKDAYS_BIT 	        2
#define PCF8523_WEEKDAYS_LENGTH         3

#define PCF8523_DAYS_10_BIT          5
#define PCF8523_DAYS_10_LENGTH       2
#define PCF8523_DAYS_1_BIT           3
#define PCF8523_DAYS_1_LENGTH        4

#define PCF8523_MONTH_10_BIT         4
#define PCF8523_MONTH_1_BIT          3
#define PCF8523_MONTH_1_LENGTH       4

#define PCF8523_YEAR_10H_BIT         7
#define PCF8523_YEAR_10H_LENGTH      4
#define PCF8523_YEAR_1H_BIT          3
#define PCF8523_YEAR_1H_LENGTH       4



#define PCF8523_TMR_CLKOUT_CTRL_TAM_BIT		7
#define PCF8523_TMR_CLKOUT_CTRL_TBM_BIT		6
#define PCF8523_TMR_CLKOUT_CTRL_TBC_BIT		0



typedef enum {
	eSUNDAY = 0,
	eMONDAY,
	eTUESDAY,
	eWEDNESDAY,
	eTHURSDAY,
	eFRIDAY,
	eSATURDAY
} eWEEKDAYS;

typedef enum {
	eTB_4KHZ = 0,
	eTB_64HZ,
	eTB_SECOND,
	eTB_MINUTE,
	eTB_HOUR
} eTIMER_TIMEBASE;

typedef enum {
	eCAP_7pF = 0,
	eCAP_12_5pF
}eCAP_SEL;

typedef enum {
	eAM = 0,
	ePM
}eAM_PM;

typedef enum {
	eTWENTYFOURHOUR = 0,
	eTWELVEHOUR
}e12_24;

typedef struct {
	uint8_t		minutes;
	uint8_t		hours;
	uint8_t     days;
	eWEEKDAYS	weekdays;
	e12_24		twentyFourHour;
	eAM_PM		AmPm;
	uint8_t		minutesEnabled;
	uint8_t 		hoursEnabled;
	uint8_t 		daysEnabled;
	uint8_t 		weekdaysEnabled;
}ALARM_SETTINGS;


typedef struct {
	uint8_t		year;
	uint8_t		month;
	uint8_t     day;
	uint8_t		hour;
	uint8_t		minute;
	uint8_t     second;
}RTCDateTime;

#include <stdio.h>
#include <stdlib.h>

// Simple general-purpose date/time class (no TZ / DST / leap second handling!)
class DateTime {
public:



    //! Destructor
    ~DateTime();
    DateTime (uint32_t t =0);

protected:
    uint8_t yOff, m, d, hh, mm, ss;
};

class PCF8523{

	public:
		static bool begin(void);
		static void setTime(const DateTime& dt);

		static DateTime readTime();

		// Alarms
		void setAlarm(uint8_t minute_alarm );
		void setAlarm(uint8_t hour_alarm,uint8_t minute_alarm );
		void setAlarm(uint8_t day_alarm, uint8_t hour_alarm,uint8_t minute_alarm );
		void setWeekDayAlarm(eWEEKDAYS weekday_alarm, uint8_t hour_alarm,uint8_t minute_alarm);
		void getAlarm(uint8_t* buf);
		void getAlarm(ALARM_SETTINGS* settings);
		void enableAlarm(bool enable);
		void ackAlarm(void);

		// Periodic Timers
		void setTimer1(eTIMER_TIMEBASE timebase, uint8_t value);
		void ackTimer1(void);
		uint8_t getTimer1(void);
		void setTimer2(eTIMER_TIMEBASE timebase,uint8_t value);
		void ackTimer2(void);
		uint8_t getTimer2(void);

		// Utility
		void reset();
		uint8_t isrunning(void);
		void rtcStop(void);
		void rtcStart(void);
		bool rtcBatteryLow(void);
		void setTwelveTwentyFourHour(e12_24 mode);
		e12_24 getTwelveTwentyFourHour(void);

		// Register Access
		uint8_t rtcReadReg(uint8_t address);
		void rtcReadReg(uint8_t* buf, uint8_t size, uint8_t address);
		void rtcWriteReg(uint8_t address, uint8_t data);
		void rtcWriteReg(uint8_t address, uint8_t* buf, uint8_t size);

		// void startCounter_1(uint8_t value);
		void setBatterySwitchover(void);
	protected:
		void stop_32768_clkout();
		uint8_t clearRtcInterruptFlags();
		void 	rtcCapSelect(eCAP_SEL value);


};





void PCF8563_TEST1Enable(uint8_t Enable);
void PCF8563_STOPEnable(uint8_t Enable);
void PCF8563_TESTCEnable(uint8_t Enable);
void PCF8563_InterruptEnable(uint8_t Enable);
void PCF8563_AlarmFlagEnable(uint8_t Enable);
void PCF8563_TimerFlagEnable(uint8_t Enable);
void PCF8563_AlarmInterruptEnable(uint8_t Enable);
void PCF8563_TimerInterruptEnable(uint8_t Enable);



void PCF8563_GetDateTime(RTCDateTime *DateTime);	// Use in blocking/interrupt mode in PCF8563_INT EXTI handler
void PCF8563_Init(I2C_HandleTypeDef *hi2c);

#ifdef __cplusplus
}
#endif

#endif //_PCF8523_H_
