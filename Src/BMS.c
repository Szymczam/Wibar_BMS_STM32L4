/*
 * BMS.c
 *
 *  Created on: Jun 6, 2020
 *      Author: bnowa
 */
#include "BMS.h"

const float RLUT[] = { 1.00993197023228, 1.00854615323033, 1.00708351086493,
		1.00553187630362, 1.00389456930645, 1.00216644446723, 1.00035406907070,
		0.998422849866883, 0.996406192406930, 0.994280406726633,
		0.992030839441294, 0.989685622160340, 0.987206864419941,
		0.984598618193782, 0.981869232108654, 0.978993919292427,
		0.975979341259014, 0.972837281153450, 0.969532100108814,
		0.966130884041332, 0.962506666020265, 0.958745589814389,
		0.954813756670799, 0.950715624467541, 0.946438184101920,
		0.941996600994524, 0.937363437727604, 0.932540027845458,
		0.927529076146588, 0.922334664720793, 0.916943772672310,
		0.911337209302326, 0.905554693732825, 0.899554193368626,
		0.893355538267043, 0.886927480916031, 0.880321979776477,
		0.873505117365669, 0.866465717606043, 0.859229042824214,
		0.851783002151626, 0.844158200290276, 0.836286608002200,
		0.828248031496063, 0.819959767545820, 0.811512087401209,
		0.802876872282171, 0.794091821374812, 0.785032123632575,
		0.775898163155051, 0.766555441478439, 0.757034422742216,
		0.747372372372372, 0.737492231199503, 0.727551402869994,
		0.717470679353839, 0.707142857142857, 0.696902263617072,
		0.686497082421590, 0.675958406414433, 0.665322580645161,
		0.654431304809992, 0.643720962011478, 0.632832256251756,
		0.622023809523809, 0.611111111111111, 0.600177843983705,
		0.589212232682836, 0.578253785700594, 0.567285354361626,
		0.556345870453480, 0.545444617651089, 0.534553646490122,
		0.523745078740157, 0.512948680459098, 0.502236140128339,
		0.491605933170954, 0.481052595840447, 0.470566429418743,
		0.460225213434278, 0.449924667923909, 0.439788938016184,
		0.429753117841690, 0.419793014230272, 0.410042276351529,
		0.400368726641751, 0.390852226537801, 0.381461369260379,
		0.372281345849986, 0.363153152091971, 0.354221376050420,
		0.345445806732443, 0.336848065621939, 0.328380588876772,
		0.320132271815446, 0.311980064928477, 0.304011949943584,
		0.296170295489891, 0.288546904462602, 0.281081367051106,
		0.273707126829790, 0.266599034728214, 0.259608320635612,
		0.252745141100253, 0.246106727574751, 0.239530988274707,
		0.233203652633949, 0.226956891661940, 0.220889530232027,
		0.215011944508404, 0.209258191114409, 0.203645000408559,
		0.198179821984607, 0.192850237689797, 0.187662861120366,
		0.182603943478209, 0.177689557198237, 0.172894622145815,
		0.168234644317850, 0.163704296831981, 0.159297749354323,
		0.154997852578759, 0.150830301261673, 0.146766386386886,
		0.142820668654921, 0.138985593199008, 0.135253165756749,
		0.131626384676877, 0.128096696220430, 0.124678329390704,
		0.121350777730720 };


const short int TempLUT_MCU[] = { -40, -39, -38, -37, -36, -35, -34, -33, -32, -31,
		-30, -29, -28, -27, -26, -25, -24, -23, -22, -21, -20, -19, -18, -17,
		-16, -15, -14, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1,
		0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
		20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37,
		38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55,
		56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73,
		74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90 };


union fExtMeas fExternalMeasurements;
union extMeas externalMeasurements;
BatteryState BatteryPack;
volatile double BatterySOC;
void CheckForFaults(void)
{

}

void StartExternalMeasurements(void)
{
//
//	while(HAL_ADCEx_Calibration_Start(&hExtADC, ADC_SINGLE_ENDED) != HAL_OK);
//
//HAL_ADC_Start_DMA(&hExtADC, (uint32_t*)externalMeasurements.MeasurementArray, 5);


}


void ConvertExternalMeasurements(void)
{
//	float VDDA;
//	VREFINT_CAL= *((uint16_t*)VREFINT_CAL_ADDR);
//	VDDA = 3.0 * (float)VREFINT_CAL/(float)externalMeasurements.extMeasurementsStruct.Vref;
//fExternalMeasurements.fExtMeasurementsStruct.VBAT = externalMeasurements.extMeasurementsStruct.VBAT / 4095.0 * VDDA * VBAT_DIV;
//fExternalMeasurements.fExtMeasurementsStruct.VIN_CHRG = (float)externalMeasurements.extMeasurementsStruct.VIN_CHRG / 4095.0 * VDDA * VIN_DIV;
//fExternalMeasurements.fExtMeasurementsStruct.MCP9700A = (((float)externalMeasurements.extMeasurementsStruct.MCP9700A * VDDA / 4095.0) - 0.5) / Tc;
//fExternalMeasurements.fExtMeasurementsStruct.BAT_CURR = (((float)externalMeasurements.extMeasurementsStruct.BAT_CURR * VDDA / 4095.0) - 1.65) / (0.025);
//fExternalMeasurements.fExtMeasurementsStruct.MCP9700A = approximateTemp(((float)externalMeasurements.extMeasurementsStruct.MCP9700A * VDDA / 4095.0),RLUT ,TempLUT_MCU, sizeof(RLUT)/sizeof(RLUT[0]));

}


BatteryState CheckBatteryStatus(void) {


	if (BMSOpRegs.STATUS2.STATUS2_BIT.CHING == 1
			&& (BMSDataRegs.BMSDataRegisters.IPACK_LSB > 3)) {
		return Charging;
	}

	if (BMSOpRegs.STATUS2.STATUS2_BIT.DCHING == 1
			&& (BMSDataRegs.BMSDataRegisters.IPACK_LSB > 3)) {
		return Discharging;
	}

	return Idle;

}

void UpdateLED(BatteryState BatteryPack) {
	static uint8_t Segments = 0;
	static uint8_t BasicSegments = 0;
	static uint8_t PrevSegments = 0;

	if (BatterySOC >= 0.85) {
		BasicSegments = 0x0F;

	}

	if (BatterySOC >= 0.5 && BatterySOC < 0.85) {
		BasicSegments = 0x07;

	}

	if (BatterySOC >= 0.3 && BatterySOC < 0.5) {
		BasicSegments = 0x03;

	}

	if (BatterySOC >= 0.1 && BatterySOC < 0.3) {
		BasicSegments = 0x01;

	}

	if (BatterySOC >= 0.0 && BatterySOC < 0.1) {
		BasicSegments = 0x00;

	}

	switch (BatteryPack) {

	case Charging:

		Segments = BasicSegments | ((PrevSegments << 1) + 1);

		PrevSegments = Segments;
		if (Segments > 0x0F) {
			Segments = BasicSegments;
			PrevSegments = BasicSegments;
		}

		break;
	case Discharging:

		Segments = BasicSegments & (PrevSegments >> 1);
		PrevSegments = Segments;

		if (Segments == 0 && PrevSegments == 0) {
			Segments = BasicSegments;
			PrevSegments = BasicSegments;
		}
		break;

	default:
		Segments = BasicSegments;
	}

//	LED_DRV_SetSegments(Segments);
//	LED_DRV_UpdateRegisters();
}



void BMSCheckTemp(void) {

//	static uint8_t FET_State = 1;

	for (int i = 0; i < 4; i++) {
		h[i].Tmax = fmaxf(*h[i].T1, *h[i].T2);
		h[i].Tmin = fminf(*h[i].T1, *h[i].T2);
	}
/*	if (OneWireBus.SensorNum == 17 && heaterEnable == 1) {

		for (int i = 0; i < 4; i++) {
			if (h[i].Tmax > 50.0) {
				//idlecnt = 0;
				h[i].DIS = 1;
				h[i].DIR = COOL;
				h[i].duty = 0.6;
				h[i].overheat = 1;
			}

			else if (h[i].Tmin > 35.0 && h[i].Tmax <= 50.0) {
				//idlecnt = 0;
				h[i].DIS = 1;
				h[i].DIR = COOL;
				h[i].duty = 0.3;
				h[i].overheat = 1;
			}

			else if (h[i].Tmin > 10.0 && h[i].Tmax <= 35.0) {

				h[i].DIS = 1;
				h[i].duty = 0.0;
				h[i].overheat = 0;
				h[i].underheat = 0;

			} else if (h[i].Tmin < 0.0) {
				//idlecnt = 0;
				h[i].DIS = 1;
				h[i].duty = 0.0;
				h[i].underheat = 1;
				h[i].DIR = HEAT;
				h[i].duty = 0.6;

			}

			else if (h[i].Tmin >= 0.0 && h[i].Tmax < 10.0) {
				h[i].DIS = 1;

				h[i].underheat = 1;
				h[i].DIR = HEAT;
				h[i].duty = 0.3;

			}

			if (BMSMeasurements.InternalTemp > 65.0) {

				h[i].DIS = 1;
				h[i].DIR = COOL;
				h[i].duty = 0.6;
				h[i].overheat = 1;
				h[i].underheat = 0;
			}

		}

		if (*T1_P > 80.0 || *T2_P > 80.0 || *T3_P > 80.0 || *T4_P > 80.0) {
			h[0].DIS = 0;
			h[0].duty = 0.0;

			h[1].DIS = 0;
			h[1].duty = 0.0;

			h[2].DIS = 0;
			h[2].duty = 0.0;

			h[3].DIS = 0;
			h[3].duty = 0.0;

		}

		if (*T1_WYM > 80.0 || *T2_WYM > 80.0 || *T3_WYM > 80.0) {
			h[0].DIS = 0;
			h[0].duty = 0.0;

			h[1].DIS = 0;
			h[1].duty = 0.0;

			h[2].DIS = 0;
			h[2].duty = 0.0;

			h[3].DIS = 0;
			h[3].duty = 0.0;

		}
	} else {
//		h[0].DIS = 0;
//		h[0].duty = 0.0;
//
//		h[1].DIS = 0;
//		h[1].duty = 0.0;
//
//		h[2].DIS = 0;
//		h[2].duty = 0.0;
//
//		h[3].DIS = 0;
//		h[3].duty = 0.0;
	}
*/
	if (h[0].Tmax >= 60.0 || h[1].Tmax >= 60.0 || h[2].Tmax >= 60.0
			|| h[3].Tmax >= 60.0 || *FET_Temp >= 54.0 || h[0].Tmin <= -1.0
			|| h[1].Tmin <= -1.0 || h[2].Tmin <= -1.0 || h[3].Tmin <= -1.0) {
		h[0].duty = 0.0;
		h[1].duty = 0.0;
		h[2].duty = 0.0;
		h[3].duty = 0.0;
		ISL94202_ControlFETs(0);
		//new_event = goPowerdownMode_e;
	} else if (h[0].Tmax <= 50.0 && h[1].Tmax <= 50.0 && h[2].Tmax <= 50.0
			&& h[3].Tmax <= 50.0 && *FET_Temp <= 52.0 && h[0].Tmin >= 5.0
			&& h[1].Tmin >= 5.0 && h[2].Tmin >= 5.0 && h[3].Tmin >= 5.0)

		ISL94202_ControlFETs(1 && CAN_FET_Control);

}

void BMSCheckSOC(void)
{
	float V = BMSMeasurements.Vcellmin;
	BatterySOC = SOCp1 * pow(V,4) + SOCp2 * pow(V,3) + SOCp3 * pow(V,2) + SOCp4 * V + SOCp5;

}

void BMSSaveSOCValue(void)
{

}


void BackupRegisterInit(void)
{

	__HAL_RCC_PWR_CLK_ENABLE();
	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_RTC_CONFIG(RCC_RTCCLKSOURCE_LSI);
	__HAL_RCC_RTC_ENABLE();

}

//void BackupRegWrite(uint8_t regnum,uint32_t data)
//{
//volatile unsigned int *ptr;
//ptr = RTC_BASE + 0x50 + regnum * 4;
//*ptr = data;


//}
//uint32_t BackupRegRead(uint32_t regnum)
//{
//	volatile unsigned int *ptr;
//	ptr = RTC_BASE + 0x50 + regnum * 4;
//	return *ptr;
//}
