#include "ISL94202.h"

const union BMSConfigRegisters BasicBMSConfig ={{
		0xFF,

		0xFD,

		0x7F,

		0x0D,

		0xAA,

		0xF9,

		0xAA,

		0x0A,

		0x2A,

		0x0E,

		0x55,

		0x07,

		0xD4,

		0x0D,

		0x00,

		0x08,

		0x01,

		0x08,

		0x01,

		0x08,


		0x64,

		0x02,


		0x05,

		0x78,


		0x05,

		0x78,


		0x05,

		0x68,


		0xAA,

		0x09,

		0xD4,

		0x0D,

		0x22,

		0x00,

		0xAB,

		0x01,


		0x64,

		0x04,


		0x05,

		0x00,

		0x78,

		0x0B,

		0x27,

		0x0B,

		0x0F,

		0x06,

		0xC3,

		0x06,

		0x33,

		0x04,

		0xC5,

		0x04,

		0xAB,

		0x0B,

		0x78,

		0x0B,

		0x33,

		0x04,

		0xC5,

		0x04,

		0xAB,

		0x0B,

		0x78,

		0x0B,

		0x64,

		0x06,

		0x10,

		0x06,

		0xFF,

		0x08,


		0x01,


		0x1E,



		0xF3,


		0xEF,





		0xA0,



		0xC1

}};

const short int TempLUT[] = { -40, -39, -38, -37, -36, -35, -34, -33, -32, -31,
		-30, -29, -28, -27, -26, -25, -24, -23, -22, -21, -20, -19, -18, -17,
		-16, -15, -14, -13, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1,
		0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
		20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37,
		38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55,
		56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73,
		74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90 };

const float VLUT[] = { 0.765099977448696, 0.764050116083580, 0.762942053685556,
		0.761766572957287, 0.760526188868525, 0.759217003384268,
		0.757843991720231, 0.756380946868851, 0.754853176065856,
		0.753242732368661, 0.751538514728253, 0.749761834969955,
		0.747883988196925, 0.745908044086198, 0.743840327355041,
		0.741662060070020, 0.739378288832586, 0.736997940267765,
		0.734494015233950, 0.731917336394948, 0.729171716682019,
		0.726322416526052, 0.723343755053636, 0.720239109445107,
		0.716998624319636, 0.713633788632215, 0.710123816460306,
		0.706469718064741, 0.702673542535294, 0.698738382364237,
		0.694654373236598, 0.690406976744186, 0.686026283130928,
		0.681480449521687, 0.676784498687154, 0.671914758269720,
		0.666910590739755, 0.661746301034598, 0.656413422428821,
		0.650931093048647, 0.645290153145172, 0.639513788098694,
		0.633550460607727, 0.627460629921260, 0.621181642080167,
		0.614781884394855, 0.608240054759221, 0.601584713162736,
		0.594721305782254, 0.587801638753827, 0.580723819301848,
		0.573510926319861, 0.566191191191191, 0.558706235757199,
		0.551175305204541, 0.543538393449878, 0.535714285714286,
		0.527956260315963, 0.520073547289083, 0.512089701829116,
		0.504032258064516, 0.495781291522721, 0.487667395463241,
		0.479418375948300, 0.471230158730159, 0.462962962962963,
		0.454680184836140, 0.446372903547603, 0.438071049773177,
		0.429761632092141, 0.421474144282939, 0.413215619432644,
		0.404964883704638, 0.396776574803150, 0.388597485196286,
		0.380481924339651, 0.372428737250723, 0.364433784727611,
		0.356489719256623, 0.348655464722938, 0.340852021154477,
		0.333173437891049, 0.325570543819462, 0.318025010780509,
		0.310638088145098, 0.303309641395266, 0.296100171619547,
		0.288985885803318, 0.282031322613626, 0.275116024312099,
		0.268349527310924, 0.261701368736700, 0.255187928501469,
		0.248773173391494, 0.242524448345035, 0.236348534036725,
		0.230312083290594, 0.224371435977190, 0.218596139744396,
		0.212940429584171, 0.207353883961962, 0.201968965703192,
		0.196672970178494, 0.191473591742616, 0.186444490586932,
		0.181462869905081, 0.176669433813597, 0.171937039137833,
		0.167340553206081, 0.162887836748791, 0.158528932662431,
		0.154276515461029, 0.150136228776218, 0.146098664916513,
		0.142168834182095, 0.138336320816825, 0.134613300907756,
		0.130980774352890, 0.127450488119583, 0.124018406690895,
		0.120680113147214, 0.117422615589969, 0.114265379743692,
		0.111186656353701, 0.108197476253728, 0.105292116059855,
		0.102464519512688, 0.0997169580885432, 0.0970429516821442,
		0.0944532798414424, 0.0919324073717579 };


struct ISL94202_OutputValues BMSMeasurements;
struct BMSOperationRegs BMSOpRegs;
union BMSDataRegisters BMSDataRegs;

void BMS_writeByte(uint8_t Dest, uint8_t Data) {
	static uint8_t dataBuf[2];
	dataBuf[0] = Dest;
	dataBuf[1] = Data;
	while (ISL94202_I2C_PORT.State == HAL_I2C_STATE_BUSY) {
		;
	}
	HAL_I2C_Master_Transmit(&ISL94202_I2C_PORT, (ISL94202_I2C_ADDR << 1),
			dataBuf, 2, 10);
}

uint8_t BMS_readByte(uint8_t Dest) {
	static uint8_t temp;
	static uint8_t data;
	temp = Dest;
	while (ISL94202_I2C_PORT.State == HAL_I2C_STATE_BUSY) {
		;
	}
	HAL_I2C_Master_Transmit(&ISL94202_I2C_PORT, (ISL94202_I2C_ADDR << 1), &temp,
			1, 10);
	HAL_I2C_Master_Receive(&ISL94202_I2C_PORT, (ISL94202_I2C_ADDR << 1), &data,
			1, 10);
	return data;

}

void ISL94202_Init(void) {
	ISL94202_WriteCfgBytes(BasicBMSConfig);

	//static union CONTROL0_REG Control0;
	//Control0.CONTROL0 = 0x00;
	//Control0.CONTROL0_BIT.CG = 0x01;

	//BMS_writeByte(CONTROL0_ADDR, Control0.CONTROL0);

	//static union CONTROL1_REG Control1;
	//Control1.CONTROL1 = 0x00;

	//BMS_writeByte(CONTROL1_ADDR, Control1.CONTROL1);
}

union BMSConfigRegisters ISL94202_ReadCfgBytes(void) {
	union BMSConfigRegisters CurrentCfg;

	for (int i = 0; i <= 75; i++) {
		CurrentCfg.BMSConfigRegsArray[i] = BMS_readByte(
				ISL94202_Config_Regs_Start + i);
	}

	return CurrentCfg;
}

void ISL94202_WriteCfgBytes(union BMSConfigRegisters Config) {

	for (int i = 0; i <= 75; i++) {
		BMS_writeByte(ISL94202_Config_Regs_Start + i,
				Config.BMSConfigRegsArray[i]);
	}
}

uint8_t ISL94202_CompareCfgBytes(union BMSConfigRegisters Config1,
		union BMSConfigRegisters Config2) {
	for (int i = 0; i <= 75; i++) {
		if (Config1.BMSConfigRegsArray[i] != Config2.BMSConfigRegsArray[i])
			return 1;
	}
	return 0;
}

void ISL94202_EnableEEPROM(void) {
	BMS_writeByte(CONTROL3_ADDR, 0x01); // Force IDLE State
	BMS_writeByte(CONTROL2_ADDR, 0x04);

	BMS_writeByte(EEPROM_ENABLE_ADDR, 0x01); // Enable EEPROM
}
void ISL94202_DisableEEPROM(void) {
	BMS_writeByte(EEPROM_ENABLE_ADDR, 0x00);
}

void ISL94202_ReadPackCurrent(void) {

	int sign;

	ISL94202_ReadStatusRegisters();
	if (BatteryPack == Discharging)
		sign = -1.0;
	else if (BatteryPack == Charging)
		sign = 1.0;
	else
		sign = 0.0;
	BMSDataRegs.BMSDataRegisters.IPACK_LSB = BMS_readByte(IPACK_LSB_ADDR);
	BMSDataRegs.BMSDataRegisters.IPACK_MSB = BMS_readByte(IPACK_MSB_ADDR);

	uint32_t temp = BMSDataRegs.BMSDataRegisters.IPACK_LSB + (BMSDataRegs.BMSDataRegisters.IPACK_MSB << 8);
	BMSMeasurements.PackCurrent = ((float) temp) * 1.8 / (4095.0 * 5.0 * 0.001)* sign;

}

void ISL94202_ReadCellVoltages(void) {

	for (int i = 0; i < 16; i++) {
		BMSDataRegs.BMSDataRegsArray[6 + i] = BMS_readByte(VCELL1_LSB_ADDR + i);
	}

	for (int i = 0; i < 8; i++) {
		BMSMeasurements.CellVoltages[i] =
				(float) (BMSDataRegs.BMSDataRegsArray[6 + 2 * i]
						+ (BMSDataRegs.BMSDataRegsArray[6 + 2 * i + 1] << 8))
						* Vcell_ConvConst;
	}
}

void ISL94202_ReadTemperatures(void) {

	for (int i = 0; i < 6; i++) {
		BMSDataRegs.BMSDataRegsArray[22 + i] = BMS_readByte(ITEMP_LSB_ADDR + i);
	}

	BMSMeasurements.InternalTemp =
			(float) ((BMSDataRegs.BMSDataRegisters.ITEMP_LSB
					+ (BMSDataRegs.BMSDataRegisters.ITEMP_MSB << 8))
					* IntTemp_ConvConst - 273.15);

	float VT1 = (float) ((BMSDataRegs.BMSDataRegisters.XT1_LSB
			+ (BMSDataRegs.BMSDataRegisters.XT1_MSB << 8)) * 1.8 / 4095.0)
			/ 2.0;
	float VT2 = (float) ((BMSDataRegs.BMSDataRegisters.XT2_LSB
			+ (BMSDataRegs.BMSDataRegisters.XT2_MSB << 8)) * 1.8 / 4095.0)
			/ 2.0;

//		BMSMeasurements.Temp1 = p1 * pow(VT1,3) + p2 * pow(VT1,2) + p3 * VT1 + p4;
//		BMSMeasurements.Temp2 = p1 * pow(VT2,3) + p2 * pow(VT2,2) + p3 * VT2 + p4;
	BMSMeasurements.Temp1 = approximateTemp(VT1, VLUT, TempLUT,
			sizeof(VLUT) / sizeof(VLUT[0]));
	BMSMeasurements.Temp2 = approximateTemp(VT2, VLUT, TempLUT,
			sizeof(VLUT) / sizeof(VLUT[0]));
}

void ISL94202_ReadPackVoltage(void) {
	BMSDataRegs.BMSDataRegisters.VBATT_LSB = BMS_readByte(VBATT_LSB_ADDR);
	BMSDataRegs.BMSDataRegisters.VBATT_MSB = BMS_readByte(VBATT_MSB_ADDR);

	BMSMeasurements.Vpack = (float) ((BMSDataRegs.BMSDataRegisters.VBATT_LSB
			+ (BMSDataRegs.BMSDataRegisters.VBATT_MSB << 8)) * Vpack_ConvConst);
}

void ISL94202_MinMaxVoltage(void) {
	BMSDataRegs.BMSDataRegisters.CELLMAX_LSB = BMS_readByte(CELLMAX_LSB_ADDR);
	BMSDataRegs.BMSDataRegisters.CELLMAX_MSB = BMS_readByte(CELLMAX_MSB_ADDR);
	BMSDataRegs.BMSDataRegisters.CELLMIN_LSB = BMS_readByte(CELLMIN_LSB_ADDR);
	BMSDataRegs.BMSDataRegisters.CELLMIN_MSB = BMS_readByte(CELLMIN_MSB_ADDR);

	BMSMeasurements.Vcellmax =
			(float) (BMSDataRegs.BMSDataRegisters.CELLMAX_LSB
					+ (BMSDataRegs.BMSDataRegisters.CELLMAX_MSB << 8))
					* Vcell_ConvConst;
	BMSMeasurements.Vcellmin =
			(float) (BMSDataRegs.BMSDataRegisters.CELLMIN_LSB
					+ (BMSDataRegs.BMSDataRegisters.CELLMIN_MSB << 8))
					* Vcell_ConvConst;

	BMSMeasurements.Vcelldiff = BMSMeasurements.Vcellmax - BMSMeasurements.Vcellmin;

}

void ISL94202_ReadVRGO(void) {
	BMSDataRegs.BMSDataRegisters.VRGO_LSB = BMS_readByte(VRGO_LSB_ADDR);
	BMSDataRegs.BMSDataRegisters.VRGO_MSB = BMS_readByte(VRGO_MSB_ADDR);

	BMSMeasurements.VRGO = (float) (BMSDataRegs.BMSDataRegisters.VRGO_LSB
			+ (BMSDataRegs.BMSDataRegisters.VRGO_LSB << 8)) * VRGO_ConvConst
			/ 2.0;
}
int ISL94202_ReadAllMeasurements(void) {
//	if (BMSOpRegs.STATUS2.STATUS2_BIT.INT_SCAN == 0) {
//		BMSOpRegs.STATUS2.STATUS2 = BMS_readByte(STATUS2_ADDR);
//		if (BMSOpRegs.STATUS2.STATUS2_BIT.INT_SCAN == 1) {
//			ISL94202_ReadCellVoltages();
//			ISL94202_ReadPackCurrent();
//			ISL94202_ReadTemperatures();
//			ISL94202_ReadPackVoltage();
//			ISL94202_MinMaxVoltage();
//			ISL94202_ReadVRGO();
//			BMSOpRegs.STATUS2.STATUS2_BIT.INT_SCAN = 0;
//			return 1;
//		}
//	}


	ISL94202_ReadCellVoltages();
	ISL94202_ReadPackCurrent();
	ISL94202_ReadTemperatures();
	ISL94202_ReadPackVoltage();
	ISL94202_MinMaxVoltage();
	ISL94202_ReadVRGO();

	ISL94202_ReadStatusRegisters();
	ISL94202_ReadControlRegisters();


	return 0;
}

void ISL94202_ReadStatusRegisters(void) {
	BMSOpRegs.CBFC = BMS_readByte(CBFC_ADDR);
	BMSOpRegs.STATUS0.STATUS0 = BMS_readByte(STATUS0_ADDR);
	BMSOpRegs.STATUS1.STATUS1 = BMS_readByte(STATUS1_ADDR);
	BMSOpRegs.STATUS2.STATUS2 = BMS_readByte(STATUS2_ADDR);
	BMSOpRegs.STATUS3.STATUS3 = BMS_readByte(STATUS3_ADDR);

}

void ISL94202_ReadControlRegisters(void) {

	BMSOpRegs.CONTROL0.CONTROL0 = BMS_readByte(CONTROL0_ADDR);
	BMSOpRegs.CONTROL1.CONTROL1 = BMS_readByte(CONTROL1_ADDR);
	BMSOpRegs.CONTROL2.CONTROL2 = BMS_readByte(CONTROL2_ADDR);
	BMSOpRegs.CONTROL3.CONTROL3 = BMS_readByte(CONTROL3_ADDR);
}

void ISL94202_ControlDFET(int State) {

	BMSOpRegs.CONTROL1.CONTROL1 = BMS_readByte(CONTROL1_ADDR);
	if (BMSOpRegs.CONTROL1.CONTROL1_BIT.DFET == State)
		return;
	BMSOpRegs.CONTROL1.CONTROL1_BIT.DFET = State;
	BMS_writeByte(CONTROL1_ADDR, BMSOpRegs.CONTROL1.CONTROL1);
}

void ISL94202_ControlCFET(int State) {

	BMSOpRegs.CONTROL1.CONTROL1 = BMS_readByte(CONTROL1_ADDR);
	if (BMSOpRegs.CONTROL1.CONTROL1_BIT.CFET == State)
		return;
	BMSOpRegs.CONTROL1.CONTROL1_BIT.CFET = State;
	BMS_writeByte(CONTROL1_ADDR, BMSOpRegs.CONTROL1.CONTROL1);
}

void ISL94202_ControlFETs(int State) {
	BMSOpRegs.CONTROL1.CONTROL1 = BMS_readByte(CONTROL1_ADDR);
	BMSOpRegs.CONTROL2.CONTROL2 = BMS_readByte(CONTROL2_ADDR);
	if (BMSOpRegs.CONTROL2.CONTROL2_BIT.uC_FET == 0 && State == 1)
		return;

	if (BMSOpRegs.CONTROL2.CONTROL2_BIT.uC_FET == 1 && State == 1) {
		BMSOpRegs.CONTROL2.CONTROL2_BIT.uC_FET = 0;

		BMS_writeByte(CONTROL2_ADDR, BMSOpRegs.CONTROL2.CONTROL2);
	}

	if (State == 0) {
		BMSOpRegs.CONTROL2.CONTROL2_BIT.uC_FET = 1;

		BMSOpRegs.CONTROL1.CONTROL1_BIT.CFET = 0;
		BMSOpRegs.CONTROL1.CONTROL1_BIT.DFET = 0;
		BMS_writeByte(CONTROL2_ADDR, BMSOpRegs.CONTROL2.CONTROL2);
		BMS_writeByte(CONTROL1_ADDR, BMSOpRegs.CONTROL1.CONTROL1);
	}

}

float approximateTemp(float input, const float *RLUT, const short int *TLUT,
		int LUTsize) {
	static int i = 0;
	static short MatchFound = 0;
	static float temp;
	static float a;
	MatchFound = 0;
	i = 0;
	do {

		if ((*(RLUT + i) >= input) && (*(RLUT + i + 1) <= input)) {
			MatchFound = 1;
			a = (*(RLUT + i) - *(RLUT + i + 1))
					/ (float) (*(TLUT + i) - (float) *(TLUT + i + 1));
			temp = ((input - *(RLUT + i)) / a) + ((float) *(TLUT + i));

			return temp;
		}
		i++;
		if (i >= LUTsize)
			return -255.0;

	} while (MatchFound != 1);

	return -255.0;
}

