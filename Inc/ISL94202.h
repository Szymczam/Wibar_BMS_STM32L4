/*
 * ISL94202.h
 *
 *  Created on: Jun 6, 2020
 *      Author: bnowa
 */

#ifndef ISL94202_H_
#define ISL94202_H_

#include "stdint.h"
#include "main.h"
#include "math.h"
#include "BMS.h"


	#define ISL94202_I2C_PORT hi2c2
	extern I2C_HandleTypeDef ISL94202_I2C_PORT;
	#define ISL94202_I2C_ADDR 0x28

	#define ISL94202_Config_Regs_Start 0x00
	#define ISL94202_Config_Regs_End 0x4B

	#define ISL94202_Operation_Regs_Start 0x80
	#define ISL94202_Operation_Regs_Stop 0x89

	#define ISL94202_Data_Regs_Start 0x8A
	#define ISL94202_Data_Regs_Stop 0xAB



	#define Vcell_ConvConst 1.8 * 8 / (4095.0 * 3.0)
	#define IntTemp_ConvConst 0.237254
	#define Vpack_ConvConst 0.014066
	#define VRGO_ConvConst 1.8*2.0/(4095.0)



	#define p1 -801.6
	#define p2 987.2
	#define p3 -535.9
	#define p4 141.0



	extern const float RLUT[];

//***********************************************************************
// Feedback struct

	struct ISL94202_OutputValues{
		float Vcellmin;
		float Vcellmax;
		float PackCurrent;
		float CellVoltages[8];

		float InternalTemp;
		float Temp1;
		float Temp2;
		float Vpack;
		float VRGO;

	};

extern struct ISL94202_OutputValues BMSMeasurements;


//***********************************************************************
// Register definitions

	#define Vcell_OV_LSB_ADDR 0x00
	#define Vcell_OV_MSB_ADDR 0x01

	#define Vcell_OVR_LSB_ADDR 0x02
	#define Vcell_OVR_MSB_ADDR 0x03

	#define Vcell_UV_LSB_ADDR 0x04
	#define Vcell_UV_MSB_ADDR 0x05

	#define Vcell_UVR_LSB_ADDR 0x06
	#define Vcell_UVR_MSB_ADDR 0x07

	#define Vcell_OVLO_LSB_ADDR 0x08
	#define Vcell_OVLO_MSB_ADDR 0x09

	#define Vcell_UVLO_LSB_ADDR 0x0A
	#define Vcell_UVLO_MSB_ADDR 0x0B

	#define Vcell_EOC_LSB_ADDR 0x0C
	#define Vcell_EOC_MSB_ADDR 0x0D

	#define Vcell_LVCL_LSB_ADDR 0x0E
	#define Vcell_LVCL_MSB_ADDR 0x0F

	#define Vcell_OVDT_LSB_ADDR 0x10
	#define Vcell_OVDT_MSB_ADDR 0x11

	#define Vcell_UVDT_LSB_ADDR 0x12
	#define Vcell_UVDT_MSB_ADDR 0x13

	#define OWT_LSB_ADDR 0x14
	#define OWT_MSB_ADDR 0x15

	#define DOCT_LSB_ADDR 0x16
	#define DOCT_MSB_ADDR 0x17

	#define COCT_LSB_ADDR 0x18
	#define COCT_MSB_ADDR 0x19

	#define DSCT_LSB_ADDR 0x1A
	#define DSCT_MSB_ADDR 0x1B

	#define CBMIN_LSB_ADDR 0x1C
	#define CBMIN_MSB_ADDR 0x1D

	#define CBMAX_LSB_ADDR 0x1E
	#define CBMAX_MSB_ADDR 0x1F

	#define CBMIND_LSB_ADDR 0x20
	#define CBMIND_MSB_ADDR 0x21

	#define CBMAXD_LSB_ADDR 0x22
	#define CBMAXD_MSB_ADDR 0x23

	#define CBON_LSB_ADDR 0x24
	#define CBON_MSB_ADDR 0x25

	#define CBOFF_LSB_ADDR 0x26
	#define CBOFF_MSB_ADDR 0x27

	#define CBUT_LSB_ADDR 0x28
	#define CBUT_MSB_ADDR 0x29

	#define CBUTR_LSB_ADDR 0x2A
	#define CBUTR_MSB_ADDR 0x2B

	#define CBOT_LSB_ADDR 0x2C
	#define CBOT_MSB_ADDR 0x2D

	#define CBOTR_LSB_ADDR 0x2E
	#define CBOTR_MSB_ADDR 0x2F

	#define COT_LSB_ADDR 0x30
	#define COT_MSB_ADDR 0x31

	#define COTR_LSB_ADDR 0x32
	#define COTR_MSB_ADDR 0x33

	#define CUT_LSB_ADDR 0x34
	#define CUT_MSB_ADDR 0x35

	#define CUTR_LSB_ADDR 0x36
	#define CUTR_MSB_ADDR 0x37

	#define DOT_LSB_ADDR 0x38
	#define DOT_MSB_ADDR 0x39

	#define DOTR_LSB_ADDR 0x3A
	#define DOTR_MSB_ADDR 0x3B

	#define DUT_LSB_ADDR 0x3C
	#define DUT_MSB_ADDR 0x3D

	#define DUTR_LSB_ADDR 0x3E
	#define DUTR_MSB_ADDR 0x3F

	#define IOT_LSB_ADDR 0x40
	#define IOT_MSB_ADDR 0x41

	#define IOTR_LSB_ADDR 0x42
	#define IOTR_MSB_ADDR 0x43

	#define Vcell_SLV_LSB_ADDR 0x44
	#define Vcell_SLV_MSB_ADDR 0x45

	#define SDT_LSB_ADDR 0x46
	#define SDT_MSB_ADDR 0x47

	#define MODE_T_ADDR 0x48

	#define CELL_S_ADDR 0x49
	#define SETUP0_ADDR 0x4A
	#define SETUP1_ADDR 0x4B

	struct BMSConfigRegsStruct{

			uint8_t Vcell_OV_LSB;
			uint8_t Vcell_OV_MSB;

			uint8_t Vcell_OVR_LSB;
			uint8_t Vcell_OVR_MSB;

			uint8_t Vcell_UV_LSB;
			uint8_t Vcell_UV_MSB;

			uint8_t Vcell_UVR_LSB;
			uint8_t Vcell_UVR_MSB;

			uint8_t Vcell_OVLO_LSB;
			uint8_t Vcell_OVLO_MSB;

			uint8_t Vcell_UVLO_LSB;
			uint8_t Vcell_UVLO_MSB;

			uint8_t Vcell_EOC_LSB;
			uint8_t Vcell_EOC_MSB;

			uint8_t Vcell_LVCL_LSB;
			uint8_t Vcell_LVCL_MSB;

			uint8_t Vcell_OVDT_LSB;
			uint8_t Vcell_OVDT_MSB;

			uint8_t Vcell_UVDT_LSB;
			uint8_t Vcell_UVDT_MSB;

			uint8_t OWT_LSB;
			uint8_t OWT_MSB;

			uint8_t DOCT_LSB;
			uint8_t DOCT_MSB;

			uint8_t COCT_LSB;
			uint8_t COCT_MSB;

			uint8_t DSCT_LSB;
			uint8_t DSCT_MSB;

			uint8_t CBMIN_LSB;
			uint8_t CBMIN_MSB;

			uint8_t CBMAX_LSB;
			uint8_t CBMAX_MSB;

			uint8_t CBMIND_LSB;
			uint8_t CBMIND_MSB;

			uint8_t CBMAXD_LSB;
			uint8_t CBMAXD_MSB;

			uint8_t CBON_LSB;
			uint8_t CBON_MSB;

			uint8_t CBOFF_LSB;
			uint8_t CBOFF_MSB;

			uint8_t CBUT_LSB;
			uint8_t CBUT_MSB;

			uint8_t CBUTR_LSB;
			uint8_t CBUTR_MSB;

			uint8_t CBOT_LSB;
			uint8_t CBOT_MSB;

			uint8_t CBOTR_LSB;
			uint8_t CBOTR_MSB;

			uint8_t COT_LSB;
			uint8_t COT_MSB;

			uint8_t COTR_LSB;
			uint8_t COTR_MSB;

			uint8_t CUT_LSB;
			uint8_t CUT_MSB;

			uint8_t CUTR_LSB;
			uint8_t CUTR_MSB;

			uint8_t DOT_LSB;
			uint8_t DOT_MSB;

			uint8_t DOTR_LSB;
			uint8_t DOTR_MSB;

			uint8_t DUT_LSB;
			uint8_t DUT_MSB;

			uint8_t DUTR_LSB;
			uint8_t DUTR_MSB;

			uint8_t IOT_LSB;
			uint8_t IOT_MSB;

			uint8_t IOTR_LSB;
			uint8_t IOTR_MSB;

			uint8_t Vcell_SLV_LSB;
			uint8_t Vcell_SLV_MSB;

			uint8_t SDT_LSB;
			uint8_t SDT_MSB;

			uint8_t MODE_T;

			uint8_t CELL_S;
			uint8_t SETUP0;
			uint8_t SETUP1;


		};

 typedef union BMSConfigRegisters{
	struct BMSConfigRegsStruct BMSConfigRegisters;
	uint8_t BMSConfigRegsArray[76];
 }BMSConfigRegs;


	#define STATUS0_ADDR 0x80
	#define STATUS1_ADDR 0x81
	#define STATUS2_ADDR 0x82
	#define STATUS3_ADDR 0x83
	#define CBFC_ADDR 0x84
	#define CONTROL0_ADDR 0x85
	#define CONTROL1_ADDR 0x86
	#define CONTROL2_ADDR 0x87
	#define CONTROL3_ADDR 0x88
	#define EEPROM_ENABLE_ADDR 0x89



	union STATUS0_REG
	{
			struct STATUS0_BITS{
				unsigned int OVF :1;
				unsigned int OVLOF :1;
				unsigned int UVF :1;
				unsigned int UVLOF :1;
				unsigned int DOTF :1;
				unsigned int DUTF :1;
				unsigned int COTF :1;
				unsigned int CUTF :1;

			}STATUS0_BIT;
			uint8_t STATUS0;
	};

	union STATUS1_REG
	{
			struct STATUS1_BITS{
				unsigned int IOTF :1;
				unsigned int COCF :1;
				unsigned int DOCF :1;
				unsigned int DSCF :1;
				unsigned int CELLF :1;
				unsigned int OPENF :1;
				unsigned int RSVD :1;
				unsigned int VEOC :1;
			}STATUS1_BIT;
			uint8_t STATUS1;
	};

	union STATUS2_REG
	{
			struct STATUS2_BITS{
				unsigned int LD_PRSNT :1;
				unsigned int CH_PRSNT :1;
				unsigned int CHING :1;
				unsigned int DCHING :1;
				unsigned int ECC_USED :1;
				unsigned int ECC_FAIL :1;
				unsigned int INT_SCAN :1;
				unsigned int LVCHG :1;
			}STATUS2_BIT;
			uint8_t STATUS2;
	};


	union STATUS3_REG
	{
			struct STATUS3_BITS{
				unsigned int CBOT :1;
				unsigned int CBUT :1;
				unsigned int CBOV :1;
				unsigned int CBUV :1;
				unsigned int IDLE :1;
				unsigned int DOZE :1;
				unsigned int SLEEP :1;
				unsigned int RSVD :1;
			}STATUS3_BIT;
			uint8_t STATUS3;
	};

	union CONTROL0_REG
	{
			struct CONTROL0_BITS{
				unsigned int AMUXOUT :4;
				unsigned int CG:2;
				unsigned int ADC_START:1;
				unsigned int RSVD:1;


			}CONTROL0_BIT;
			uint8_t CONTROL0;
	};

	union CONTROL1_REG
	{
			struct CONTROL1_BITS{
				unsigned int DFET :1;
				unsigned int CFET :1;
				unsigned int PCFET :1;
				unsigned int PSD :1;
				unsigned int CMON_EN :1;
				unsigned int CLR_CERR :1;
				unsigned int LMON_EN :1;
				unsigned int CLR_LERR :1;
			}CONTROL1_BIT;
			uint8_t CONTROL1;
	};

	union CONTROL2_REG
	{
			struct CONTROL2_BITS{
				unsigned int CBAL_ON :1;
				unsigned int OW_STRT :1;
				unsigned int uC_SCAN :1;
				unsigned int uC_CMON :1;
				unsigned int uC_LMON :1;
				unsigned int uC_CBAL :1;
				unsigned int uC_FET :1;
				unsigned int RSVD :1;
			}CONTROL2_BIT;
			uint8_t CONTROL2;
	};

	union CONTROL3_REG
	{
			struct CONTROL3_BITS{
				unsigned int IDLE :1;
				unsigned int DOZE :1;
				unsigned int SLEEP :1;
				unsigned int PDWN :1;
				unsigned int RSVD :4;

			}CONTROL3_BIT;
			uint8_t CONTROL3;
	};



	struct BMSOperationRegs{

		union STATUS0_REG STATUS0;
		union STATUS1_REG STATUS1;
		union STATUS2_REG STATUS2;
		union STATUS3_REG STATUS3;
		uint8_t CBFC;
		union CONTROL0_REG CONTROL0;
		union CONTROL1_REG CONTROL1;
		union CONTROL2_REG CONTROL2;
		union CONTROL3_REG CONTROL3;
		uint8_t EEPROM_ENABLE;

	};


extern struct BMSOperationRegs BMSOpRegs;
#define CELLMIN_LSB_ADDR 0x8A
#define CELLMIN_MSB_ADDR 0x8B

#define CELLMAX_LSB_ADDR 0x8C
#define CELLMAX_MSB_ADDR 0x8D

#define IPACK_LSB_ADDR 0x8E
#define IPACK_MSB_ADDR 0x8F

#define VCELL1_LSB_ADDR 0x90
#define VCELL1_MSB_ADDR 0x91

#define VCELL2_LSB_ADDR 0x92
#define VCELL2_MSB_ADDR 0x93

#define VCELL3_LSB_ADDR 0x94
#define VCELL3_MSB_ADDR 0x95

#define VCELL4_LSB_ADDR 0x96
#define VCELL4_MSB_ADDR 0x97

#define VCELL5_LSB_ADDR 0x98
#define VCELL5_MSB_ADDR 0x99

#define VCELL6_LSB_ADDR 0x9A
#define VCELL6_MSB_ADDR 0x9B

#define VCELL7_LSB_ADDR 0x9C
#define VCELL7_MSB_ADDR 0x9D

#define VCELL8_LSB_ADDR 0x9E
#define VCELL8_MSB_ADDR 0x9F

#define ITEMP_LSB_ADDR 0xA0
#define ITEMP_MSB_ADDR 0xA1

#define XT1_LSB_ADDR 0xA2
#define XT1_MSB_ADDR 0xA3

#define XT2_LSB_ADDR 0xA4
#define XT2_MSB_ADDR 0xA5

#define VBATT_LSB_ADDR 0xA6
#define VBATT_MSB_ADDR 0xA7

#define VRGO_LSB_ADDR 0xA8
#define VRGO_MSB_ADDR 0xA9

#define ADC_LSB_ADDR 0xAA
#define ADC_MSB_ADDR 0xAB

union BMSDataRegisters
{
	struct BMSDataRegsStruct{

		uint8_t CELLMIN_LSB;
		uint8_t CELLMIN_MSB;

		uint8_t CELLMAX_LSB;
		uint8_t CELLMAX_MSB;

		uint8_t IPACK_LSB;
		uint8_t IPACK_MSB;

		uint8_t VCELL1_LSB;
		uint8_t VCELL1_MSB;

		uint8_t VCELL2_LSB;
		uint8_t VCELL2_MSB;

		uint8_t VCELL3_LSB;
		uint8_t VCELL3_MSB;

		uint8_t VCELL4_LSB;
		uint8_t VCELL4_MSB;

		uint8_t VCELL5_LSB;
		uint8_t VCELL5_MSB;

		uint8_t VCELL6_LSB;
		uint8_t VCELL6_MSB;

		uint8_t VCELL7_LSB;
		uint8_t VCELL7_MSB;

		uint8_t VCELL8_LSB;
		uint8_t VCELL8_MSB;

		uint8_t ITEMP_LSB;
		uint8_t ITEMP_MSB;

		uint8_t XT1_LSB;
		uint8_t XT1_MSB;

		uint8_t XT2_LSB;
		uint8_t XT2_MSB;

		uint8_t VBATT_LSB;
		uint8_t VBATT_MSB;

		uint8_t VRGO_LSB;
		uint8_t VRGO_MSB;

		uint8_t ADC_LSB;
		uint8_t ADC_MSB;

	}BMSDataRegisters;

	uint8_t BMSDataRegsArray[34];
};

extern union BMSDataRegisters BMSDataRegs;
//**********************************************************

void BMS_writeByte(uint8_t Dest,uint8_t Data);
uint8_t BMS_readByte(uint8_t Dest);
void ISL94202_Init(void);
union BMSConfigRegisters ISL94202_ReadCfgBytes(void);
void ISL94202_WriteCfgBytes(union BMSConfigRegisters Config);

uint8_t ISL94202_CompareCfgBytes(union BMSConfigRegisters Config1,union BMSConfigRegisters Config2);
void ISL94202_EnableEEPROM(void);
void ISL94202_DisableEEPROM(void);


void ISL94202_ReadPackCurrent(void);
void ISL94202_ReadCellVoltages(void);
void ISL94202_ReadTemperatures(void);

void ISL94202_MinMaxVoltage(void);
void ISL94202_ReadVRGO(void);
int ISL94202_ReadAllMeasurements(void);

void ISL94202_ReadStatusRegisters(void);
void ISL94202_ReadControlRegisters(void);

void ISL94202_ControlCFET(int State);
void ISL94202_ControlDFET(int State);

void ISL94202_ControlFETs(int State);

float approximateTemp(float input, const float *RLUT,const short int *TLUT, int LUTsize) ;

#endif /* ISL94202_H_ */
