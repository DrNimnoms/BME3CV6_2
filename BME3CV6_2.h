/************************************
REVISION HISTORY
$Revision: 1000 $
$Date: 2015-06-07 

Copyright (c) 2015, General Atomics (GA)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Copyright 2015 General Atomics (GA)
***********************************************************/

/*!******************************************************************************************************
* BME3CV6_2.h
* Created by Nima Ghods
* Data: 6/8/2015
* setsup miscellaneous parameters for running BME v6.X
*********************************************************************************************************/


#ifndef BME3CV6_2_H
#define BME3CV6_2_H

//! Set "pin" low
//! @param pin pin to be driven LOW
#define output_low(pin)   digitalWrite(pin, LOW)
//! Set "pin" high
//! @param pin pin to be driven HIGH
#define output_high(pin)  digitalWrite(pin, HIGH)
//! Return the state of pin "pin"
//! @param pin pin to be read (HIGH or LOW).
//! @return the state of pin "pin"
#define input(pin)        digitalRead(pin)


/*
	Pre computed crc15 table used for the LTC6804 PEC calculation
	*/
static const unsigned int crc15Table[256] = {
0x0,0xc599, 0xceab, 0xb32, 0xd8cf, 0x1d56, 0x1664, 0xd3fd, 0xf407, 0x319e, 0x3aac,  //!<precomputed CRC15 Table
0xff35, 0x2cc8, 0xe951, 0xe263, 0x27fa, 0xad97, 0x680e, 0x633c, 0xa6a5, 0x7558, 0xb0c1, 
0xbbf3, 0x7e6a, 0x5990, 0x9c09, 0x973b, 0x52a2, 0x815f, 0x44c6, 0x4ff4, 0x8a6d, 0x5b2e,
0x9eb7, 0x9585, 0x501c, 0x83e1, 0x4678, 0x4d4a, 0x88d3, 0xaf29, 0x6ab0, 0x6182, 0xa41b,
0x77e6, 0xb27f, 0xb94d, 0x7cd4, 0xf6b9, 0x3320, 0x3812, 0xfd8b, 0x2e76, 0xebef, 0xe0dd, 
0x2544, 0x2be, 0xc727, 0xcc15, 0x98c, 0xda71, 0x1fe8, 0x14da, 0xd143, 0xf3c5, 0x365c, 
0x3d6e, 0xf8f7,0x2b0a, 0xee93, 0xe5a1, 0x2038, 0x7c2, 0xc25b, 0xc969, 0xcf0, 0xdf0d, 
0x1a94, 0x11a6, 0xd43f, 0x5e52, 0x9bcb, 0x90f9, 0x5560, 0x869d, 0x4304, 0x4836, 0x8daf,
0xaa55, 0x6fcc, 0x64fe, 0xa167, 0x729a, 0xb703, 0xbc31, 0x79a8, 0xa8eb, 0x6d72, 0x6640,
0xa3d9, 0x7024, 0xb5bd, 0xbe8f, 0x7b16, 0x5cec, 0x9975, 0x9247, 0x57de, 0x8423, 0x41ba,
0x4a88, 0x8f11, 0x57c, 0xc0e5, 0xcbd7, 0xe4e, 0xddb3, 0x182a, 0x1318, 0xd681, 0xf17b, 
0x34e2, 0x3fd0, 0xfa49, 0x29b4, 0xec2d, 0xe71f, 0x2286, 0xa213, 0x678a, 0x6cb8, 0xa921, 
0x7adc, 0xbf45, 0xb477, 0x71ee, 0x5614, 0x938d, 0x98bf, 0x5d26, 0x8edb, 0x4b42, 0x4070, 
0x85e9, 0xf84, 0xca1d, 0xc12f, 0x4b6, 0xd74b, 0x12d2, 0x19e0, 0xdc79, 0xfb83, 0x3e1a, 0x3528, 
0xf0b1, 0x234c, 0xe6d5, 0xede7, 0x287e, 0xf93d, 0x3ca4, 0x3796, 0xf20f, 0x21f2, 0xe46b, 0xef59, 
0x2ac0, 0xd3a, 0xc8a3, 0xc391, 0x608, 0xd5f5, 0x106c, 0x1b5e, 0xdec7, 0x54aa, 0x9133, 0x9a01, 
0x5f98, 0x8c65, 0x49fc, 0x42ce, 0x8757, 0xa0ad, 0x6534, 0x6e06, 0xab9f, 0x7862, 0xbdfb, 0xb6c9, 
0x7350, 0x51d6, 0x944f, 0x9f7d, 0x5ae4, 0x8919, 0x4c80, 0x47b2, 0x822b, 0xa5d1, 0x6048, 0x6b7a, 
0xaee3, 0x7d1e, 0xb887, 0xb3b5, 0x762c, 0xfc41, 0x39d8, 0x32ea, 0xf773, 0x248e, 0xe117, 0xea25, 
0x2fbc, 0x846, 0xcddf, 0xc6ed, 0x374, 0xd089, 0x1510, 0x1e22, 0xdbbb, 0xaf8, 0xcf61, 0xc453, 
0x1ca, 0xd237, 0x17ae, 0x1c9c, 0xd905, 0xfeff, 0x3b66, 0x3054, 0xf5cd, 0x2630, 0xe3a9, 0xe89b, 
0x2d02, 0xa76f, 0x62f6, 0x69c4, 0xac5d, 0x7fa0, 0xba39, 0xb10b, 0x7492, 0x5368, 0x96f1, 0x9dc3, 
0x585a, 0x8ba7, 0x4e3e, 0x450c, 0x8095}; 

static const unsigned char reverse[16] = {   // used to reverse the 4 bit numbers
0x0, 0x8, 0x4, 0xc, 0x2, 0xa, 0x6, 0xe,
0x1, 0x9, 0x5, 0xd, 0x3, 0xb, 0x7, 0xf, };

/*! 
 
  |MD| Dec  | ADC Conversion Model|
  |--|------|---------------------|
  |01| 1    | Fast 			   	  |
  |10| 2    | Normal 		      |
  |11| 3    | Filtered 		      |
*/
#define MD_FAST 1
#define MD_NORMAL 2
#define MD_FILTERED 3

/*! 
 |CH | Dec  | Channels to convert |
 |---|------|---------------------|
 |000| 0    | All Cells  		  |
 |001| 1    | Cell 1 and Cell 7   |
 |010| 2    | Cell 2 and Cell 8   |
 |011| 3    | Cell 3 and Cell 9   |
 |100| 4    | Cell 4 and Cell 10  |
 |101| 5    | Cell 5 and Cell 11  |
 |110| 6    | Cell 6 and Cell 12  |
*/

#define CELL_CH_ALL 0
#define CELL_CH_1and7 1
#define CELL_CH_2and8 2
#define CELL_CH_3and9 3
#define CELL_CH_4and10 4
#define CELL_CH_5and11 5
#define CELL_CH_6and12 6


 /*!

  |CHG | Dec  |Channels to convert   | 
  |----|------|----------------------|
  |000 | 0    | All GPIOS and 2nd Ref| 
  |001 | 1    | GPIO 1 			     |
  |010 | 2    | GPIO 2               |
  |011 | 3    | GPIO 3 			  	 |
  |100 | 4    | GPIO 4 			  	 |
  |101 | 5    | GPIO 5 			 	 |
  |110 | 6    | Vref2  			  	 |
*/

#define AUX_CH_ALL 0
#define AUX_CH_GPIO1 1
#define AUX_CH_GPIO2 2
#define AUX_CH_GPIO3 3
#define AUX_CH_GPIO4 4
#define AUX_CH_GPIO5 5
#define AUX_CH_VREF2 6


/*!

  |CHG | Dec  |Channels to convert | 
  |----|------|--------------------|
  |000 | 0    | All                | 
  |001 | 1    | SOC                |
  |010 | 2    | ITMP               |
  |011 | 3    | VA                 |
  |100 | 4    | VD                 |
*/

#define STAT_CH_ALL 0
#define STAT_CH_SOC 1
#define STAT_CH_ITMP 2
#define STAT_CH_VA 3
#define STAT_CH_VD 4


//uint8_t CHG = 0; //!< aux channels to be converted
 /*!****************************************************
  \brief Controls if Discharging transitors are enabled
  or disabled during Cell conversions.
  
 |DCP | Discharge Permitted During conversion |
 |----|----------------------------------------|
 |0   | No - discharge is not permitted         |
 |1   | Yes - discharge is permitted           |
 
********************************************************/
#define DCP_DISABLED 0
#define DCP_ENABLED 1

 /*!
  
 |COM | communiaction continues or not         |
 |----|----------------------------------------|
 |0   | continue communication                 |
 |1   | end communication                      |
 
********************************************************/
#define COM_END 1
#define COM_CONTINUE 0

 /*!
  
 |COM | communiaction continues or not         |
 |----|----------------------------------------|
 |0   | continue communication                 |
 |1   | end communication                      |
 
********************************************************/
#define GOURP_A 0
#define GOURP_B 1
#define GOURP_C 2
#define GOURP_D 3

// myTemperature reading prameters
#define IntTempConst  0.013333  //0.1mV/7.5mV is the convertion from the reading to K for internal myTemperature
#define Binv 0.00024777  // 1/ B-value where B-value is 4036 K
#define T0inv 0.003354  // 1/T_0 where T_0 is 298.15 K


// battery parameters
#define BME_NUM 4
#define VC_NUM 3 
#define BYTE_INT 2
#define TEMP_NUM 3
#define myHs_temp_NUM 4

// initialize the SPI communication with clock divider and spi mode
void bme_com_initialize(uint8_t clock_div, uint8_t spi_mode);

class bmes
{
public:
	
    //constructur
	bmes(uint8_t csPin);  
	//
	void set_adc(uint8_t md, uint8_t dcp, uint8_t ch, uint8_t chg, uint8_t chst);
	void set_limits(float limits[13]);
	// void set_vol_limits(float min_vol, float max_vol);
	
	void meas_act_bmes();
	void bme_self_test();
	void reset_flags();

	void set_flag_over(uint16_t priority2Flag);
	uint8_t set_bal2vol(float bal2vol);
	void set_tolerance(float bal_tolerance);
	void set_balancing(bool bal_on);
	
	void set_fan(uint8_t bme_addr, bool fan_on);

	uint16_t get_bme_flag(uint8_t bme_addr);
	float get_cell_vol(uint8_t bme_addr, uint8_t cell_loc);
	float get_cell_temp(uint8_t bme_addr, uint8_t cell_loc);
	float get_bme_hst(uint8_t bme_addr);
	float get_bme_Vref(uint8_t bme_addr);
	float get_bme_totalV(uint8_t bme_addr);
	float get_bme_itmp(uint8_t bme_addr);
	bool get_crc(uint8_t bme_addr);
	bool get_st_cell(uint8_t bme_addr);
	bool get_st_gpio(uint8_t bme_addr);
	bool get_st_stat(uint8_t bme_addr);

	void data_voltage(uint8_t vol_out[BME_NUM*VC_NUM*BYTE_INT]);
	void data_temp(uint8_t temp_out[BME_NUM*TEMP_NUM*BYTE_INT]);
	void data_hst(uint8_t hs_out[BME_NUM*BYTE_INT]);
	void data_vtotal(uint8_t vTotal_out[BME_NUM*BYTE_INT]);
	void data_itemp(uint8_t itmp_out[BME_NUM*BYTE_INT]);
	void data_bme_flag(uint8_t flag_out[BME_NUM*BYTE_INT]);
	void data_bme_flagOver(uint8_t flag_out[BME_NUM*BYTE_INT]);

	uint16_t cal_bme_flag();
	float cal_sum_of_bmes();
	float cal_min_of_bmes();
	float cal_max_of_bmes();

private:
	uint8_t myCS; // The chip select channel for the BMEs
	/*!
     6804 conversion command variables.  
	*/
	uint8_t ADCV[2]; // Cell myVoltage conversion command.
	uint8_t ADAX[2]; // GPIO conversion command.
	uint8_t ADSTAT[2]; // State conversion command.
	uint8_t CVST[2]; // Cell myVoltage self test command.
	uint8_t AXST[2]; // GPIO self test command.
	uint8_t STATST[2]; // State self test command.
	uint8_t DIAGN[2]; // Diagnose MUX and Poll Status
	uint16_t BME_CON_DELAY_CV;  // delay for BME ADC conversion commands for CV
	uint16_t BME_CON_DELAY_AUX;	// delay for BME ADC conversion commands for AUX
	uint16_t BME_CON_DELAY_ST;  // delay for BME ADC conversion commands for state
	uint16_t SELF_CHECK_VAL_1;  // self check 01 value
	uint16_t SELF_CHECK_VAL_2;  // self check 10 value

	// FLAG LIMITS
	float myAccuracyLimit;	//if myVol_ref2 is within myAccuracyLimit of 3V then the ADC is working
    float myVolHighAlarm;  //high virtual cell myVoltage limit for myVoltage error
    float myVolLowWarn;       //low virtual cell myVoltage limit for myVoltage warning
    float myVolLowAlarm;      //low virtual cell myVoltage limit for myVoltage error
    float myDeadBatAlarm;   // the myVoltage at whitch the system will not go in to charge mode
    float myVolBmeMismatch;       //myVoltage mismatch limit between calculated and measured total myVoltage of half-string
    float myTempVCAlarm;    //virtual cell myTemperature limit for myTemperature error
    float myTempTiAlarm;    // BME LTC chip myTemperature limit for myTemperature error
    float myTempHSAlarm;   //heat sink myTemperature limit for myTemperature error
    float myTempVCWarn;   //virtual cell myTemperature limit for myTemperature warning
    float myTempTiWarn;    //BME LTC chip myTemperature limit for myTemperature warning
    float myTempHSWarn;   //heat sink myTemperature limit for myTemperature warning
    float myLowTempAlarm;  // Low myTemperature sensors limit for low myTemperature alarm 

	//data from BME
	uint8_t vol_data[BME_NUM*VC_NUM*BYTE_INT];    // array of virtual cell myVoltages
	uint8_t temp_data[BME_NUM*TEMP_NUM*BYTE_INT];  // temperature of: 0-2 = virtual cells
	uint8_t myHs_data[BME_NUM*BYTE_INT];				// heatsink temperature data
	uint8_t ref_data[BME_NUM*BYTE_INT];			   //measured BME myVoltage reference2
	uint8_t total_vol_data[BME_NUM*BYTE_INT];		//measured total BME myVoltage
	uint8_t itmp_data[BME_NUM*BYTE_INT];			// internal myTemperature of BME LTC84 chip
	union {
		uint8_t asBytes[BYTE_INT];
		uint16_t asInt;
	} int2byte;
 	

	//float versions of all data
	float myVoltage[BME_NUM][VC_NUM];
	float myTemperature[BME_NUM][TEMP_NUM];
	float myHs_temp[BME_NUM];
	float myVol_ref2[BME_NUM];
	float myTotal_bme_vol[BME_NUM];
	float myInternal_temp[BME_NUM];

	// bme flag data
	uint16_t myBmeFlag[BME_NUM];
	uint16_t myBmeFlagOverride[BME_NUM];
	bool myCrcCheck[BME_NUM];
    bool myStCell[BME_NUM];
    bool myStGpio[BME_NUM];
    bool myStStat[BME_NUM];

	// balancing  prameters
    float myBal2Vol;	// myVoltage to balance to
    float myBalTolerance;  // balnacing tolerance
    bool myBalOn;		// controlls all fans
    uint16_t myDcc[BME_NUM];  //controlls the discharging resistors
    bool balTempCon[BME_NUM];

    // cooling prameters
    bool myFanOn[BME_NUM];		// controlls all fans
	
    // set flags
    void set_meas_flags();
    void set_st_flags();

    // calculated values
    float cal_bme_sum(uint8_t bme_addr);
	float cal_max_vol(uint8_t bme_addr);
	float cal_min_vol(uint8_t bme_addr);
	float cal_max_temp(uint8_t bme_addr);
	float cal_min_temp(uint8_t bme_addr);

    // balancing function 
	void balance_bme(uint8_t bme_addr);

	// requist and read functions   
 	uint16_t meas_vol();
	uint16_t meas_temp();
	uint16_t meas_stat();

	// self test commands
 	uint16_t bme_statst();
 	uint16_t bme_axst();
 	uint16_t bme_cvst();

 	// data conversions
 	void data2stat(uint16_t crc_check);
 	void data2temp(uint16_t crc_check);
 	void data2vol(uint16_t crc_check);
 	void int2bool(int i_max, int data_in, bool bool_out[]);
 	uint16_t float2int(float val);

 	// BME action commands
 	void bme_clr_all(); 
 	void bme_adstat();
 	void bme_adax();
	void bme_adcv();
	void bme_wrcfg(uint8_t addr);

	// wakeup commands
	void wakeup_sleep();
    void wakeup_idle();
	
	// low level send and receive commands
	void bme_write_array(uint8_t len, uint8_t *tx_data, bool com);
	bool bme_read_array(uint8_t len, uint8_t *rx_data);
	uint16_t pec15_calc(uint8_t len, uint8_t *data);
};

#endif










