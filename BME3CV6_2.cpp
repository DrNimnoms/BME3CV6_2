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
* BME3CV6_2.cpp
* Created by Nima Ghods
* Data: 6/8/2015
* setsup miscellaneous functions for running BME v6.X
*********************************************************************************************************/

#include <stdint.h>
#include <Arduino.h>
#include <SPI.h>
#include "BME3CV6_2.h"

/*!******************************************************************************************************************
 \brief This function will initialize the SPI port.

@param[in] uint8_t clock_div The SPI clock divid 
@param[in] uint8_t spi_mode The SPI communication mode  
 ******************************************************************************************************************/
void bme_com_initialize(uint8_t clock_div, uint8_t spi_mode)
{
  SPI.begin();
  SPI.setClockDivider(clock_div);
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(spi_mode); 
}

/*!******************************************************************************************************************
  bmes() constructur
  initializes all the internal bmes values
 ******************************************************************************************************************/
bmes::bmes(uint8_t csPin)
{
	myCS=csPin;

  myBal2Vol = 4.2;
  myBalTolerance = 0.002;  // the max myVoltage that is allowed durring balancing 
  myBalOn = false;
  for(int current_bme = 0; current_bme<BME_NUM; current_bme++) {
    myDcc[current_bme] = 0;
    myFanOn[current_bme] = false;
    myCrcCheck[current_bme] = false;
    myStCell[current_bme] = false;
    myStGpio[current_bme] = false;
    myStStat[current_bme] = false;
    myVol_ref2[current_bme] = 3.0;
    myTotal_bme_vol[current_bme] = 3.7*VC_NUM;
    for (int current_cell = 0; current_cell<BME_NUM; current_cell++) {
      myVoltage[current_bme][current_cell]=3.7;
    }
  }

  //BME FLAG LIMITS
    myAccuracyLimit = 0.015; //if myVol_ref2 is within myAccuracyLimit of 3V then the ADC is working
    myVolHighAlarm = 4.205;  //high virtual cell myVoltage limit for myVoltage error
    myVolLowWarn = 3.2;       //low virtual cell myVoltage limit for myVoltage warning
    myVolLowAlarm = 3.0;      //low virtual cell myVoltage limit for myVoltage error
    myDeadBatAlarm = 2.5;   // the myVoltage at which the system will not go in to charge mode
    myVolMismatch  = 0.06;   //myVoltage mismatch limit between calculated and measured total myVoltage of battery module
    myTempVCAlarm = 60;    //virtual cell myTemperature limit for myTemperature error
    myTempTiAlarm = 75;    // BME LTC chip myTemperature limit for myTemperature error
    myTempHSAlarm = 120;   //heat sink myTemperature limit for myTemperature error
    myTempVCWarn = 40;    //virtual cell myTemperature limit for myTemperature warning
    myTempTiWarn = 65;    //BME LTC chip myTemperature limit for myTemperature warning
    myTempHSWarn = 110;   //heat sink myTemperature limit for myTemperature warning
    myLowTempAlarm = -40; // Low myTemperature sensors limit for low myTemperature alarm 
    
}

/*!****************************************************
  \brief measured all BME data
 
  Collects myVoltage, status and myTemperature readings of the BME's.
*****************************************************/
void bmes::meas_act_bmes(){
  uint16_t tempo;
  uint16_t crc_tempo=0;
  wakeup_sleep();

  tempo = meas_vol();
  crc_tempo = tempo;
  data2vol(tempo);
  tempo = meas_temp();
  crc_tempo |= tempo;
  data2temp(tempo);
  tempo = meas_stat();
  data2stat(tempo);
  crc_tempo |= tempo;

  set_meas_flags();  // set bme flags for warnigs and alarms 

  for(int current_bme = 0; current_bme<BME_NUM; current_bme++){
    myDcc[current_bme]=0;
    if(myBalOn) balance_bme(current_bme);
    bme_wrcfg(current_bme);
  } 

  int2bool(BME_NUM,crc_tempo, myCrcCheck);
}

/*!****************************************************
  \brief calculates which cells need to be reduced to balance the string
 puts all the information in myDcc
*****************************************************/
void bmes::balance_bme(uint8_t bme_addr){

  if(!myCrcCheck[bme_addr] && !balTempCon[bme_addr]){
    for(int current_cell=0;current_cell<VC_NUM;current_cell++){
      if(myVoltage[bme_addr][current_cell] - myBal2Vol > myBalTolerance){
        myDcc[bme_addr] |= (1<<(VC_NUM-current_cell-1));// balance by enabling the bit flag corresponding to the i-th virtual layer
      }  
    }
  }
}


/*!****************************************************
  \brief self-test all BMEs
 
  clears the ADC registers and performs a self-test on each BME
  checks the digital filter, memory, and the MUX of each BME
*****************************************************/
void bmes::bme_self_test(){
  uint16_t tempo;
  wakeup_sleep();    // wake the bmes
  bme_clr_all();      // clear all BME registers
  tempo = bme_cvst();
  int2bool(BME_NUM,tempo, myStCell);
  delay(2);
  tempo = bme_axst();
  int2bool(BME_NUM,tempo, myStGpio);
  delay(2);
  tempo = bme_statst();
  int2bool(BME_NUM,tempo, myStStat);
  
  // bme_clr_all();  
  // delay(2);
  // meas_vol();
  // meas_temp();
  // meas_stat();
  // meas_act_bmes();
  bme_adax();     // ? needed to reboot aux reg after a self test
  // // delayMicroseconds(BME_CON_DELAY_AUX);
   bme_adcv();     // ? needed to reboot aux reg after a self test
  // // delayMicroseconds(BME_CON_DELAY_CV);
  // bme_adstat();
  // // delayMicroseconds(BME_CON_DELAY_ST);

  set_st_flags(); // set BME self test flags
}

/*!******************************************************************************************************************
 \brief sets the bmes to balance
 @param[in] float bal2vol the myVoltage that the bmes should balance to

 @return int8_t error
 
 0: No PEC error detected
 -1: PEC error detected, retry read
 ******************************************************************************************************************/
uint8_t bmes::set_bal2vol(float bal2vol){

  for(int current_bme = 0; current_bme<BME_NUM; current_bme++) myDcc[current_bme]=0;
  if(bal2vol > 2.7){
    myBal2Vol = bal2vol;
    return 0;
  }
  else return -1;
}

/*!******************************************************************************************************************
 \brief sets the bmes to balancing tolerance
 @param[in] float bal_tolerance the myVoltage that the bmes should balance to

 ******************************************************************************************************************/
void bmes::set_tolerance(float bal_tolerance){
  myBalTolerance = bal_tolerance;
}


/*!******************************************************************************************************************
 \brief set the fans on or off

 @param[in] uint8_t bme_addr,  the BME address
 @param[in] bool bal_on true turns the balancing on and false turns the balancing off

 ******************************************************************************************************************/
void bmes::set_balancing(bool bal_on){ 
  myBalOn= bal_on;
  if(!bal_on) myBal2Vol = 4.2;
}

/*!******************************************************************************************************************
 \brief set the fans on or off

 @param[in] uint8_t bme_addr,  the BME address
 @param[in] bool fan_on true turns the fans on and false turns the fans off
 ******************************************************************************************************************/
void bmes::set_fan(uint8_t bme_addr, bool fan_on){
  
  if(bme_addr >= BME_NUM | bme_addr<0){
    for(int current_bme = 0; current_bme<BME_NUM; current_bme++) myFanOn[current_bme] = fan_on;
  }
  else myFanOn[bme_addr] = fan_on;
}

/*!******************************************************************************************************************
 \brief for sending myVoltage data in byte form

 @param[out] uint8_t vol_out[],  the bme myVoltages in byte form
 ******************************************************************************************************************/
void bmes::data_voltage(uint8_t vol_out[BME_NUM*VC_NUM*BYTE_INT]){
  for(int i = 0; i<BME_NUM*VC_NUM*BYTE_INT; i++) vol_out[i] = vol_data[i];
}

/*!******************************************************************************************************************
 \brief for sending myTemperature data in byte form

 @param[out] uint8_t temp_out[],  the bme myTemperature in byte form
 ******************************************************************************************************************/
void bmes::data_temp(uint8_t temp_out[BME_NUM*TEMP_NUM*BYTE_INT]){
  for(int current_bme=0; current_bme<BME_NUM;current_bme++){
    for(int current_temp=0;current_temp<TEMP_NUM;current_temp++){
      int2byte.asInt=float2int(myTemperature[current_bme][current_temp]+273);
      for(int k=0;k<BYTE_INT;k++) temp_out[2*current_bme*TEMP_NUM+2*current_temp+k]=int2byte.asBytes[k];
    }
  }
  // for(int i = 0; i<BME_NUM*VC_NUM*BYTE_INT; i++) temp_out[i] = temp_data[i];
}

/*!******************************************************************************************************************
 \brief turns a float to an int and keeps two decimal places
 @param[in] float val,  the value in float form
 @return uint16_t value,  the float value *100 in uint16_t form
 ******************************************************************************************************************/

uint16_t bmes::float2int(float val){
  uint16_t int_val=0;
  if(val>655) int_val=655;
  else if(val<0) int_val=0;
  else int_val=(uint16_t)(val*100); 
  return int_val;
}

/*!******************************************************************************************************************
 \brief for sending heat sink temperature data in byte form

 @param[out] uint8_t temp_out[],  the bme heat sink temperature in byte form
 ******************************************************************************************************************/
void bmes::data_hst(uint8_t hs_out[BME_NUM*BYTE_INT]){
  for(int current_bme=0; current_bme<BME_NUM;current_bme++){
    int2byte.asInt=float2int(myHs_temp[current_bme]+273);
    for(int k=0;k<BYTE_INT;k++) hs_out[2*current_bme+k]=int2byte.asBytes[k];
  }
  // for(int i = 0; i<BME_NUM*BYTE_INT; i++) hs_out[i] = myHs_temp[i];
}

/*!******************************************************************************************************************
 \brief for sending total measured bme voltage data in byte form

 @param[out] uint8_t vTotal_out[],  the bme total measured voltage in byte form
 ******************************************************************************************************************/
void bmes::data_vtotal(uint8_t vTotal_out[BME_NUM*BYTE_INT]){
  for(int i = 0; i<BME_NUM*BYTE_INT; i++) vTotal_out[i] = total_vol_data[i];
}

/*!******************************************************************************************************************
 \brief for sending internal chip temperature data in byte form

 @param[out] uint8_t itmp_out[],  the bme internal chip temperature in byte form
 ******************************************************************************************************************/
void bmes::data_itemp(uint8_t itmp_out[BME_NUM*BYTE_INT]){
  for(int i = 0; i<BME_NUM*BYTE_INT; i++) itmp_out[i] = itmp_data[i];
}

/*!******************************************************************************************************************
 \brief for sending bme flag data in byte form

 @param[out] uint8_t flag_out[],  the bme flag in byte form
 ******************************************************************************************************************/
void bmes::data_bme_flag(uint8_t flag_out[BME_NUM*BYTE_INT]){
  for(int current_bme=0; current_bme<BME_NUM;current_bme++){
    int2byte.asInt=myBmeFlag[current_bme];
    for(int k=0;k<BYTE_INT;k++) flag_out[2*current_bme+k]=int2byte.asBytes[k];
  }
}

/*!******************************************************************************************************************
 \brief for sending bme flagoverride data in byte form

 @param[out] uint8_t flag_out[],  the bme flagoverride in byte form
 ******************************************************************************************************************/
void bmes::data_bme_flagOver(uint8_t flag_out[BME_NUM*BYTE_INT]){
  for(int current_bme=0; current_bme<BME_NUM;current_bme++){
    int2byte.asInt=myBmeFlagOverride[current_bme];
    for(int k=0;k<BYTE_INT;k++) flag_out[2*current_bme+k]=int2byte.asBytes[k];
  }
}

/*!******************************************************************************************************************
 \brief get the bme flag for a given BME
 @param[in] uint8_t bme_addr,  the BME address

 @return float myVoltage,  the myVoltage referance 2 of the given bme
 ******************************************************************************************************************/
uint16_t bmes::get_bme_flag(uint8_t bme_addr){
  if(bme_addr<BME_NUM && bme_addr>= 0) return myBmeFlag[bme_addr];
  else return -1;
}

/*!******************************************************************************************************************
 \brief get a cell myVoltage
 @param[in] uint8_t bme_addr,  the BME address
 @param[in] uint8_t cell_loc,  the cell location

 @return float myVoltage,  the cell myVoltages of the given bme at a cell location
 ******************************************************************************************************************/
float bmes::get_cell_vol(uint8_t bme_addr, uint8_t cell_loc){
  if(bme_addr<BME_NUM && bme_addr>= 0 && cell_loc<VC_NUM && cell_loc>=0) return myVoltage[bme_addr][cell_loc];
  else return -1;
}

/*!******************************************************************************************************************
 \brief get a cell myTemperature
 @param[in] uint8_t bme_addr,  the BME address
 @param[in] uint8_t temp_loc,  the temp location

 @return float myTemperature,  the cell myTemperature of the given bme at a myTemperature location 
 ******************************************************************************************************************/
float bmes::get_cell_temp(uint8_t bme_addr, uint8_t temp_loc){
  if(bme_addr<BME_NUM && bme_addr>= 0 && temp_loc<TEMP_NUM && temp_loc>=0) return myTemperature[bme_addr][temp_loc];
  else return -273;
}

/*!******************************************************************************************************************
 \brief get a heat sink myTemperature
 @param[in] uint8_t bme_addr,  the BME address

 @return float myTemperature, the heat sink myTemperature of the given bme
 ******************************************************************************************************************/
float bmes::get_bme_hst(uint8_t bme_addr){
  if(bme_addr<BME_NUM && bme_addr>= 0) return myHs_temp[bme_addr];
  else return -273;
}

/*!******************************************************************************************************************
 \brief get a myVoltage referance 2
 @param[in] uint8_t bme_addr,  the BME address

 @return float myVoltage,  the myVoltage referance 2 of the given bme
 ******************************************************************************************************************/
float bmes::get_bme_Vref(uint8_t bme_addr){
  if(bme_addr<BME_NUM && bme_addr>= 0) return myVol_ref2[bme_addr];
  else return -1;
}

/*!******************************************************************************************************************
 \brief get a total measured bme myVoltage
 @param[in] uint8_t bme_addr,  the BME address

 @return float myVoltage,  the total myVoltage of the given bme
 ******************************************************************************************************************/
float bmes::get_bme_totalV(uint8_t bme_addr){
  if(bme_addr<BME_NUM && bme_addr>= 0) return myTotal_bme_vol[bme_addr];
  else return -1;
}

/*!******************************************************************************************************************
 \brief get a internal chip myTemperature
 @param[in] uint8_t bme_addr,  the BME address

 @return float myTemperature, the internal chip myTemperature of the given bme
 ******************************************************************************************************************/
float bmes::get_bme_itmp(uint8_t bme_addr){
  if(bme_addr<BME_NUM && bme_addr>= 0) return myInternal_temp[bme_addr];
  else return -273;
}

/*!******************************************************************************************************************
 \brief get crc check 
 @param[in] uint8_t bme_addr,  the BME address

 @return bool check, the crc check of the given bme
 ******************************************************************************************************************/
bool bmes::get_crc(uint8_t bme_addr){
  if(bme_addr<BME_NUM && bme_addr>= 0) return myCrcCheck[bme_addr];
  else return true;
}

/*!******************************************************************************************************************
 \brief get cell myVoltage self test 
 @param[in] uint8_t bme_addr,  the BME address

 @return bool check, the cell myVoltage self test of the given bme
 ******************************************************************************************************************/
bool bmes::get_st_cell(uint8_t bme_addr){
  if(bme_addr<BME_NUM && bme_addr>= 0) return myStCell[bme_addr];
  else return true;
}

/*!******************************************************************************************************************
 \brief get GPIO self test 
 @param[in] uint8_t bme_addr,  the BME address

 @return bool check, the GPIO self test of the given bme
 ******************************************************************************************************************/
bool bmes::get_st_gpio(uint8_t bme_addr){
  if(bme_addr<BME_NUM && bme_addr>= 0) return myStGpio[bme_addr];
  else return true;
}

/*!******************************************************************************************************************
 \brief get status self test 
 @param[in] uint8_t bme_addr,  the BME address

 @return bool check, the status self test of the given bme
 ******************************************************************************************************************/
bool bmes::get_st_stat(uint8_t bme_addr){
  if(bme_addr<BME_NUM && bme_addr>= 0) return myStStat[bme_addr];
  else return true;
}

/*!******************************************************************************************************************
 \brief calculate the active flags that have not been overridden

 @return uint16_t bmeFlags,  the active flags for this bme channal
 ******************************************************************************************************************/
uint16_t bmes::cal_bme_flag(){ 
  uint16_t bmeFlag=0;
  
  for(uint8_t current_bme = 0; current_bme<BME_NUM; current_bme++){
    bmeFlag |= (myBmeFlag[current_bme] & ~myBmeFlagOverride[current_bme]);
  }
  return bmeFlag;
}

/*!******************************************************************************************************************
 \brief calculate the sum of layer voltages on the bme channal

 @return float voltage,  the sum of all the cell myVoltages
 ******************************************************************************************************************/
float bmes::cal_sum_of_bmes(){ 
  float sumVol=0;
  
  for(uint8_t current_bme = 0; current_bme<BME_NUM; current_bme++){
    sumVol += cal_bme_sum(current_bme);
  }
  return sumVol;
}

/*!******************************************************************************************************************
 \brief calculate the min of layer voltages on the bme channal

 @return float voltage,  the sum of all the cell myVoltages
 ******************************************************************************************************************/
float bmes::cal_min_of_bmes(){ 
  float minVol=cal_min_vol(0);
  float minTempo=0;

  for(uint8_t current_bme = 1; current_bme<BME_NUM; current_bme++){
    minTempo = cal_min_vol(current_bme);
    if(minTempo < minVol) minVol= minTempo;
  }
  return minVol;
}

/*!******************************************************************************************************************
 \brief calculate the max of layer voltages on the bme channal

 @return float voltage,  the sum of all the cell myVoltages
 ******************************************************************************************************************/
float bmes::cal_max_of_bmes(){ 
  float maxVol=cal_max_vol(0);
  float maxTempo=0;

  for(uint8_t current_bme = 1; current_bme<BME_NUM; current_bme++){
    maxTempo = cal_max_vol(current_bme);
    if(maxTempo > maxVol) maxVol = maxTempo;
  }
  return maxVol;
}

/*!******************************************************************************************************************
 \brief calculate the sum myVoltage on a bme 
 @param[in] uint8_t bme_addr,  the BME address

 @return float myVoltage,  the sum of the bme cell myVoltages
 ******************************************************************************************************************/
float bmes::cal_bme_sum(uint8_t bme_addr){ 
  float sumVol=0;

  for(uint8_t current_cell = 0; current_cell<VC_NUM; current_cell++){
    sumVol += myVoltage[bme_addr][current_cell];
  }
  return sumVol;
}

/*!******************************************************************************************************************
 \brief calculate the max myVoltage on a bme
 @param[in] uint8_t bme_addr,  the BME address

 @return float myVoltage,  the max cell myVoltages
 ******************************************************************************************************************/
float bmes::cal_max_vol(uint8_t bme_addr){ 
  float maxVol=myVoltage[bme_addr][0];
  
  for(uint8_t current_cell = 1; current_cell<VC_NUM; current_cell++){
    if(myVoltage[bme_addr][current_cell] > maxVol) maxVol=myVoltage[bme_addr][current_cell];
  }
  return maxVol;
}

/*!******************************************************************************************************************
 \brief calculate the min myVoltage on a bme
 @param[in] uint8_t bme_addr,  the BME address

 @return float myVoltage,  the min cell myVoltages
 ******************************************************************************************************************/
float bmes::cal_min_vol(uint8_t bme_addr){ 
  float minVol=myVoltage[bme_addr][0];

  for(uint8_t current_cell = 1; current_cell<VC_NUM; current_cell++){
    if(myVoltage[bme_addr][current_cell] < minVol) minVol=myVoltage[bme_addr][current_cell];
  }
  return minVol;
}

/*!******************************************************************************************************************
 \brief calculate the max myTemperature on a bme
 @param[in] uint8_t bme_addr,  the BME address

 @return float myVoltage,  the max cell myTemperature
 ******************************************************************************************************************/
float bmes::cal_max_temp(uint8_t bme_addr){ 
  float maxTemp=myTemperature[bme_addr][0];

  for(uint8_t current_temp = 1; current_temp<TEMP_NUM; current_temp++){
    if(myTemperature[bme_addr][current_temp] > maxTemp) maxTemp=myTemperature[bme_addr][current_temp];
  }
  return maxTemp;
}

/*!******************************************************************************************************************
 \brief calculate the max myTemperature on a bme
 @param[in] uint8_t bme_addr,  the BME address

 @return float myVoltage,  the max cell myTemperature
 ******************************************************************************************************************/
float bmes::cal_min_temp(uint8_t bme_addr){ 
  float minTemp=myTemperature[bme_addr][0];

  for(uint8_t current_temp = 1; current_temp<TEMP_NUM; current_temp++){
    if(myTemperature[bme_addr][current_temp] < minTemp) minTemp=myTemperature[bme_addr][current_temp];
  }
  if(myHs_temp[bme_addr] < minTemp) minTemp = myHs_temp[bme_addr];
  if(myInternal_temp[bme_addr] < minTemp) minTemp = myInternal_temp[bme_addr];
  return minTemp;
}

/*!******************************************************************************************************************
 \brief Maps ADC control variables to the appropriate control bytes for each of the different ADC commands
 
@param[in] uint8_t MD The adc conversion mode
@param[in] uint8_t DCP Controls if Discharge is permitted during cell conversions
@param[in] uint8_t CH Determines which cells are measured during an ADC conversion command
@param[in] uint8_t CHG Determines which GPIO channels are measured during Auxiliary conversion command
 
 Command Code: \n
     |command  |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   | 
     |-----------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
     |ADCV:      |   0   |   1   | MD[1] | MD[2] |   1   |   1   |  DCP  |   0   | CH[2] | CH[1] | CH[0] | 
     |ADAX:      |   1   |   0   | MD[1] | MD[2] |   1   |   1   |  DCP  |   0   | CHG[2]| CHG[1]| CHG[0]| 
 ******************************************************************************************************************/
void bmes::set_adc(uint8_t md, //ADC Mode
      uint8_t dcp, //Discharge Permit
      uint8_t ch, //Cell Channels to be measured
      uint8_t chg, //GPIO Channels to be measured
      uint8_t chst //state Channels to be measured
      )
{
  uint8_t md_bits;
  
  md_bits = (md & 0x02) >> 1;
  ADCV[0] = md_bits + 0x02;
  md_bits = (md & 0x01) << 7;
  ADCV[1] =  md_bits + 0x60 + (dcp<<4) + ch;
 
  md_bits = (md & 0x02) >> 1;
  ADAX[0] = md_bits + 0x04;
  md_bits = (md & 0x01) << 7;
  ADAX[1] = md_bits + 0x60 + chg ;

  md_bits = (md & 0x02) >> 1;
  ADSTAT[0] = md_bits + 0x04;
  md_bits = (md & 0x01) << 7;
  ADSTAT[1] = md_bits + 0x68 + chst ;

  md_bits = (md & 0x02) >> 1;
  CVST[0] = md_bits + 0x02;
  md_bits = (md & 0x01) << 7;
  CVST[1] = md_bits + 0x27;

  md_bits = (md & 0x02) >> 1;
  AXST[0] = md_bits + 0x04;
  md_bits = (md & 0x01) << 7;
  AXST[1] = md_bits + 0x47;
  
  md_bits = (md & 0x02) >> 1;
  STATST[0] = md_bits + 0x04;
  md_bits = (md & 0x01) << 7;
  STATST[1] = md_bits + 0x2F;

  DIAGN[0]=0x07;
  DIAGN[1]=0x15;

  if(md==MD_FAST){
    BME_CON_DELAY_CV = 1100;   
    BME_CON_DELAY_AUX = 1500;
    BME_CON_DELAY_ST = 748; 
    SELF_CHECK_VAL_1 = 0x9565; 
    SELF_CHECK_VAL_2 = 0x6A9A;
  }
  else if(md==MD_NORMAL){
    BME_CON_DELAY_CV = 2300;
    BME_CON_DELAY_AUX = 3000;  
    BME_CON_DELAY_ST = 2000;  
    SELF_CHECK_VAL_1 = 0x9555;  
    SELF_CHECK_VAL_2 = 0x6AAA;
  }

  // initilize the chip select of the BMEs
  pinMode(myCS, OUTPUT);           // set pin to output
  digitalWrite(myCS, HIGH);
  //make sure balancing resistors and fans are off
  wakeup_sleep();    // wake the bmes
  for(int current_bme = 0; current_bme<BME_NUM; current_bme++) myDcc[current_bme]=0;
  bme_wrcfg(16);

}


/*!****************************************************
  \brief Wake the LTC6804 from the sleep state
  
 Generic wakeup commannd to wake the LTC6804 from sleep
 *****************************************************/
void bmes::wakeup_sleep()
{
  output_low(myCS);
  delay(1); // Guarantees the LTC6804 will be in standby
  output_high(myCS);
}

/***********************************************************//**
 \brief Clears all registers on the BMEs
 
 The command clears all registers and intiallizes 
 all values to 1. The register will read back hexadecimal 0xFF
 after the command is sent.
***************************************************************/
void bmes::bme_clr_all()
{
  uint8_t cmd[2];
  
  cmd[0] = 0x07;
  cmd[1] = 0x11;
  wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake.This command can be removed.
  bme_write_array(2,cmd,COM_END);  // send broadcast clrcell command to the BME's
  cmd[1] = 0x12;
  bme_write_array(2,cmd,COM_END);  // send broadcast clraux command to the BME's
  cmd[1] = 0x13;
  bme_write_array(2,cmd,COM_END); // send broadcast clrstat command to the BME's

}

/***********************************************************//**
 \brief self tests the digital filter and memory of 
 SATA readings and the multiplexer of all BMEs 
 
 The command starts self test on digital filter and memory of 
 SATA readings and proper operation of each multiplexer channel. 
 Reads self-test results from bme_num of LTC6804-2 chips. 
 Returns the results in a 16 bit number.

@param[in] uint8_t lt_cs; This is the chip select for the BMEs
@param[in] uint8_t total_ic; This is the number of BMEs in the Channel

@return int16_t, self-test results.
***************************************************************/
uint16_t bmes::bme_statst()
{
  uint16_t self_test_data=0;
  uint8_t temp_data_array[12];
  uint16_t temp_data=0;
  bool temp_chck[BME_NUM]={0};
  uint8_t cmd[2]={0x07,0x15};
  
  wakeup_idle(); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.
  bme_write_array(2,STATST,COM_END);  // send broadcast command to LTC6804 stack
  // 4
  delayMicroseconds(BME_CON_DELAY_ST);

  bme_write_array(2,cmd,COM_END);  // send broadcast command to LTC6804 stack
  // 4
  delayMicroseconds(BME_CON_DELAY_CV);
  
  wakeup_idle(); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.

  for(int current_bme = 0; current_bme<BME_NUM; current_bme++)
  {
    cmd[0]=0x80 + (reverse[current_bme]<<3);
    for(int current_reg=GOURP_A;current_reg <= GOURP_B;current_reg++)
    {
      if(!temp_chck[current_bme])
      {
        cmd[1]=(0x08 + current_reg)<<1;
        bme_write_array(2,&cmd[0],COM_CONTINUE);// send the command reading configuration register group
        temp_chck[current_bme] = bme_read_array(6, &temp_data_array[current_reg*6]); // read configuration register group and return CRC
      }
    }
    if(!temp_chck[current_bme])
    {
      for(int j=0;j<4;j++)
      {
        temp_data=((temp_data_array[2*j+1]<<8) | temp_data_array[2*j]);
        if(temp_data != SELF_CHECK_VAL_1) self_test_data |= (1<<current_bme);
      }
      if(((temp_data_array[11]>>1) & 0x01)==1) self_test_data |= (1<<current_bme);
    }
  }
  return self_test_data;
}

/***********************************************************//**
 \brief self tests the digital filter and memory of 
 GPIO readings of all BMEs 
 
 The command starts self test on digital filter and memory of 
 GPIO readings. Reads self-test results from bme_num of 
 LTC6804-2 chips. Returns the results in a 16 bit number.

@param[in] uint8_t lt_cs; This is the chip select for the BMEs
@param[in] uint8_t total_ic; This is the number of BMEs in the Channel

@return int16_t, self-test results.
***************************************************************/
uint16_t bmes::bme_axst()
{
  uint16_t self_test_data=0;
  uint8_t cmd[2];
  uint8_t temp_data_array[6];
  uint16_t temp_data=0;
  bool temp_chck[BME_NUM]={0};
  

  wakeup_idle(); //This will guarantee that the LTC6804 isoSPI port is awake.This command can be removed.
  
  bme_write_array(2,AXST,COM_END);// send broadcast aux selftest command to the BMEs
  delayMicroseconds(BME_CON_DELAY_AUX);

  wakeup_idle(); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.

  for(int current_bme = 0; current_bme<BME_NUM; current_bme++)
  {
    cmd[0]=0x80 + (reverse[current_bme]<<3);
    for(int current_reg=GOURP_A;current_reg <= GOURP_B;current_reg++)
    {
      if(!temp_chck[current_bme])
      {
        cmd[1]=(0x06 + current_reg)<<1;
        bme_write_array(2,&cmd[0],COM_CONTINUE);// send the command reading configuration register group
        temp_chck[current_bme] = bme_read_array(6, &temp_data_array[0]); // read configuration register group and return CRC
        if(!temp_chck[current_bme])
        {
          for(int j=0;j<3;j++)
          {
            temp_data=((temp_data_array[2*j+1]<<8) | temp_data_array[2*j]);
            if(temp_data != SELF_CHECK_VAL_2) self_test_data= self_test_data| (1<<current_bme);
          }
        }
      }
      delay(1);
    }
  }
  return self_test_data;
}

/***********************************************************//**
 \brief self tests the digital filter and memory of 
 myVoltage readings of all BMEs 

 The command starts self test on digital filter and memory of 
 myVoltage readings. Reads self-test results from bme_num of 
 LTC6804-2 chips. Returns the results in a 16 bit number.

@param[in] uint8_t lt_cs; This is the chip select for the BMEs
@param[in] uint8_t total_ic; This is the number of BMEs in the Channel

@return int16_t, self-test results.
***************************************************************/
uint16_t bmes::bme_cvst()
{
  uint8_t cmd[2];
  uint16_t self_test_data=0;
  uint8_t temp_data_array[6];
  uint16_t temp_data=0;
  bool temp_chck[BME_NUM]={0};
  
  wakeup_idle(); //This will guarantee that the LTC6804 isoSPI port is awake.This command can be removed.
  bme_write_array(2,CVST,COM_END);  // send broadcast clrcell command to LTC6804 stack
  delayMicroseconds(BME_CON_DELAY_CV);

  wakeup_idle(); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.

  for(int current_bme = 0; current_bme<BME_NUM; current_bme++)
  {
    cmd[0]=0x80 + (reverse[current_bme]<<3);
    cmd[1]=(0x02 + GOURP_A)<<1;
    bme_write_array(2,&cmd[0],COM_CONTINUE);// send the command reading configuration register group
    temp_chck[current_bme] = bme_read_array(6, &temp_data_array[0]); // read configuration register group and return CRC
    if(!temp_chck[current_bme]) 
    {
      for(int j=0;j<3;j++)
      {
        temp_data=((temp_data_array[2*j+1]<<8) | temp_data_array[2*j]);
        if(temp_data != SELF_CHECK_VAL_1) self_test_data = self_test_data| (1<<current_bme);
      }
    }
  }
  // return temp_data;
  return self_test_data;
}

/***********************************************//**
 \brief set bme flags that are related to the measurments.
*************************************************/
void bmes::reset_flags(){
  for(int current_bme = 0; current_bme<BME_NUM; current_bme++)
  {
    // clear the flags related to the measurements
    myBmeFlag[current_bme] = 0; 
    myBmeFlagOverride[current_bme] = 0;
  }
}

/***********************************************//**
 \brief set a bme override flags 
*************************************************/
void bmes::set_flag_over(uint16_t priority2Flag){
  for(int current_bme = 0; current_bme<BME_NUM; current_bme++)
  {
    myBmeFlagOverride[current_bme] |= (myBmeFlag[current_bme] & priority2Flag);
  }
  
}

/***********************************************//**
 \brief set bme flags that are related to the measurments.
*************************************************/
void bmes::set_meas_flags()
{
  for(int current_bme = 0; current_bme<BME_NUM; current_bme++)
  {
    // set miss communication flag
    if(myCrcCheck[current_bme]) myBmeFlag[current_bme] |= (1<<3);

    // set accuracy check
    if(abs(myVol_ref2[current_bme]-3.0) > myAccuracyLimit){
      myBmeFlag[current_bme] |= (1<<4);
    } 

    // set high myVoltage alarm check 
    if(cal_max_vol(current_bme)>myVolHighAlarm){
      myBmeFlag[current_bme] |= (1<<5);
    }

    // low myVoltage checks
    float minVol=cal_min_vol(current_bme);
    // set dead battery alarm
    if (minVol < myDeadBatAlarm) myBmeFlag[current_bme] |= (1<<8);
    // set low myVoltage alarm
    else if(minVol < myVolLowAlarm) myBmeFlag[current_bme] |= (1<<7);
    // set low myVoltage warning 
    else if(minVol < myVolLowWarn) myBmeFlag[current_bme] |= (1<<6);

    // set bme mismatch alarm
    if(abs(myTotal_bme_vol[current_bme]-cal_bme_sum(current_bme)) > myVolMismatch){
      myBmeFlag[current_bme] |= (1<<9);
    } 

    // high myTemperature checks
    float maxTemp=cal_max_temp(current_bme);
    // set high tempearture alarm
    if(maxTemp>myTempVCAlarm || myHs_temp[current_bme] > myTempHSAlarm || myInternal_temp[current_bme] > myTempTiAlarm){
      myBmeFlag[current_bme] |= (1<<10);
    }
    else if(maxTemp>myTempVCWarn || myHs_temp[current_bme] > myTempHSWarn || myInternal_temp[current_bme] > myTempTiWarn){
      myBmeFlag[current_bme] |= (1<<11);
    }

    // set low myTemperature alarm
    if(cal_min_temp(current_bme) < myLowTempAlarm){
      myBmeFlag[current_bme] |= (1<<12);
    }

     // tempearture control for balancing 
    balTempCon[current_bme]=false;
    if(myHs_temp[current_bme] > myTempHSAlarm-5 || myInternal_temp[current_bme] > myTempTiAlarm-5 || myHs_temp[current_bme] < myLowTempAlarm){
      balTempCon[current_bme]=true;
    } 

  }


}

/***********************************************//**
 \brief set bme flags that are related to the self test.
*************************************************/
void bmes::set_st_flags()
{
  for(int current_bme = 0; current_bme<BME_NUM; current_bme++)
  {
    // set the myVoltage self test flag
    if(myStCell[current_bme]) myBmeFlag[current_bme] |= 1;

    // set the GPIO self test flag
    if(myStGpio[current_bme]) myBmeFlag[current_bme] |= (1<<1);

    // set the status self test flag
    if(myStStat[current_bme]) myBmeFlag[current_bme] |= (1<<2);
  }

}


/***********************************************//**
 \brief Measures, Reads and parses the BMEs status registers.
 
 The function is used to read the status of the LTC6804.
 This function will send a start conversion command, wait then
 send the requested read commands parse the data
 and store the cell myTemperature in total_vol_data and 
 itemp_data variables. 
 
@param[out] uint8_t temp_data[BME_NUM*TEMP_NUM*2];
@param[out] float myTemperature[BME_NUM][TEMP_NUM];
An array of the parsed cell myTemperature from highest to lowest. 
 The cell myTemperature will be stored in the temp_data[] array 
 and in a myVoltage matrix
  
 @return int16_t, PEC Status.
 | bit0     | bit1     | bit2     | .... |bit11     |
 | BME 1 CRC| BME 2 CRC| BME 3 CRC| .... |BME 12 CRC|
  0: No PEC error detected
  1: PEC error detected, retry read
 *************************************************/
uint16_t bmes::meas_stat()
{
  uint8_t cmd[2];
  uint16_t crc_array = 0;
  uint8_t tempo_data[6];
  bool crc_tempo;

  bme_adstat(); // send broadcast cell gpio conversion to all BMEs on this channel
  delayMicroseconds(BME_CON_DELAY_ST); // waits for conversion to finish
  wakeup_idle();//This will guarantee that the BMEs isoSPI port is awake. This command can be removed.
  for(int current_bme = 0; current_bme < BME_NUM; current_bme++)
  {
    cmd[0]=0x80 + (reverse[current_bme]<<3);  // set the BMEs address
    for(int current_reg=GOURP_A;current_reg <= GOURP_A;current_reg++)
    {
      if(((crc_array >> current_bme) & 0x01)==0)
      {
        cmd[1]=(0x08 + current_reg)<<1;  //set the command for the right register
        bme_write_array(2,&cmd[0],COM_CONTINUE);// send the command reading configuration register group
        crc_tempo = bme_read_array(6, &tempo_data[current_reg*6]); // read configuration register group and return 
        if(crc_tempo) crc_array=crc_array | (1<<current_bme);
      }
    }
    if(((crc_array >> current_bme) & 0x01)==0){
      for(int i=0;i<2;i++){
        total_vol_data[2*current_bme+i]=tempo_data[i];
        itmp_data[2*current_bme+i]= tempo_data[i+2];
      }
    }
  }
  data2stat(crc_array);
  return crc_array;
}

/***********************************************//**
 \brief Measures, Reads and parses the BMEs cell myTemperature registers.
 
 The function is used to read the cell myTemperature of the LTC6804.
 This function will send a start conversion command, wait then
 send the requested read commands parse the data
 and store the cell myTemperature in temp_data and ref_data variables. 
 
@param[out] uint8_t temp_data[BME_NUM*TEMP_NUM*2];
@param[out] float myTemperature[BME_NUM][TEMP_NUM];
An array of the parsed cell myTemperature from highest to lowest. The cell myTemperature will
  be stored in the temp_data[] array and in a myVoltage matrix
  
 @return int16_t, PEC Status.
 | bit0     | bit1     | bit2     | .... |bit11     |
 | BME 1 CRC| BME 2 CRC| BME 3 CRC| .... |BME 12 CRC|
  0: No PEC error detected
  1: PEC error detected, retry read
 *************************************************/
uint16_t bmes::meas_temp()
{
  uint8_t cmd[2];
  uint16_t crc_array = 0;
  uint8_t tempo_data[12];
  bool crc_tempo;

  bme_adax(); // send broadcast cell gpio conversion to all BMEs on this channel
  delayMicroseconds(BME_CON_DELAY_AUX); // waits for conversion to finish
  wakeup_idle();//This will guarantee that the BMEs isoSPI port is awake. This command can be removed.
  for(int current_bme = 0; current_bme < BME_NUM; current_bme++)
  {
    cmd[0]=0x80 + (reverse[current_bme]<<3);  // set the BMEs address
    for(int current_reg=GOURP_A;current_reg <= GOURP_B;current_reg++)
    {
      if(((crc_array >> current_bme) & 0x01)==0)
      {
        cmd[1]=(0x06 + current_reg)<<1;  //set the command for the right register
        bme_write_array(2,&cmd[0],COM_CONTINUE);// send the command reading configuration register group
        crc_tempo = bme_read_array(6, &tempo_data[current_reg*6]); // read configuration register group and return 
        if(crc_tempo) crc_array=crc_array | (1<<current_bme);
      }
    }

    if(((crc_array >> current_bme) & 0x01)==0){
        int j=0;
        for(int i=0;i<4;i++){ 
          if(i+1 == myHs_temp_NUM){
            myHs_data[2*current_bme] = tempo_data[2*i];
            myHs_data[2*current_bme+1] = tempo_data[2*i+1];
          } 
          else{
            temp_data[2*current_bme*TEMP_NUM+2*i] = tempo_data[2*i];
            temp_data[2*current_bme*TEMP_NUM+2*i+1] = tempo_data[2*i+1];
            j++;
          }
        }
        // for(int i=0;i<2;i++) myHs_data[2*current_bme+i]= tempo_data[i+6];
        for(int i=0;i<2;i++) ref_data[2*current_bme+i]= tempo_data[i+10];
    }
    // Serial.println(myHs_data[current_bme*2],HEX);
  }
  return crc_array;
}

/***********************************************//**
 \brief Measures, Reads and parses the BMEs cell myVoltage registers.
 
 The function is used to read the cell myVoltages of the LTC6804.
 This function will send a start conversion command, wait then
 send the requested read commands parse the data
 and store the cell myVoltages in vol_data variable. 
 
@param[out] uint8_t vol_data[BME_NUM*VC_NUM*2];
@param[out] float myVoltage[BME_NUM][VC_NUM];
An array of the parsed cell myVoltage from highest to lowest. The cell myVoltages will
  be stored in the vol_data[] array and in a myVoltage matrix
  
 @return int16_t, PEC Status.
 | bit0     | bit1     | bit2     | .... |bit11     |
 | BME 1 CRC| BME 2 CRC| BME 3 CRC| .... |BME 12 CRC|
  0: No PEC error detected
  1: PEC error detected, retry read
 *************************************************/
uint16_t bmes::meas_vol()
{
  uint8_t cmd[2];
  bool crc_temp = false;
  uint16_t crc_array = 0;
  uint8_t reg_num = (VC_NUM-1)/3;
  uint8_t tempo_data[24] = {0};
  
  bme_adcv(); // send broadcast cell myVoltage conversion to all BMEs on this channel
  delayMicroseconds(BME_CON_DELAY_CV); // waits for conversion to finish
  wakeup_idle();//This will guarantee that the BMEs isoSPI port is awake. This command can be removed.
  for(int current_bme = 0; current_bme < BME_NUM; current_bme++)
  {
    cmd[0]=0x80 + (reverse[current_bme]<<3);  // set the BME address 
    for(int current_reg=0; current_reg <= reg_num; current_reg++)
    {
      if(((crc_array >> current_bme) & 0x01)==0)
      {
        cmd[1]=(0x02 + current_reg)<<1;  //set the command for the right register
        bme_write_array(2,&cmd[0],COM_CONTINUE);// send the command reading configuration register group
        crc_temp = bme_read_array(6, &tempo_data[current_reg*6]); // read configuration register group and return CRC
        if(crc_temp) crc_array |= (1<<current_bme);
      }
    }
    if(((crc_array >> current_bme) & 0x01)==0){
      for(int i=0;i<VC_NUM;i++){
        vol_data[2*current_bme*VC_NUM+2*(VC_NUM-i)-1]=tempo_data[2*i+1];
        vol_data[2*current_bme*VC_NUM+2*(VC_NUM-i-1)]=tempo_data[2*i];
      }
    }
  }
  
  return crc_array;
}

/*!******************************************************************************************************
 \brief Convert byte myVoltage data to floating myVoltage data
 takes data from 2 bytes form to float form
 *********************************************************************************************************/
void bmes::data2vol(uint16_t crc_check)
{
  for(int current_bme=0;current_bme<BME_NUM;current_bme++){
    if(((crc_check >> current_bme) & 0x01)==0){
      for(int current_cell=0;current_cell<VC_NUM;current_cell++){
        myVoltage[current_bme][current_cell] = ((vol_data[2*current_bme*VC_NUM+2*current_cell+1]<<8) | vol_data[2*current_bme*VC_NUM+2*current_cell])*0.0001;
      }  
    }
  }
}

/*!******************************************************************************************************
 \brief Convert byte myTemperature and v_ref2 data to floating myTemperature and v_ref2 data
 takes data from 2 bytes form to float form
 *********************************************************************************************************/
void bmes::data2temp(uint16_t crc_check)
{
  float tempo=0;

  for(int current_bme=0;current_bme<BME_NUM;current_bme++){
    if(((crc_check >> current_bme) & 0x01)==0){
      myVol_ref2[current_bme] = ((ref_data[2*current_bme+1]<<8) | ref_data[2*current_bme])*0.0001;
      for(int current_temp=0;current_temp<TEMP_NUM;current_temp++){
        tempo= ((temp_data[2*current_bme*TEMP_NUM+2*current_temp+1]<<8) | temp_data[2*current_bme*TEMP_NUM+2*current_temp]);
        if(tempo==0) myTemperature[current_bme][current_temp]=-273;
        else{
          tempo=tempo*0.0001;
          tempo=tempo/(myVol_ref2[current_bme]-tempo); 
          tempo=T0inv + Binv*log(tempo);
          myTemperature[current_bme][current_temp]=1.0/tempo-273;
        }
      }
      tempo= ((myHs_data[2*current_bme+1]<<8) | myHs_data[2*current_bme]);
      if(tempo==0) myHs_temp[current_bme]=-273;
      else{
        tempo=tempo*0.0001;
        tempo=tempo/(myVol_ref2[current_bme]-tempo); 
        tempo=T0inv + Binv*log(tempo);
        myHs_temp[current_bme]=1.0/tempo-273;
      }
    } 
  }
}

/*!******************************************************************************************************
 \brief Convert byte total myVoltage and internal myTemperature data to floating total myVoltage and 
   internal myTemperature data
 takes data from 2 bytes form to float form
 *********************************************************************************************************/
void bmes::data2stat(uint16_t crc_check)
{
  for(int current_bme=0;current_bme<BME_NUM;current_bme++){
    if(((crc_check >> current_bme) & 0x01)==0){
      myTotal_bme_vol[current_bme] = ((total_vol_data[2*current_bme+1]<<8) | total_vol_data[2*current_bme])*0.002; 
      myInternal_temp[current_bme] = (((itmp_data[2*current_bme+1]<<8) | itmp_data[2*current_bme])*IntTempConst)-273;
    }
  }
}

 /*!******************************************************************************************************
 \brief takes int data and turns it into boolean array
 *********************************************************************************************************/
void bmes::int2bool(int i_max, int data_in, bool bool_out[])
 {
   for(int i=0;i<i_max;i++){
     if(((data_in>>i) & 0x01) == 1) bool_out[i] = true;
     else bool_out[i] = false;
   }
 }

/*****************************************************//**
 \brief Write the LTC6804 configuration register 
 
 This command will write the configuration registers of the stacks 
 connected in a stack stack. 

@param[in] uint8_t addr; The address of BME being written with 16 being Broadcast to all. 
 
@param[in] uint16_t dcc;  The discharging cells
 | bit0     | bit1     | bit2     | .... |bit11     |
 | cell 1   | cell 2   | cell 3   | .... |cell 12   |
  0: do not discharge the cell
  1: discharge the cell
 The function will calculate the needed PEC codes for the write data
 and then transmit data to the ICs on a stack.
********************************************************/
void bmes::bme_wrcfg(uint8_t addr)
{
  
  uint8_t cmd[2];
  uint8_t cfg[6]={0};
  uint8_t vouv[3]={0x13,0xE6,0xA7};
 
  wakeup_idle ();                               //This will guarantee that the LTC6804 isoSPI port is awake.This command can be removed.
  
  if(addr<=15 && addr>=0) cmd[0]=0x80 + (reverse[addr]<<3);  // set the LTC68042 address
  else cmd[0]=0x00;                          // Broadcast command to all addresses 
  cmd[1]=0x01;            // the command for WRCFG
  
  cfg[0]=0xff;
  if(myFanOn[addr]) cfg[0]=0x7f;  // set the fan on 
  for(int i=0;i<3;i++){
    cfg[i+1]=vouv[i];      // set the myVoltage under/over limits
  }
  cfg[4]=(uint8_t)myDcc[addr];        // set the balancing MOSFETs
  cfg[5]=(myDcc[addr]>>8) & 0x0F;

  bme_write_array(2,&cmd[0],COM_CONTINUE);// send the command writing configuration register group
  bme_write_array(6,&cfg[0],COM_END); // send register group
  
}

/*!******************************************************************************************************
 \brief Start an States Conversion
 
  Starts an ADC conversions of the LTC6804 states.
  The type of ADC conversion done is set using the associated global variables:
 |Variable|Function                                        | 
 |--------|------------------------------------------------|
 | MD     | Determines the filter corner of the ADC        |
 | CHST   | Determines which states channels are converted |
 
*********************************************************************************************************/
void bmes::bme_adstat()
{
  //3
  wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.
  //4
  bme_write_array(2,ADSTAT,COM_END);
}

/*!******************************************************************************************************
 \brief Start an GPIO Conversion
 
  Starts an ADC conversions of the LTC6804 GPIO inputs.
  The type of ADC conversion done is set using the associated global variables:
 |Variable|Function                                      | 
 |--------|----------------------------------------------|
 | MD     | Determines the filter corner of the ADC      |
 | CHG    | Determines which GPIO channels are converted |
 
*********************************************************************************************************/
void bmes::bme_adax()
{
  //3
  wakeup_idle (); //This will guarantee that the LTC6804 isoSPI port is awake. This command can be removed.
  //4
  bme_write_array(2,ADAX,COM_END);
}

/*!*********************************************************************************************
  \brief Starts cell myVoltage conversion
  
  Starts ADC conversions of the LTC6804 Cpin inputs.
  The type of ADC conversion done is set using the associated global variables:
 |Variable|Function                                      | 
 |--------|----------------------------------------------|
 | MD     | Determines the filter corner of the ADC      |
 | CH     | Determines which cell channels are converted |
 | DCP    | Determines if Discharge is Permitted          |
  
***********************************************************************************************/
void bmes::bme_adcv()
{ 
  //2
  wakeup_idle (); //This will guarantee that the BMEs isoSPI port is awake. This command can be removed.
  //3
  bme_write_array(2,ADCV,COM_END); //send broadcast adcv command to BMEs stack

}


/*!****************************************************
  \brief Wake isoSPI up from idle state
 Generic wakeup commannd to wake isoSPI up out of idle
 *****************************************************/
void bmes::wakeup_idle()
{
  output_low(myCS);
  delayMicroseconds(10); //Guarantees the isoSPI will be in ready mode
  output_high(myCS);
}

/*!******************************************************************************************************************
 \brief Writes an array of bytes out of the SPI port
 
 @param[in] uint8_t len length of the data array being written on the SPI port
 @param[in] uint8_t data[] the data array to be written on the SPI port
 @param[in] bool com continue communication or end communication 
 ******************************************************************************************************************/
void bmes::bme_write_array(uint8_t len, // Option: Number of bytes to be written on the SPI port
           uint8_t *tx_data, //Array of bytes to be written on the SPI port
           bool com // continue the communication if true and end if false
           )
{
  uint16_t dataPec;
  dataPec = (uint16_t)pec15_calc(len, &tx_data[0]);// calculating the PEC for each board

  output_low(myCS);
  for(uint8_t i = 0; i < len; i++)
  {
     SPI.transfer((char)tx_data[i]);
  }
  SPI.transfer((char)(dataPec >> 8));
  SPI.transfer((char)dataPec);
  if(com) output_high(myCS);
}

/*!******************************************************************************************************************
 \brief Read an array of bytes out of the SPI port

@param[out] uint8_t rx_data array that read data will be written too. 
@param[in] uint8_t rx_len number of bytes to be read from the SPI port.
 ******************************************************************************************************************/
bool bmes::bme_read_array(uint8_t len, //Option: number of bytes to be read from the SPI port
          uint8_t *rx_data//Input: array that will store the data read by the SPI port
          )
{
  uint16_t receivedPec;
  uint16_t dataPec;
  bool crcCheck = false; 
  uint8_t dataIn[len+2];

  output_low(myCS);
  for(uint8_t i = 0; i < len+2; i++)
  {
    dataIn[i] = SPI.transfer(0x00);
  }
  output_high(myCS);

  receivedPec = (dataIn[len]<<8) + dataIn[len+1];
  dataPec = pec15_calc(len, &dataIn[0]);
  if(receivedPec != dataPec)
  {
    crcCheck = true;
  }  
  else
  {
    for(uint8_t i = 0; i < len; i++)
    {
      rx_data[i] = dataIn[i];
    }
  }
  return crcCheck;
}


/*!**********************************************************
 \brief calaculates  and returns the CRC15  

@param[in]  uint8_t len: the length of the data array being passed to the function
               
@param[in]  uint8_t data[] : the array of data that the PEC will be generated from
  

@return  The calculated pec15 as an unsigned int16_t
***********************************************************/
uint16_t bmes::pec15_calc(uint8_t len, uint8_t *data)
{
  uint16_t remainder,addr;
  
  remainder = 16;//initialize the PEC
  for(uint8_t i = 0; i<len;i++) // loops for each byte in data array
  {
    addr = ((remainder>>7)^data[i])&0xff;//calculate PEC table address 
    remainder = (remainder<<8)^crc15Table[addr];
  }
  return(remainder*2);//The CRC15 has a 0 in the LSB so the remainder must be multiplied by 2
}


