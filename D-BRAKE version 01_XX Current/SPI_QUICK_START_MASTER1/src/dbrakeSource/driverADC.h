//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//                FILE: DRIVER_RTC.H
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
#ifndef _DRIVERADC_H
#define _DRIVERADC_H

  
//---------------------GLOBAL DEFINITIONS--------------------------
 
#define ADC_INPUTVOLTAGE_8		155*8
#define ADC_INPUTVOLTAGE_8PT5   (155*85)/10
#define ADC_INPUTVOLTAGE_10PT5   (155*105)/10
#define ADC_INPUTVOLTAGE_10PT2   (155*102)/10
#define ADC_INPUTVOLTAGENONE    500

#define SUPERCAP_17V	2615   //0XA37
#define SUPERCAP_17PT5	2690

//---------------------GLOBAL VARIABLES--------------------------
extern uint16_t maxCurrentRead; 
extern uint16_t maxFSRRead; 
extern uint16_t adcTimer;
extern uint8_t adcTimeout;
#define ADCTIME 3
//---------------------GLOBAL PROTOTYPES--------------------------
void ADCInit(void);
void configure_adc(uint8_t which);
void configure_adc_callbacks(void);
void ADCTask(void);
void ADCStart(void);
uint16_t ADCGetReading(uint8_t which); 
	#define ADC_INPUT_VOLTAGE 0
	#define ADC_INPUT_CURRENT 1
	#define ADC_INPUT_FSR     2
	#define ADC_INPUT_SUPERCAP 3
#endif //__ACCEL_H__



