//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//                FILE: appMotor.c
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// Processor: STM32F103R
// TOOLS: IAR Workbench 
// DATE:
// CONTENTS: This file contains  
//------------------------------------------------------------------------------
// HISTORY: This file  
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
#include <asf.h>
#include "dbrakeDefs.h"
#include "appMotor.h"
#include "driverButtons.h"
#include "driverADC.h"
#include "appAccel.h"
#include "appProtocol.h"
#include "config.h"
#include "appEncoder.h"
#include "appBluetooth.h"
 
#define TESTWITHOUTSETUP 0
#define MAX_BRAKESETUPEXTEND 9
#define STOP_BRAKESETUPEXTEND 1     //0x4E20  //20000 - CHANGE BACK TO 1 FOR NORMAL BUILDS
 
#define HOME_IN 0
#define HOME_OUT 1
uint8_t homeLimit = HOME_OUT;  
 
uint8_t brakeState;
uint8_t prevBrakeState;
uint8_t prevBrakeState2; 
uint8_t brakeChange;

#define BRAKESUPTIME 100
#define BRAKESUPTIME_SHORT 20
#define BRAKESUPTIME_MEDIUM 30 
#define BRAKESUPTIME_TIMEOUT 150
#define BRAKESUPTIME_SETUPPAUSE 20
#define BRAKESUPTIMESHORT 10
 
#if BRAKEBOARD

#define MOTOR_RUN_TIME 300 //beth 200
//---------------------GLOBAL VARIABLES-----------------------------------
uint16_t voltageBadTime; 
uint16_t fastVoltageBadTime; 
uint16_t ditherTimer; 

#define BREAKAWAY_HOLD_TIME (500/25)  //25 msec timer
uint16_t breakawayHoldTimer; 

#define NEEDNEWBASELINE_TIME 100   //10 SECONDS FOR 100MSEC BASE
uint8_t needNewBaseline=1;
uint16_t needNewBaselineTimer; 

uint16_t prevEncoderCount; 
uint8_t actionEncoderSample;

//---------------------LOCAL VARIABLES------------------------------------
uint8_t chargingSupercap = 0; //01_38_#2
uint16_t supercapValue = 0; //01_38_#2
	//-----------matchCurrent
	// this is the current value to look for in the encoder table to 
	// retract. 
uint16_t matchCurrent; 

uint16_t instantGain;
uint16_t holdG;
uint16_t newG; 
uint16_t diffG; 
uint16_t gPrime; 

uint16_t encoderCountBack;  
uint16_t encoderCountBackTotal; 
 
uint8_t action = NONE; 
uint8_t prevAction = NONE;
uint8_t motorOn = FALSE; 

//uint8_t brakeSetupLED;
uint16_t brakeSetupExtend;
uint16_t motorx;
uint16_t motory; 
uint16_t motorz; 
uint8_t motorAccBaseline; 
int16_t motorAccXBaseline; 
int16_t motorAccYBaseline; 
int16_t motorAccZBaseline; 

	//----------------setupExtendTriggered
	// used to see if the extend is triggered during setup
	// should not happen. 
uint8_t	setupExtendTriggered = 0;



int16_t	tempx;
int16_t tempbasex;
int16_t tempdiffx;
	#define MAX_THRESHOLD_NEEDED 3
uint8_t thresholdmet = 0;   

uint8_t hlimitCount;
uint8_t flimitCount;  

uint8_t brakeBiLED;
uint8_t brakeBlueLED;
uint8_t brakeRedLED; 



uint16_t motorRunTime;
uint16_t prevMotorRunTime; 

uint8_t loadState = 0;
uint16_t loadTime = 0; 
 
//---------------------LOCAL FUNCTION PROTOTYPES--------------------------  
void ConfigureMotorHLimitChannel(void);
void ConfigureMotorHLimitCallbacks(void);
void MotorHLimitCallback(void);
void ConfigureMotorFLimitChannel(void);
void ConfigureMotorFLimitCallbacks(void);
void MotorFLimitCallback(void);
uint8_t BrakeActuatorControl(uint8_t which);
	#define BRAKE_HOME 0
	#define BRAKE_AWAY 1
	#define BRAKE_RUN  2
	#define BRAKE_MIDDLESTOP 3
	
	#define BRAKE_ERROR 0
	#define BRAKE_GOOD 1
void BrakeLEDControl(void);	
void BrakeSupervisorytask(void);
uint16_t CurrentMotorCalculated(void);
uint16_t FsrMotorCalculated(void);
int16_t MotorGetAcc(uint8_t activeBraking);
uint16_t CurrentMotorCalculatedBreakaway(void);
uint16_t FsrMotorCalculatedBreakaway(void);
uint16_t CurrentMotorCalculatedManual(void);
uint16_t FsrMotorCalculatedManual(void);
void BrakeEnterIdleSleepMode(void);
uint16_t CurrentMotorCalculatedForG(uint16_t gin);
void RetractByEncoderCount(void);
void MotorExtendMore(void);
void MotorNeedNewBaseline(void);
uint16_t CurrentMotorCalculatedStartup(uint8_t forceMax);
uint16_t FsrMotorCalculatedStartup(uint8_t forceMax);
	
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// ---------------------------GLOBAL FUNCTIONS ----------------------------------
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
uint16_t testValue = 0; 

void BrakeHoldOff(uint16_t howLong)
{
	brakeHoldOffTime = howLong; 
	while (brakeHoldOffTime >0)
	{
		
	}	
}


uint16_t LoadCell(uint8_t whichState)
{
	uint16_t value,itemp; 
	value = 0; 
	
	itemp = ADCGetReading(ADC_INPUT_FSR);	
	if ((loadState != whichState)||(itemp<0x60))
	{
		//----------------------
		// start the time to average over. 
		switch (whichState)
		{
			case BRAKESTATE_WAITONSETUP:
			{
				loadTime = 100;
				break;
			}	
			case BRAKESTATE_HOLDOFF_ACTIVEFROMSETUP:
			{
				loadTime = 100;
				break;
			}	
			case BRAKESTATE_HOLDOFF_ACTIVE:
			{
				loadTime = 100;
				break;
			}				
			case BRAKESTATE_ACTIVE:
			{
				loadTime = 500;  //01_28  was 100 
				break;
			}							
		}
		loadState = whichState; 
	}
	if (loadTime == 0)
	{
		value = itemp;
	}
	
//V01.30	if (value >4000)
//V01.30	{
//V01.30		value = 0; 
//V01.30	}
// testing only 	value = testValue; 
	return value;
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:
//------------------------------------------------------------------------------
// This function
//
//==============================================================================
void BrakeInit(void)
{
	brakeBiLED = BRAKEBILED_OFF;
	brakeBlueLED = BRAKEBLUELED_OFF;
	brakeRedLED = BRAKEREDLED_OFF; 
	brakeStatus.BrakeState |= BRAKESTATE_NOTSETUP;	
	brakeState = BRAKESTATE_POWERINGUP; 
	table0.Item.MaxForce = 5; 
	BrakeBoardStateMachineTask();	
	gPrime = 0; 
	//------------------------
//V01_11	poweredUp = 1;
//V01_11	brakeState = BRAKESTATE_RESET;
	poweredUp = 0;
	brakeState = BRAKESTATE_POWERINGUP;	
	
	BrakeBoardStateMachineTask();	
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:
//------------------------------------------------------------------------------
// This function
//
//==============================================================================
void BrakeEnterIdleSleepMode(void)
{
	brakeBiLED = BRAKEBILED_OFF;
	brakeBlueLED = BRAKEBLUELED_OFF;
	brakeRedLED = BRAKEREDLED_OFF;
	brakeState = BRAKESTATE_IDLESLEEP;
 
	
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:
//------------------------------------------------------------------------------
// This function
//
//==============================================================================
uint8_t BrakeActuatorControl(uint8_t which)
{
	uint8_t status; 
	status = BRAKE_GOOD; 
	switch (which)
	{
		case BRAKE_HOME:
		{
			if ((hlimitState != 0)&&(homeLimit == HOME_OUT)) //V01_26
			{	 
				if (encoderCount == 0)
				{
					brakeStatus.ActuatorStatus |= ACTUATORSTATUS_ENCODERFAIL;
					status = BRAKE_ERROR;
				}
				else
				{
					brakeStatus.ActuatorStatus |= ACTUATORSTATUS_HOMEOFFFAIL;
					brakeStatus.ActuatorStatus &= ~ACTUATORSTATUS_ENCODERFAIL;
					status = BRAKE_ERROR; 
				}
			}
			else
			{
				brakeStatus.ActuatorStatus &= ~ACTUATORSTATUS_HOMEOFFFAIL;
			}
			break;
		}
		case BRAKE_AWAY:
		{
			if (flimitState != 0)
			{
				if (encoderCount == 0)
				{
					brakeStatus.ActuatorStatus |= ACTUATORSTATUS_ENCODERFAIL;
					status = BRAKE_ERROR;
				}
				else
				{
					brakeStatus.ActuatorStatus |= ACTUATORSTATUS_EXTENDOFFFAIL;
					brakeStatus.ActuatorStatus &= ~ACTUATORSTATUS_ENCODERFAIL;
					status = BRAKE_ERROR;
				}
			}
			else
			{
				brakeStatus.ActuatorStatus &= ~ACTUATORSTATUS_EXTENDOFFFAIL;
			}
			break;
		}		
		case BRAKE_RUN:
		{
			if (encoderCount == 0)
			{
				brakeStatus.ActuatorStatus |= ACTUATORSTATUS_ENCODERFAIL;
				status = BRAKE_ERROR;
			}
			else
			{
				brakeStatus.ActuatorStatus &= ~ACTUATORSTATUS_ENCODERFAIL;
			}
			break;
		}		
		case BRAKE_MIDDLESTOP:
		{
			if (flimitState == 0)
			{
				brakeStatus.ActuatorStatus |= ACTUATORSTATUS_EXTENDONFAIL;
				status = BRAKE_ERROR;
			}
			else
			{
				brakeStatus.ActuatorStatus &= ~ACTUATORSTATUS_EXTENDONFAIL;
			}
			if (hlimitState == 0)
			{
				brakeStatus.ActuatorStatus |= ACTUATORSTATUS_HOMEONFAIL;
				status = BRAKE_ERROR;
			}
			else
			{
				brakeStatus.ActuatorStatus &= ~ACTUATORSTATUS_HOMEONFAIL;
			}			
			break;
		}		
	}
	if ((flimitState == 0)&&(hlimitState ==0))
	{
		brakeStatus.ActuatorStatus |= ACTUATORSTATUS_BOTHLIMITSACTIVE;	
		status = BRAKE_ERROR;
	}
	else
	{
		brakeStatus.ActuatorStatus &= ~ACTUATORSTATUS_BOTHLIMITSACTIVE;
	}
	return status; 
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:
//------------------------------------------------------------------------------
// This function
//
//==============================================================================
void BrakeLEDControl(void)
{
	//--------------------------------
	// what should the bi color LED be doing?
	// 1. flash red for the following errors: 
	//    a. brakeStatus.BrakeState |= BRAKESTATE_INPUTVOLTAGEBAD;	
	switch (brakeState)
	{
		case BRAKESTATE_POWERINGUP:
		case BRAKESTATE_POWEREDUP0:		
		case BRAKESTATE_POWEREDUP:
		case BRAKESTATE_IDLESLEEP:
		{
			//leds are off 
			brakeBiLED = BRAKEBILED_OFF;
			brakeBlueLED = BRAKEBLUELED_OFF; 
			brakeRedLED = BRAKEREDLED_OFF; 
			break; 
		}
		case BRAKESTATE_RESET:
		case BRAKESTATE_PRESETUP:
		case BRAKESTATE_PRESETUP0:
		{
			brakeBiLED = BRAKEBILED_OFF;
			brakeBlueLED = BRAKEBLUELED_ALTGREEN;
			brakeRedLED = BRAKEREDLED_OFF; 
			break;
		}		
		case BRAKESTATE_WAITONSETUPLOADCELL:
		{
			//v01_57
			if ((brakeStatus.BrakeState & BRAKESTATE_NOINPUTVOLTAGE)!= 0)
			{
				brakeBiLED = BRAKEBILED_YELLOWFLASH;
				brakeBlueLED = BRAKEBLUELED_OFF;
				brakeRedLED = BRAKEREDLED_OFF;
			}
			else
			{			
				brakeBiLED = BRAKEBILED_REDSOLID;
				brakeBlueLED = BRAKEBLUELED_SOLID;
				brakeRedLED = BRAKEREDLED_OFF;
			}
			break;
		}
		case BRAKESTATE_ERROR_RETRACT:
		case BRAKESTATE_ERROR_RETRACT_LOWVOLTAGE:
		case BRAKESTATE_ERROR_FINAL:
		case BRAKESTATE_ERRORLOAD:
		case BRAKESTATE_ERRORLOADWAIT:
		case BRAKESTATE_ERROR_VOLTAGE_ACTIVE:					
		{
			brakeBiLED = BRAKEBILED_YELLOWFLASH;
			brakeBlueLED = BRAKEBLUELED_OFF;
			brakeRedLED = BRAKEREDLED_OFF; 			
			break;
		}				
		case BRAKESTATE_ERROR:
		{
			brakeBiLED = BRAKEBILED_REDFLASH;
			brakeBlueLED = BRAKEBLUELED_OFF;
			brakeRedLED = BRAKEREDLED_OFF; 			
			break;
		}		
		case BRAKESTATE_SETUP:
		case BRAKESTATE_SETUPACTIVE:
		{
			//v01_57
			if ((brakeStatus.BrakeState & BRAKESTATE_NOINPUTVOLTAGE)!= 0)
			{
				brakeBiLED = BRAKEBILED_YELLOWFLASH;
				brakeBlueLED = BRAKEBLUELED_OFF;
				brakeRedLED = BRAKEREDLED_OFF;
			}
			else
			{
				brakeBiLED = BRAKEBILED_OFF;
				brakeBlueLED = BRAKEBLUELED_ALTGREEN;
				brakeRedLED = BRAKEREDLED_OFF; 			
			}
			break;
		}		
		case BRAKESTATE_ACTIVE:
		case BRAKESTATE_ACTIVELOAD:
		case BRAKESTATE_HOLDOFF_ACTIVE:
		{
			brakeBiLED = BRAKEBILED_GREENSOLID;
			if ((brakeStatus.BrakeState & BRAKESTATE_NOINPUTVOLTAGE)!= 0)
			{
				brakeBiLED = BRAKEBILED_YELLOWFLASH;
			}
			else
			{
				if ((brakeStatus.BrakeState & BRAKESTATE_INPUTVOLTAGEBAD)!= 0)
				{
					if ((brakeStatus.BrakeState & BRAKESTATE_COMMERROR)!= 0)
					{
						brakeBiLED = BRAKEBILED_YELLOWFLICKER;
					}
					else	
					{			
						brakeBiLED = BRAKEBILED_YELLOWSOLID;
					}
				}
				else
				{
					if ((brakeStatus.BrakeState & BRAKESTATE_COMMERROR)!= 0)
					{
//V01_11				brakeBiLED = BRAKEBILED_GREENFLICKER;
						brakeBiLED = BRAKEBILED_GREENSOLID;						
					}
				}
			}
			brakeBlueLED = BRAKEBLUELED_OFF;	
			brakeRedLED = BRAKEREDLED_OFF; 					
			break;
		}
		case BRAKESTATE_ACTIVE_EXTEND:
		case BRAKESTATE_ACTIVE_RETRACT:
		case BRAKESTATE_ACTIVE_HOLD:
		case BRAKESTATE_END_RETRACT:
		{
			brakeBlueLED = BRAKEBLUELED_SOLID;
			brakeRedLED = BRAKEREDLED_OFF; 			
			break;
		}		
		case BRAKESTATE_ACTIVE_EXTEND_BREAKAWAY:
		case BRAKESTATE_ACTIVE_HOLD_BREAKAWAY:
		{
			brakeRedLED = BRAKEREDLED_SOLID;
			brakeBlueLED = BRAKEBLUELED_SOLID;
			break;
		} 
	}
}


//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:
//------------------------------------------------------------------------------
// This function
//
//==============================================================================
void BrakeSupervisorytask(void)
{
	uint16_t currentvalue;
	
	//--------------------------------
	// check voltages
	currentvalue = ADCGetReading(ADC_INPUT_VOLTAGE);
	supercapValue = ADCGetReading(ADC_INPUT_SUPERCAP);		
	if (currentvalue< ADC_INPUTVOLTAGE_10PT2)  //ADC_INPUTVOLTAGE_8)
	{
		if ((fastVoltageBadTime >= VOLTAGE_BAD_TIME)&&
			    ((brakeState == BRAKESTATE_PRESETUP0)||(brakeState == BRAKESTATE_PRESETUP)))
		{
			if ((brakeStatus.BrakeState & BRAKESTATE_INPUTVOLTAGEBAD)== 0)
			{
				brakeStatus.BrakeState |= BRAKESTATE_INPUTVOLTAGEBAD;
			}
		}		
		if (voltageBadTime >= VOLTAGE_BAD_TIME)
		{
			if ((brakeStatus.BrakeState & BRAKESTATE_INPUTVOLTAGEBAD)== 0)
			{
				brakeStatus.BrakeState |= BRAKESTATE_INPUTVOLTAGEBAD;
			}
		}
		if (currentvalue < ADC_INPUTVOLTAGENONE)
		{
			brakeStatus.BrakeState |= BRAKESTATE_NOINPUTVOLTAGE;
		}
	}
	else
	{
		brakeStatus.BrakeState &= ~BRAKESTATE_NOINPUTVOLTAGE;
		if ((brakeStatus.BrakeState & BRAKESTATE_INPUTVOLTAGEBAD)!= 0)
		{
			if ((currentvalue> ADC_INPUTVOLTAGE_8PT5)&&(supercapValue > 0x800))
			{
				brakeStatus.BrakeState &= ~BRAKESTATE_INPUTVOLTAGEBAD;	
			}
		}
		else
		{
			voltageBadTime = 0; 
			fastVoltageBadTime = 0; 
		}
	}
	//-----------------------------------------
	// if voltage is over 10.5 volts and board is turned on - 
	// enable the super cap. 
//	if ((brakeState != BRAKESTATE_IDLESLEEP)&&(brakeState != BRAKESTATE_POWERINGUP))
//	{
		
		if (currentvalue> ADC_INPUTVOLTAGE_10PT5)
		{	
			//----------------boc //01_38_#2

			if (chargingSupercap == 0)
			{
				if (supercapValue < 0xada)
				{
					port_pin_set_output_level(SUPERCAPEN, true);  
					chargingSupercap = 1;   
				}
			}
			else
			{
				if (supercapValue > 0xb47)
				{
					port_pin_set_output_level(SUPERCAPEN, false);
					chargingSupercap = 0;   
				}		
			}
		}
		else
		{
			if (currentvalue< ADC_INPUTVOLTAGE_10PT2)
			{
				if (chargingSupercap != 0)
				{				
					port_pin_set_output_level(SUPERCAPEN, false);
					chargingSupercap = 0;
				}
				if (supercapValue < 0x0400)
				{
					brakeStatus.BrakeState |= BRAKESTATE_NOINPUTVOLTAGE;					
				}				
			}
		}
		if (chargingSupercap == 0)
		{
			port_pin_set_output_level(SUPERCAPEN, false);  		
		}
		else
		{
			port_pin_set_output_level(SUPERCAPEN, true);  		
		}		
}

uint16_t newCurrentThreshold; 
uint16_t currentRead;
uint16_t newFSRThreshold;
uint16_t fsrRead;
uint16_t setupCurrent;
uint16_t setupFSR; 
uint8_t brakeInitiationCount=0; 
uint8_t brakeCycleCount=0; 
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:
//------------------------------------------------------------------------------
// This function
//
//==============================================================================
void BrakeBoardStateMachineTask(void)
{
	uint8_t i,done,button,itemp,forceExtend;
	uint16_t itemp2,itemp3;

//----- boc 01_38_#3	
	prevBrakeState2 = prevBrakeState;
	prevBrakeState = brakeState; 
	if ((brakeStatus.BrakeState & BRAKESTATE_INPUTVOLTAGEBAD)!= 0)
	{
//		brakeState = BRAKESTATE_ERROR_RETRACT;
	}	
	//-----------------
	// if action is EXTENDING OR RETRACTING
	// check that encoder count is changing. 
	// if not, there is an error. 
	// runs every 25msec for for 1 sec 
	// 1000/25 = 40
	//----------------------------
	if (((action == EXTENDING)||(action == RETRACTING_BY_ENCODER) ||
	     (action == RETRACTING)||(action == EXTENDING_BY_ENCODER))   && 
		 (brakeState !=BRAKESTATE_ERROR_FINAL ))
	{
		if (encoderCount == 0)
	    {
	//		brakeState = BRAKESTATE_ERROR_FINAL;
		}
		if (prevEncoderCount != encoderCount)
		{
			actionEncoderSample = 0; 
		}
		else
		{	
			actionEncoderSample++;	
			if (actionEncoderSample >120) //20)  //0)
			{
				brakeState = BRAKESTATE_ERROR_RETRACT;
 				brakeStatus.BrakeState |= BRAKESTATE_NOTSETUP;
 				brakeStatus.BrakeState |= BRAKESTATE_ERRORLOADSET;					
			}
		}
		prevEncoderCount = encoderCount;  
	}
//---- eoc 01_38_#3 
	
//	table0.Item.MaxForce = 5;  //hard coded for testing
	done = 0;
	button = ButtonChanged();
	BrakeSupervisorytask();
	if (breakawayRing_pressed == 1)
	{
		brakeStatus.BrakeState |= BRAKESTATE_BREAKAWAYREADY;
	}
	else
	{
		brakeStatus.BrakeState &= ~BRAKESTATE_BREAKAWAYREADY;
	}	

	
	switch(brakeState)
	{
//----------------------------
// POWERING UP STATES - that will retract arm from power-up. 
// this happens when the board has NO POWER and the processor was not 
// powered up and power is added. This is REALLY A FIRST TIME board has any 
// power on it and the supercaps are just going to get charged. 
//-------------------------------		
		case BRAKESTATE_POWERINGUP:
		{
			MotorOff(1);
			brakeState = BRAKESTATE_POWEREDUP;
			encoderCountBack =0;
			if ((brakeStatus.BrakeState & (BRAKESTATE_INPUTVOLTAGEBAD|BRAKESTATE_NOINPUTVOLTAGE))!= 0)
			{
				brakeState = BRAKESTATE_ERROR_RETRACT;
			}	
			else
			{		
				if (hlimitState != 0)
				{
					brakeState = BRAKESTATE_POWEREDUP0;
					MotorCW();
					brakeSupTime = 20;  //3 seconds to EXTEND
					//----- boc 1_23 ---- control by encode counts 
					encoderCountBack = ENCODER_EXTEND_STARTCOUNT; 
					action = EXTENDING_BY_ENCODER; 
					//----- eoc 1_23 
					brakeChange &= ~BRAKECHANGE_SUPTIME;
				}
			}
			brakeStatus.BrakeState &= (BRAKESTATE_INPUTVOLTAGEBAD|BRAKESTATE_NOINPUTVOLTAGE); 
			break;
		}
//v00_20 added the POWEREDUP0 state to handle a short extension first		
		case BRAKESTATE_POWEREDUP0:
		{
			if (((brakeChange & BRAKECHANGE_SUPTIME)!= 0)||(encoderCountBack ==0))
			{
				MotorOff(1);
				brakeState = BRAKESTATE_POWEREDUP;
				MotorCCW();
				brakeSupTime = 150;  //15 seconds to retract
				brakeChange &= ~BRAKECHANGE_SUPTIME;
			}
			brakeStatus.BrakeState &= (BRAKESTATE_INPUTVOLTAGEBAD|BRAKESTATE_NOINPUTVOLTAGE); 
			break;
		}		
		case BRAKESTATE_POWEREDUP:
		{
			if (((brakeChange & BRAKECHANGE_SUPTIME)!= 0)||
			((hlimitState ==0)||(homeLimit == HOME_IN))) //V01_26
			{
				MotorOff(1);
				brakeSupTime = 0;
				brakeChange &= ~BRAKECHANGE_SUPTIME;
				BrakeActuatorControl(BRAKE_HOME);
				brakeState = BRAKESTATE_IDLESLEEP;
				BrakeEnterIdleSleepMode();
			}
			break;
		}
		case BRAKESTATE_IDLESLEEP:
		{
//			port_pin_set_output_level(SUPERCAPEN,false); 		
			break;
		}				
//------------------------------------
// POWER ON KEY HAS BEEN PRESSED. 
//------------------------------------		
		case BRAKESTATE_RESET:
		{
//v01_11			poweredUp = 1;
			motorAccBaseline = 0; 
			thresholdmet = 0; 
			breakawayHoldTimer = 0; 

			brakeStatus.ActuatorStatus = 0; 
			brakeState = BRAKESTATE_PRESETUP;
			brakeStatus.BrakeState = 0; 
#if TESTWITHOUTSETUP
#else		
			brakeStatus.BrakeState |= BRAKESTATE_NOTSETUP;	
#endif			
//			system_interrupt_disable_global();
			wdt_reset_count();
			//-----ADC--------------
			ADCInit();
			wdt_reset_count();
			//---------------LORA/FSK radio
			//---------------------
			// SET FCC stuff up
			setTXContinuous = 0;
			setCW = 0;
			setRXContinuous = 0;
			for (i=0;i<10;i++)
			{
//				FCCSample();
			}
			if ((constantTX_pressed!=0)&&(constantRX_pressed==0)&&(constantCW_pressed==0))
			{
				setTXContinuous = 1;
			}
			if ((constantTX_pressed==0)&&(constantRX_pressed!=0)&&(constantCW_pressed==0))
			{
				setRXContinuous = 1;
			}
			if ((constantTX_pressed==0)&&(constantRX_pressed==0)&&(constantCW_pressed!=0))
			{
				setCW = 1;
			}			
			whichRadio = WHICHRADIO_LORA; 
			switchToFSK = FALSE; 
			CommInit();
			wdt_reset_count();
			//-------RF433--- pressure sensor radio
//disabled for now			RF433Init();			
			
			system_interrupt_enable_global();
			ADCStart();
			//------------------
			// make surE the break away has been sampled.
			// ButtonSample - will give you the breakaway inputs, and
			//    the limits.
			//------------------
			for (i=0;i<8;i++)
			{
				ButtonSample();
			}
			
//--- V01_20 added short extend for new setup		
			MotorOff(1);
			if ((brakeStatus.BrakeState & (BRAKESTATE_INPUTVOLTAGEBAD|BRAKESTATE_NOINPUTVOLTAGE))!= 0)
			{
				brakeState = BRAKESTATE_ERROR_RETRACT;
			}	
			else
			{
				brakeState = BRAKESTATE_PRESETUP0;
				MotorCW();
				brakeSupTime = 20;  //3 seconds to EXTEND
				//----- boc 1_23 ---- control by encode counts
				encoderCountBack = ENCODER_EXTEND_STARTCOUNT;
				action = EXTENDING_BY_ENCODER;
				//----- eoc 1_23				
				brakeChange &= ~BRAKECHANGE_SUPTIME;
			}
			break;
		}
//----- V01_20 below state added 		
		case BRAKESTATE_PRESETUP0:
		{

			if (((brakeChange & BRAKECHANGE_SUPTIME)!= 0)||(encoderCountBack==0))
			{
				MotorOff(1);		
				if ((brakeStatus.BrakeState & (BRAKESTATE_INPUTVOLTAGEBAD|BRAKESTATE_NOINPUTVOLTAGE))!= 0)
				{
					brakeState = BRAKESTATE_ERROR_RETRACT;
				}
				else
				{				
					brakeState = BRAKESTATE_PRESETUP;
					MotorCCW();
					brakeSupTime = 150;  //5 seconds to retract
					brakeChange &= ~BRAKECHANGE_SUPTIME;
				}
			}
			break;			
		}
		case BRAKESTATE_PRESETUP:
		{

			if (((brakeChange & BRAKECHANGE_SUPTIME)!= 0)||
			((hlimitState ==0)||(homeLimit == HOME_IN)))  //V01_26
			{
				MotorOff(1);
				brakeSupTime = 0;
				brakeChange &= ~BRAKECHANGE_SUPTIME;
				if ((brakeStatus.BrakeState & (BRAKESTATE_INPUTVOLTAGEBAD|BRAKESTATE_NOINPUTVOLTAGE))!= 0)
				{
					brakeState = BRAKESTATE_ERROR_RETRACT;
				}
				else
				{				
					if (BrakeActuatorControl(BRAKE_HOME)==BRAKE_ERROR)
					{
						brakeState = BRAKESTATE_ERROR_RETRACT;
					}
					else
					{
						if ((brakeStatus.BrakeState & BRAKESTATE_NOTSETUP)== 0)
						{
							MotorNeedNewBaseline();
							brakeSupTime = BRAKESUPTIME;	
							brakeState = BRAKESTATE_HOLDOFF_ACTIVE;
							brakeStatus.BrakeState &= ~BRAKESTATE_NOTSETUP;
						}
						else
						{
							brakeState = BRAKESTATE_WAITONSETUP;
						}	 		
					}
				}
			}
			break;
		}
		case BRAKESTATE_WAITONSETUP:
		case BRAKESTATE_WAITONSETUPLOADCELL:		
		{
			//----- boc V01_23 check for force on pedal before setup
//			itemp3 = ADCGetReading(ADC_INPUT_FSR);
			itemp3 = LoadCell(BRAKESTATE_WAITONSETUP);
			if (itemp3>0x60)								
			{
				brakeState = BRAKESTATE_WAITONSETUPLOADCELL;
													
			}	
			else
			//---- eoc V01_23 
			{
				//v01_56 add to check if input voltage is present. 
				if ((brakeStatus.BrakeState & BRAKESTATE_NOINPUTVOLTAGE)== 0)
				{
					brakeState = BRAKESTATE_WAITONSETUP;
					if (setup_pressed != 0)
					{
						brakeState = BRAKESTATE_SETUPACTIVE;
						brakeSetupExtend = 0;
						maxCurrentRead = 0;
						maxFSRRead = 0; //V01_41 
						setupExtendTriggered = 0;
						if (flimitState != 0)
						{
							MotorCW();
							brakeSupTime = BRAKESUPTIME;   
							brakeChange &= ~BRAKECHANGE_SUPTIME;
						}
					}
				}		
			} 
			break;
		}
//--------------------------------
// ACTIVE SETUP STATES
// - SETUP key has been pressed and now doing the setup. The first EXTEND CW has 
// been started from the WAITONSETUP state. 		
		case BRAKESTATE_SETUPACTIVE_PAUSE_EXTEND:
		{
			if ((brakeStatus.BrakeState & (BRAKESTATE_INPUTVOLTAGEBAD|BRAKESTATE_NOINPUTVOLTAGE))!= 0)
			{
				brakeState = BRAKESTATE_ERROR_RETRACT;
			}	
			else
			{	
				if ((brakeChange & BRAKECHANGE_SUPTIME)!= 0) 
				{
				 
					brakeSupTime = 0;
					brakeChange &= ~BRAKECHANGE_SUPTIME;		
					brakeSetupExtend++;
					MotorCCW();
					brakeSupTime = BRAKESUPTIME;  	
					brakeState = BRAKESTATE_SETUPACTIVE;
				}
			}
			break;
		}
		case BRAKESTATE_SETUPACTIVE_PAUSE_RETRACT:
		{
			if ((brakeStatus.BrakeState & (BRAKESTATE_INPUTVOLTAGEBAD|BRAKESTATE_NOINPUTVOLTAGE))!= 0)
			{
				brakeState = BRAKESTATE_ERROR_RETRACT;
			}			
			else
			{
				if ((brakeChange & BRAKECHANGE_SUPTIME)!= 0)
				{
				
					brakeSupTime = 0;
					brakeChange &= ~BRAKECHANGE_SUPTIME;
					brakeSetupExtend++;
					MotorCW();
					brakeSupTime = BRAKESUPTIME;  
					brakeState = BRAKESTATE_SETUPACTIVE;
				}
			}
			break;
		}		
		case BRAKESTATE_SETUPACTIVE:
		{
			done = 0; 
			
			//---------------???????? on below 
//			if (((button & KEY_SETUP)!= 0)&&(setup_pressed != 0))
//			{
//				done = 1; 
//				brakeState = BRAKESTATE_POWERINGUP;
//			}
			if (brakeSetupExtend==(MAX_BRAKESETUPEXTEND-1))
			{
				//building table 
				if ((brakeChange & BRAKECHANGE_TABLESAMPLE)!= 0)
				{
					brakeChange &= ~BRAKECHANGE_TABLESAMPLE; 
					if (buildTableOffset >= MAX_BUILDTABLE)
					{
						buildTableOffset = 0;
					}
					buildTable.Current[buildTableOffset] = ADCGetReading(ADC_INPUT_CURRENT); 
					buildTable.EncoderCount[buildTableOffset] = encoderCount;
					buildTableOffset++;
				}
			}

			if (done == 0)
			{
			//---------------------------------
			// brakeSetupExtend
			// * counts 0,2,4,6,8 - extend
			// * counts 1,3,5,7,9 - retract
			if ((brakeSetupExtend & 0x01)==0)
			{
				forceExtend = 0x00; 
			}
			else
			{
				forceExtend = 0x01;
			}
			switch (forceExtend)   //brakeSetupExtend)
			{
				case 0:
				case 2:
				case 4:
				case 6:
				case 8:
				{
					if (flimitState == 0)
					{
						if (BrakeActuatorControl(BRAKE_AWAY)==BRAKE_ERROR)
						{
							brakeState = BRAKESTATE_ERROR_RETRACT;
						}		
						else
						{	
							setupExtendTriggered = 1;		
							brakeState = BRAKESTATE_ERROR_RETRACT;				
						}
						brakeChange &= ~BRAKECHANGE_SUPTIME;
					}
					else
					{
						if ((BrakeActuatorControl(BRAKE_RUN)==BRAKE_ERROR)&&(motorRunTime ==0))
						{
							brakeState = BRAKESTATE_ERROR_RETRACT;			
						}	
						else
						{
							if ((brakeChange & BRAKECHANGE_SUPTIME)!= 0)
							{
								brakeChange &= ~BRAKECHANGE_SUPTIME;
								MotorOff(1);
								if (BrakeActuatorControl(BRAKE_RUN)==BRAKE_ERROR)
								{
									brakeState = BRAKESTATE_ERROR_RETRACT;
								}
								else
								{
									brakeState = BRAKESTATE_ERROR_RETRACT;
								}	
								brakeChange &= ~BRAKECHANGE_SUPTIME;
							}			
							if (motorRunTime == 0)
							{
							
#if FSR_USE
								setupFSR = FsrMotorCalculatedStartup(table0.Item.ForceMaxSet);
								if (((maxFSRRead > setupFSR)&&(brakeSetupExtend!= (MAX_BRAKESETUPEXTEND-1)))||
								((maxCurrentRead > CURRENT_THRESHOLD_TABLEBUILD)&&(brakeSetupExtend== (MAX_BRAKESETUPEXTEND-1))))
//								if (((maxFSRRead > setupFSR)&&(brakeSetupExtend!= (MAX_BRAKESETUPEXTEND-1)))||
//								((brakeSetupExtend== (MAX_BRAKESETUPEXTEND-1))))								
#else								
								setupCurrent = CurrentMotorCalculatedStartup(table0.Item.ForceMaxSet);
								if (((maxCurrentRead > setupCurrent)&&(brakeSetupExtend!= (MAX_BRAKESETUPEXTEND-1)))||								
//v1.05							if (((maxCurrentRead > CURRENT_THRESHOLD_SETUP)&&(brakeSetupExtend!= 8))||
								((maxCurrentRead > CURRENT_THRESHOLD_TABLEBUILD)&&(brakeSetupExtend== (MAX_BRAKESETUPEXTEND-1))))
#endif
								{
									MotorOff(1);
									if (BrakeActuatorControl(BRAKE_RUN)==BRAKE_ERROR)
									{
										brakeState = BRAKESTATE_ERROR_RETRACT;
									}
									else
									{								
										brakeState = BRAKESTATE_SETUPACTIVE_PAUSE_EXTEND; 										
										brakeSupTime = BRAKESUPTIME_SETUPPAUSE;  									
									}
									brakeChange &= ~BRAKECHANGE_SUPTIME;
								}
							}
						}
					}
					break;
				}
				case 1:
				case 3:
				case 5:
				case 7:
				case 9:
				{
					if ((hlimitState == 0)||(homeLimit== HOME_IN)) //V01_26
					{
						if (BrakeActuatorControl(BRAKE_HOME)==BRAKE_ERROR)
						{
							brakeState = BRAKESTATE_ERROR_RETRACT;
							brakeSupTime = 0;
						}
						else
						{		
							if (brakeSetupExtend == STOP_BRAKESETUPEXTEND)
//V1.05									if (brakeSetupExtend == 9)												
							{
										if (setupExtendTriggered != 0)
										{
											brakeState = BRAKESTATE_ERROR_RETRACT;
											MotorOff(1);										
										}
										else
										{		
											brakeState = BRAKESTATE_SETUPACTIVE_END; //BRAKESTATE_HOLDOFF_ACTIVEFROMSETUP;
											MotorOff(1);
											brakeSupTime =BRAKESUPTIME;   
											if ((brakeStatus.BrakeState & (BRAKESTATE_INPUTVOLTAGEBAD|BRAKESTATE_NOINPUTVOLTAGE))!= 0)
											{
												brakeState = BRAKESTATE_ERROR_RETRACT;
											}
											else
											{
												MotorCCW(); //V01_27
											}
											//------------v1.05 boc
//											brakeInitiationCount = 0;
//											brakeCycleCount = 0;
//											if (table0.Item.Hybrid == TRUE)
//											{
//												brakeInitiationCount = 10;
//											}
											//--------------v1.05 eoc													
										}				
							}	
							else
							{
								brakeState = BRAKESTATE_SETUPACTIVE_PAUSE_RETRACT; 								
 								brakeSupTime = BRAKESUPTIME_SETUPPAUSE;  
							}
						}
						brakeChange &= ~BRAKECHANGE_SUPTIME;
					}
					else
					{
						if ((BrakeActuatorControl(BRAKE_RUN)==BRAKE_ERROR)&&(motorRunTime ==0))
						{
							brakeState = BRAKESTATE_ERROR_RETRACT;
							brakeSupTime = 0;
							MotorOff(1);
						}
						else
						{						
							if ((brakeChange & BRAKECHANGE_SUPTIME)!= 0)
							{
								brakeChange &= ~BRAKECHANGE_SUPTIME;
								//						brakeState = BRAKESTATE_IDLE;
								//						brakeBiLED = BRAKEBILED_SOLIDGREEN;
								//----------------ERROR here????
								MotorOff(1);
								if (BrakeActuatorControl(BRAKE_RUN)==BRAKE_ERROR)
								{
									brakeState = BRAKESTATE_ERROR_RETRACT;
									brakeSupTime = 0;
								}
								else
								{
									if (brakeSetupExtend == STOP_BRAKESETUPEXTEND)
//V1.05									if (brakeSetupExtend == 9)									
									{
										if (setupExtendTriggered != 0)
										{
											brakeState = BRAKESTATE_ERROR_RETRACT;
											MotorOff(1);									
										}
										else
										{		
//											MotorNeedNewBaseline();					
//											brakeStatus.BrakeState &= ~BRAKESTATE_NOTSETUP;
											brakeState = BRAKESTATE_HOLDOFF_ACTIVEFROMSETUP;
											MotorOff(1);
											brakeSupTime = 30; // BRAKESUPTIME;	
											//------------v1.05 boc
//											brakeInitiationCount = 0;
//											brakeCycleCount = 0;
//											if (table0.Item.Hybrid == TRUE)
//											{
//												brakeInitiationCount = 10;
//											}
											//--------------v1.05 eoc													
										}
									}
									else
									{						
										brakeState = BRAKESTATE_SETUPACTIVE_PAUSE_RETRACT; 
										brakeSupTime = BRAKESUPTIME_SETUPPAUSE;
									}
								}
								brakeChange &= ~BRAKECHANGE_SUPTIME;
							}			
						}
					}
					break;
				}
			}
			}
			break;
		}
//----------------------------------
// error handling states
		case BRAKESTATE_ERROR_RETRACT_LOWVOLTAGE:
		{
				if (((brakeChange & BRAKECHANGE_SUPTIME)!=0)||(brakeSupTime==0))
				{
					MotorOff(1);
					brakeSupTime = 0;
					brakeChange &= ~BRAKECHANGE_SUPTIME;
					BrakeActuatorControl(BRAKE_HOME);
					brakeState = BRAKESTATE_ERROR_FINAL;
				}
			break;
		}
		case BRAKESTATE_ERROR_RETRACT:
		{
			switch (action)
			{
				case NONE:
				{
					if ((prevAction == EXTENDING)||(prevAction == EXTENDING_BY_ENCODER))
					{
						MotorOff(1);
						MotorCCW();
						brakeSupTime = BRAKESUPTIME_MEDIUM;
						brakeChange &= ~BRAKECHANGE_SUPTIME;
						brakeState = BRAKESTATE_ERROR_RETRACT_LOWVOLTAGE;
					}
					else
					{
						MotorOff(1);
						brakeSupTime = 0;
						brakeChange |= BRAKECHANGE_SUPTIME;
						brakeState = BRAKESTATE_ERROR_RETRACT_LOWVOLTAGE;
					}
					break;
				}
				case EXTENDING:
				case EXTENDING_BY_ENCODER:
				{
					MotorOff(1);
					MotorCCW();
					brakeSupTime = BRAKESUPTIME_MEDIUM;
					brakeChange &= ~BRAKECHANGE_SUPTIME;
					brakeState = BRAKESTATE_ERROR_RETRACT_LOWVOLTAGE;
					break;
				}
				default:
				{
					MotorOff(1);
					brakeSupTime = 0;
					brakeChange |= BRAKECHANGE_SUPTIME;
					brakeState = BRAKESTATE_ERROR_RETRACT_LOWVOLTAGE;
				}
			}
			break;
		}
		case BRAKESTATE_ERRORLOAD:
		{
			if (((brakeChange & BRAKECHANGE_SUPTIME)!= 0)||(encoderCountBack==0))
			{
				MotorOff(1);
				brakeState = BRAKESTATE_ERRORLOADWAIT;
				brakeSupTime = 10;  //1 second
				brakeChange &= ~BRAKECHANGE_SUPTIME;
			}		
			break;
		}
		case BRAKESTATE_ERRORLOADWAIT:
		{
			itemp3 = LoadCell(brakeState);
			if (itemp3<0x60)								
			{
     			brakeState = BRAKESTATE_ACTIVELOAD;
		//v01_29		brakeState = BRAKESTATE_ACTIVE;				
				brakeSupTime = 0;
				brakeChange &= ~BRAKECHANGE_SUPTIME;

			}
			else
			{
				if ((brakeChange & BRAKECHANGE_SUPTIME)!= 0) 
				{
					brakeStatus.BrakeState |= BRAKESTATE_NOTSETUP;
					brakeStatus.BrakeState |= BRAKESTATE_ERRORLOADSET;  //V01_28 added 
					brakeState = BRAKESTATE_ERROR;			
				}
			}
			break;
		}
		case BRAKESTATE_ERROR:
		{
			MotorOff(0);
			break;
		}		
		case BRAKESTATE_ERROR_FINAL:
		{
			MotorOff(0);
			if (setupExtendTriggered != 0)
			{
				brakeStatus.ActuatorStatus |= ACTUATORSTATUS_EXTENDTRIGGEREDINSETUP;
			}
			brakeStatus.BrakeState &= ~BRAKESTATE_MANUALBRAKE;
			if ((brakeStatus.BrakeState & BRAKESTATE_INPUTVOLTAGEBAD)== 0)
			{
				brakeState = BRAKESTATE_RESET;
				done = 1;
			}			
			break;
		}	
		case BRAKESTATE_ERROR_VOLTAGE_ACTIVE:
		{
			done = 0;
			//V00_60 - recover after voltage comes back
			if ((brakeStatus.BrakeState & BRAKESTATE_INPUTVOLTAGEBAD)== 0)
			{
				brakeState = BRAKESTATE_ACTIVE;
				done = 1;
			}
			if (done == 0)
			{
				//---------------------check for breakaway
				if ((breakawayRing_pressed == 0) ||
				((breakawayRing_pressed ==1) && (breakawayTip_pressed ==0)))
				{
					brakeStatus.BrakeState &= ~BRAKESTATE_BREAKAWAYTIP;
				}
				if ((breakawayTip_pressed!=0)&&(breakawayRing_pressed != 0)&&
				(breakawayHoldTimer ==0))
				{
					breakawayHoldTimer = 1;
				}
				if ((breakawayTip_pressed!=0)&&(breakawayRing_pressed != 0)&&
				(breakawayHoldTimer >= BREAKAWAY_HOLD_TIME)&&((brakeStatus.BrakeState &BRAKESTATE_BREAKAWAYTIP)==0))
				{
					done = 1;
					brakeStatus.BrakeState |= BRAKESTATE_BREAKAWAYTIP;
					brakeState = BRAKESTATE_ACTIVE_EXTEND_BREAKAWAY;
					MotorCW();
					brakeSupTime = BRAKESUPTIME;
					brakeChange &= ~BRAKECHANGE_SUPTIME;
					thresholdmet = 0;
				}			
			}
			break;
		}	
		case BRAKESTATE_SETUPACTIVE_END:
		{
			if (((brakeChange & BRAKECHANGE_SUPTIME)!= 0)||
			(hlimitState == 0)||(homeLimit== HOME_IN))
			{
				brakeSupTime = 10;
				brakeChange &= ~BRAKECHANGE_SUPTIME;
				MotorOff(1);
				brakeState = BRAKESTATE_HOLDOFF_ACTIVEFROMSETUP;
			}
			break;
		}		
		case BRAKESTATE_HOLDOFF_ACTIVEFROMSETUP:
		{
			itemp3 = LoadCell(brakeState);
			if (itemp3>0x60)								
			{
				brakeState = BRAKESTATE_ERROR_RETRACT;		
			}
			else
			{
				
				if ((brakeChange & BRAKECHANGE_SUPTIME)!=0)
				{			
					MotorOff(1);		
					if ((brakeStatus.BrakeState & BRAKESTATE_INPUTVOLTAGEBAD)!= 0)
					{
						brakeState = BRAKESTATE_ERROR_VOLTAGE_ACTIVE; 
					}		
					else
					{
						brakeState = BRAKESTATE_HOLDOFF_ACTIVE;				
						brakeSupTime = 10;		
					}
				}										 
				brakeChange &= ~BRAKECHANGE_SUPTIME; 			
			}		
			break;
		}
//-----------------------------------------		
		case BRAKESTATE_HOLDOFF_ACTIVE:
		{	
			if ((brakeStatus.BrakeState & BRAKESTATE_INPUTVOLTAGEBAD)!= 0)
			{
				brakeState = BRAKESTATE_ERROR_VOLTAGE_ACTIVE; 
			}		
			else
			{		
				if ((brakeStatus.BrakeState & BRAKESTATE_NOTSETUP)!= 0)
				{
					if ((brakeChange & BRAKECHANGE_SUPTIME)!=0)
					{
	//			itemp3 = ADCGetReading(ADC_INPUT_FSR);
						itemp3 = LoadCell(brakeState);
						if (itemp3>0x60)								
						{
							brakeState = BRAKESTATE_ERROR_RETRACT;								
						}
						else
						{
							MotorNeedNewBaseline();
							brakeStatus.BrakeState &= ~BRAKESTATE_NOTSETUP;
							brakeState = BRAKESTATE_HOLDOFF_ACTIVE;
							MotorOff(1);		
							brakeSupTime = BRAKESUPTIME;	
							//------------v1.05 boc
							brakeInitiationCount = 0;
							brakeCycleCount = 0;
							if (table0.Item.Hybrid == TRUE)
							{
								brakeInitiationCount = 10;
							}
							//--------------v1.05 eoc																				
						}										 
						brakeChange &= ~BRAKECHANGE_SUPTIME; 
					}			
				}
				else
				{
					gPrime = 0; 
					if ((breakawayRing_pressed == 0) || 
					   ((breakawayRing_pressed ==1) && (breakawayTip_pressed ==0)))
					{
						brakeStatus.BrakeState &= ~BRAKESTATE_BREAKAWAYTIP;
					}						
					thresholdmet = 0; 	
					tempdiffx = MotorGetAcc(FALSE);
					if ((brakeChange & BRAKECHANGE_SUPTIME)!= 0)
					{
						brakeChange &= ~BRAKECHANGE_SUPTIME;
						brakeState = BRAKESTATE_ACTIVE; 
					}
					encoderCountBackTotal = 0;
				}
			}
			break;
		}
		case BRAKESTATE_ACTIVE:
		case BRAKESTATE_ACTIVELOAD:
		{
			done = 0; 
			//------------------ v1.23 
			// check for pedal press. 
//			itemp3 = ADCGetReading(ADC_INPUT_FSR);
			itemp3 = LoadCell(brakeState);
//V01_29			if ((itemp3>0x60)&&(brakeState == BRAKESTATE_ACTIVE))		
			if (itemp3>0x60)										
			{
				
				MotorOff(1);	
				brakeSupTime = 10;  //2 seconds to EXTEND
				if (brakeState == BRAKESTATE_ACTIVE)  //V01_29
				{
					MotorCCW();
					brakeSupTime = 20;  //2 seconds to EXTEND
					//----- boc 1_23 ---- control by encode counts 
					encoderCountBack = ENCODER_RETRACT_ACTIVECOUNT; 
					action = RETRACTING_BY_ENCODER; 
				}
				//----- eoc 1_23 
				brakeChange &= ~BRAKECHANGE_SUPTIME;
				brakeState = BRAKESTATE_ERRORLOAD;
				done = 1; 									
			}
			
			if ((brakeStatus.BrakeState & BRAKESTATE_INPUTVOLTAGEBAD)!= 0)
			{
				brakeState = BRAKESTATE_ERROR_VOLTAGE_ACTIVE; 
				done = 1; 
			}					
			//---------------------check for breakaway
			if ((breakawayRing_pressed == 0) || 
			   ((breakawayRing_pressed ==1) && (breakawayTip_pressed ==0)))
			{
				brakeStatus.BrakeState &= ~BRAKESTATE_BREAKAWAYTIP;
			}								
 			if ((breakawayTip_pressed!=0)&&(breakawayRing_pressed != 0)&&
			        (breakawayHoldTimer ==0))
			{
				breakawayHoldTimer = 1; 
			}
 			if ((breakawayTip_pressed!=0)&&(breakawayRing_pressed != 0)&&(done==0)&& //v1_23 added the done here
			        (breakawayHoldTimer >= BREAKAWAY_HOLD_TIME)&&((brakeStatus.BrakeState &BRAKESTATE_BREAKAWAYTIP)==0))
			{
				done = 1;
				brakeStatus.BrakeState |= BRAKESTATE_BREAKAWAYTIP;
				brakeState = BRAKESTATE_ACTIVE_EXTEND_BREAKAWAY;
				MotorCW();
				brakeSupTime = BRAKESUPTIME;
				brakeChange &= ~BRAKECHANGE_SUPTIME;
				thresholdmet = 0;
			}			
			//---------------------check for manual
			if ((remoteStatus & REMOTE_MANUALBRAKE_ACTIVE) == 0)
			{
				brakeStatus.BrakeState &= ~BRAKESTATE_MANUALBRAKE;
			}			
			if (done == 0)
			{
				if (((remoteStatus & REMOTE_MANUALBRAKE_ACTIVE)!=0)&&
				       ((brakeStatus.BrakeState & BRAKESTATE_MANUALBRAKE)==0))
				{
 					done = 1;
					brakeStatus.BrakeState |= BRAKESTATE_MANUALBRAKE;
					brakeState = BRAKESTATE_ACTIVE_EXTEND_MANUAL;
					//------------v1.05 boc
					if (brakeInitiationCount <10)
					{
						brakeInitiationCount++;
					}
					//--------------v1.05 eoc
					MotorCW();
					brakeSupTime = BRAKESUPTIME;
					brakeChange &= ~BRAKECHANGE_SUPTIME;
				}
			}					
			//------------------------
			// check accelerometer. 
			if (done==0)
			{
				tempdiffx = MotorGetAcc(FALSE);
				itemp2 = table0.Item.SensitivitySet;;
				if (itemp2 >9)
				{
					itemp2 = 0;
				}
				itemp2 = itemp2 * ACC_THRESHOLD_MULTIPLIER;
				itemp2 = itemp2 + ACC_SIXTEENTHS_G;
			    if (AccelProvideDecisions(itemp2,DECISION_GREATER,motorAccXBaseline)!=0) //V062 was ACC_SIXTEENTHS_G
				{
					thresholdmet++; 
//					if (thresholdmet >= MAX_THRESHOLD_NEEDED)
//					{
						done = 1; 
						thresholdmet = MAX_THRESHOLD_NEEDED;
						brakeState = BRAKESTATE_ACTIVE_EXTEND;
						//------------v1.05 boc
						if (brakeInitiationCount <10)
						{
							brakeInitiationCount++;
						}
						//--------------v1.05 eoc						
						//----------------------------------
						// EXTEND - 
						// 1. set max time to 5 seconds
						// 2. set hold max time to 15 seconds 						
						MotorCW();
						thresholdmet = 0;   //01_39_#1
						brakeSupTime = BRAKESUPTIME;
						brakeChange &= ~BRAKECHANGE_SUPTIME;											
//					}
				}
				else
				{
					thresholdmet = 0; 
				}
			} 			
			else
			{
				thresholdmet = 0;				
			}

			break;
		}		

//------------------------- 
// ACTIVE EXTEND STATE 
//-------------------------		
		case BRAKESTATE_ACTIVE_EXTEND:
		{
			done = 0; 
			//------------------------
			// you are braking .... so 
			// 1. look for 500 counts on current to stop braking. 
			// 2. when at 1/10 g then retract 
			// 3. also maximum time of braking is 15 second. 
			// 
 			if ((flimitState == 0)&&(done == 0))
			{
				done = 1; 
				MotorOff(1);
				if (BrakeActuatorControl(BRAKE_AWAY)==BRAKE_ERROR)
				{
					brakeState = BRAKESTATE_ERROR_RETRACT;
				}
				else
				{
					thresholdmet = 0; 
					brakeState = BRAKESTATE_ACTIVE_HOLD; 
					brakeChange &= ~BRAKECHANGE_DITHER;
					ditherTimer = 0;  
					if (encoderFlip != 0)
					{
						encoderFillOffset = MAX_BUILDTABLE;
					}
					else
					{
						encoderFillOffset = encoderTableOffset; 
					}
				}
				
			}	
			if (((brakeChange & BRAKECHANGE_SUPTIME)!= 0)&&(done == 0))
			{
				brakeChange &= ~BRAKECHANGE_SUPTIME;
				MotorOff(1);	
				done = 1; 
				if (BrakeActuatorControl(BRAKE_RUN)==BRAKE_ERROR)
				{
					brakeState = BRAKESTATE_ERROR_RETRACT;
					brakeSupTime = 0;
				}
				else
				{
 					MotorCCW();
					brakeSupTime = BRAKESUPTIME;
					brakeState = BRAKESTATE_END_RETRACT_TIMEOUT;
				}
				brakeChange &= ~BRAKECHANGE_SUPTIME;
			}			
			if ((motorRunTime == 0)&&(done == 0))
			{
				currentRead = ADCGetReading(ADC_INPUT_CURRENT);
				newCurrentThreshold = CurrentMotorCalculated();
				//---------------v1.05 
				if (table0.Item.Hybrid == FALSE)
				{								
					if (brakeCycleCount <2)
					{
						newCurrentThreshold = CurrentMotorCalculatedStartup(2);
					}
					if (brakeCycleCount == 2)
					{
						newCurrentThreshold = CurrentMotorCalculatedStartup(table0.Item.ForceMaxSet/2);
					}					
				}
//V01_41 boc
				fsrRead = ADCGetReading(ADC_INPUT_FSR);
				newFSRThreshold = FsrMotorCalculated();
				//---------------v1.05
				if (table0.Item.Hybrid == FALSE)
				{
					if (brakeCycleCount <2)
					{
						newFSRThreshold = FsrMotorCalculatedStartup(2);
					}
					if (brakeCycleCount == 2)
					{
						newFSRThreshold = FsrMotorCalculatedStartup(table0.Item.ForceMaxSet/2);
					}
				}
//V01_41 eoc 				
				//--------------------v1.05
//v01_41	
#if FSR_USE			
				if (fsrRead > newFSRThreshold)
#else
				if (currentRead > newCurrentThreshold)   
#endif				
				{
					MotorOff(1);		
					//------------v1.05 boc
					if (brakeCycleCount <10)
					{
						brakeCycleCount++;
					}
					//--------------v1.05 eoc					
					if (BrakeActuatorControl(BRAKE_RUN)==BRAKE_ERROR)
					{
						done = 1; 
						brakeState = BRAKESTATE_ERROR_RETRACT;
						brakeSupTime = 0;
						brakeChange &= ~BRAKECHANGE_SUPTIME;
					}
					else
					{
						done =1; 
						thresholdmet = 0;
						brakeState = BRAKESTATE_ACTIVE_HOLD;  		
						brakeChange &= ~BRAKECHANGE_DITHER;
						ditherTimer = 0;  	
						holdG = instantGain; 	
						if (encoderFlip != 0)
						{
							encoderFillOffset = MAX_BUILDTABLE;
						}
						else
						{
							encoderFillOffset = encoderTableOffset;
						}													
					}
				}
			}
			//----------------------boc ----- v01_39_#1
			if (done == 0)
			{
				tempdiffx = MotorGetAcc(TRUE);				
				itemp2 = table0.Item.SensitivitySet;;
				if (itemp2 >9)
				{
					itemp2 = 0;
				}
				itemp2 = itemp2 * ACC_THRESHOLD_MULTIPLIER;
				itemp2 = itemp2 + ACC_TWENTYITH_G;
			    if (AccelProvideDecisions(itemp2,DECISION_LESS,motorAccXBaseline)!=0)
				{
					thresholdmet++; 
					if (thresholdmet > MAX_THRESHOLD_NEEDED)
					{
 						done = 1; 
						thresholdmet = 0;
						MotorOff(1);
						MotorCCW();
						brakeSupTime = BRAKESUPTIME;   
						brakeState = BRAKESTATE_END_RETRACT;  
						brakeChange &= ~BRAKECHANGE_SUPTIME;		
					}
				}
				else
				{
					thresholdmet = 0; 
				}			
			}			
			//---------- eoc v01_39_#1 
		}				
		case BRAKESTATE_ACTIVE_HOLD:
		{
			done = 0;
			//------------------------
			// you are braking .... so
			// 1. look for 500 counts on current to stop braking.
			// 2. when at 1/10 g then retract
			// 3. also maximum time of braking is 15 second.
			//
			if (((brakeChange & BRAKECHANGE_SUPTIME)!= 0)&&(done == 0))
			{
				brakeChange &= ~BRAKECHANGE_SUPTIME;
				MotorOff(1);		
				done = 1; 
				if (BrakeActuatorControl(BRAKE_RUN)==BRAKE_ERROR)
				{
					brakeState = BRAKESTATE_ERROR_RETRACT;
				}
				else
				{
 					MotorCCW();
					brakeSupTime = BRAKESUPTIME;
					brakeState = BRAKESTATE_END_RETRACT_TIMEOUT;
				}
				brakeChange &= ~BRAKECHANGE_SUPTIME;
			}			
			if (done == 0)
			{
				tempdiffx = MotorGetAcc(TRUE);				
				itemp2 = table0.Item.SensitivitySet;;
				if (itemp2 >9)
				{
					itemp2 = 0;
				}
				itemp2 = itemp2 * ACC_THRESHOLD_MULTIPLIER;
				itemp2 = itemp2 + ACC_TWENTYITH_G;
			    if (AccelProvideDecisions(itemp2,DECISION_LESS,motorAccXBaseline)!=0)
				{
					thresholdmet++; 
 					done = 1; 
					thresholdmet = 0;
					MotorOff(1);
					MotorCCW();
					brakeSupTime = BRAKESUPTIME;   
					brakeState = BRAKESTATE_END_RETRACT;  
					brakeChange &= ~BRAKECHANGE_SUPTIME;				
				}
				else
				{
					thresholdmet = 0; 
				}			
			}
#if 0  //NO DITHER			
			if ((done == 0)&&((brakeChange & BRAKECHANGE_DITHER)!=0))
			{
				brakeChange &= ~BRAKECHANGE_DITHER; 
				//--------------------------
				// at 100msec check gain value 
				newG = MotorGetAcc(TRUE);
				if (newG < holdG)
				{
					diffG = (holdG - newG); 
					if (diffG > ACC_DITHER_TRIGGER_G)
					{
						//-------------- Retract 
						thresholdmet = 0;
						brakeState = BRAKESTATE_ACTIVE_RETRACT;  		
						brakeChange &= ~BRAKECHANGE_DITHER;
						ditherTimer = 0;  	
						//---------------------
						// Calculate the current from the equation for the g 
						matchCurrent = CurrentMotorCalculatedForG(newG);
						encoderCountBack = MotorFindEncoderMatch(matchCurrent);
						if (encoderCountBack > MAX_ENCODERCOUNT_BACK)
						{
							encoderCountBack = MAX_ENCODERCOUNT_BACK; 
						}
						encoderCountBackTotal += encoderCountBack; 
						//--------------------------
						// Calculate new encoderTableOffset 
						if (encoderTableOffset < encoderCountBack)
						{
							itemp = encoderCountBack;
							itemp -= encoderTableOffset; 
							itemp = MAX_BUILDTABLE - itemp; 
							encoderTableOffset = itemp; 
						}
						else
						{
							encoderTableOffset-= encoderCountBack; 
						}
						RetractByEncoderCount();	
						holdG = newG; 	
					}
				}				
				else
				{
					diffG = (newG - holdG);
					if (diffG > ACC_DITHER_TRIGGER_G)
					{
						//---------------Extend 
						thresholdmet = 0;
						brakeState = BRAKESTATE_ACTIVE_EXTEND;  		
						brakeChange &= ~BRAKECHANGE_DITHER;
						ditherTimer = 0;  	
						MotorExtendMore(); 								
					} 
				}				
			}
#endif			
			break;
		}		
		case BRAKESTATE_ACTIVE_RETRACT:
		{
			done = 0;
 			if (((hlimitState == 0)||(homeLimit == HOME_IN))&&(done == 0)) //V01_26
			{
				done = 1; 
				if (BrakeActuatorControl(BRAKE_HOME)==BRAKE_ERROR)
				{
					brakeState = BRAKESTATE_ERROR_RETRACT;
					brakeSupTime = 0;
				}
				else
				{
//					MotorNeedNewBaseline();
					brakeState = BRAKESTATE_HOLDOFF_ACTIVE;
					MotorOff(1);	
					brakeSupTime = BRAKESUPTIMESHORT;
				}
			}
			if (((brakeChange & BRAKECHANGE_SUPTIME)!= 0)&&(done == 0))
			{
				done = 1; 
				brakeChange &= ~BRAKECHANGE_SUPTIME;
				MotorOff(1);		
				if (BrakeActuatorControl(BRAKE_RUN)==BRAKE_ERROR)
				{
					brakeState = BRAKESTATE_ERROR_RETRACT;
					brakeSupTime = 0;
				}
				else
				{
					MotorCCW();
					brakeSupTime = BRAKESUPTIME;
					brakeState = BRAKESTATE_END_RETRACT_TIMEOUT;
				}
			}			
			if ((done == 0)&&(action == NONE)&&((brakeChange & BRAKECHANGE_DITHER)!=0))
			{
				brakeChange &= ~BRAKECHANGE_DITHER; 
				//--------------------------
				// at 100msec check gain value 
				newG = MotorGetAcc(TRUE);
				if (newG < holdG)
				{
					diffG = (holdG - newG); 
					if (diffG > ACC_DITHER_TRIGGER_G)
					{
						//---------------------
						// Calculate the current from the equation for the g 
						matchCurrent = CurrentMotorCalculatedForG(newG);
						itemp = MotorFindEncoderMatch(matchCurrent);
						if (itemp > MAX_ENCODERCOUNT_BACK)
						{
							itemp = MAX_ENCODERCOUNT_BACK; 
						}
						encoderCountBackTotal += itemp; 
						//--------------------------
						if (encoderCountBackTotal <1000)
						{
							encoderCountBack = itemp; 
							RetractByEncoderCount();	
							holdG = newG; 	
						}
					}
				}			
				else
				{
					diffG = (newG - holdG);
					if (diffG > ACC_DITHER_TRIGGER_G)
					{
						//---------------Extend
						thresholdmet = 0;
						brakeState = BRAKESTATE_ACTIVE_EXTEND;
						brakeChange &= ~BRAKECHANGE_DITHER;
						ditherTimer = 0;
						MotorExtendMore();
 									
					} 
				}							
			}  			
			break;
		}
//--------------------------------
// END OF A BRAKING OPERATION
// RETRACTING.
//-------------------------------		
		case BRAKESTATE_END_RETRACT_TIMEOUT:
		case BRAKESTATE_END_RETRACT:
		case BRAKESTATE_END_RETRACT_BREAKAWAY:
		case BRAKESTATE_END_RETRACT_MANUAL:
		{
			breakawayHoldTimer = 0; 
 			if ((hlimitState == 0)||(homeLimit == HOME_IN))  //V01_26
			{
				MotorOff(1);		
				if (BrakeActuatorControl(BRAKE_HOME)==BRAKE_ERROR)
				{
					brakeState = BRAKESTATE_ERROR_RETRACT;
					brakeSupTime = 0;
				}
				else
				{
					brakeSupTime = BRAKESUPTIME_SHORT;
					if (brakeState == BRAKESTATE_END_RETRACT_TIMEOUT)
					{
						brakeSupTime = BRAKESUPTIME_TIMEOUT;	
					}
//					MotorNeedNewBaseline();
					brakeState = BRAKESTATE_HOLDOFF_ACTIVE;
					
					if ((remoteStatus & REMOTE_MANUALBRAKE_ACTIVE) == 0)
					{
						brakeStatus.BrakeState &= ~BRAKESTATE_MANUALBRAKE;
					}
				}
				brakeChange &= ~BRAKECHANGE_SUPTIME;
			}
			else
			{
				if ((brakeChange & BRAKECHANGE_SUPTIME)!= 0)
				{
					brakeChange &= ~BRAKECHANGE_SUPTIME;
					MotorOff(1);				
					if (BrakeActuatorControl(BRAKE_RUN)==BRAKE_ERROR)
					{
						brakeState = BRAKESTATE_ERROR_RETRACT;
						brakeSupTime = 0;
					}
					else
					{
						brakeSupTime = BRAKESUPTIME_SHORT;
						if (brakeState == BRAKESTATE_END_RETRACT_TIMEOUT)
						{
							brakeSupTime = BRAKESUPTIME_TIMEOUT;	
						}		
//						MotorNeedNewBaseline();				
						brakeState = BRAKESTATE_HOLDOFF_ACTIVE;
						if ((remoteStatus & REMOTE_MANUALBRAKE_ACTIVE) == 0)
						{
							brakeStatus.BrakeState &= ~BRAKESTATE_MANUALBRAKE;
						}
					}
					brakeChange &= ~BRAKECHANGE_SUPTIME;
				}
			}
			break;
		}		
//-----------------------
// BREAKAWAY 		
		case BRAKESTATE_ACTIVE_EXTEND_BREAKAWAY:
		{	
			brakeStatus.BrakeState |= BRAKESTATE_BREAKAWAYTIP;
			if ((breakawayRing_pressed == 0) || 
			   ((breakawayRing_pressed ==1) && (breakawayTip_pressed ==0)))
			{			
					done = 1;
					MotorCCW();
					brakeSupTime = BRAKESUPTIME;
					brakeState = BRAKESTATE_END_RETRACT_BREAKAWAY;
					brakeChange &= ~BRAKECHANGE_SUPTIME;
					brakeStatus.BrakeState &= ~BRAKESTATE_BREAKAWAYTIP;
			}
			if (((brakeChange & BRAKECHANGE_SUPTIME)!= 0)&&(done == 0))
			{
				done = 1;
				brakeChange &= ~BRAKECHANGE_SUPTIME;
				MotorOff(1);	
				if (BrakeActuatorControl(BRAKE_RUN)==BRAKE_ERROR)
				{
					brakeState = BRAKESTATE_ERROR_RETRACT;
					brakeSupTime = 0;
				}
				else
				{
					MotorCCW();
					brakeSupTime = BRAKESUPTIME;
					brakeState = BRAKESTATE_END_RETRACT_BREAKAWAY;
					brakeChange &= ~BRAKECHANGE_SUPTIME;
				}
			}						
 			if ((flimitState == 0)&&(done == 0))
			{
				done = 1; 
				if (BrakeActuatorControl(BRAKE_AWAY)==BRAKE_ERROR)
				{
					brakeState = BRAKESTATE_ERROR_RETRACT;
					brakeSupTime = 0;
				}
				else
				{
					MotorOff(1);				
					brakeState = BRAKESTATE_ACTIVE_HOLD_BREAKAWAY; 
				}
				brakeChange &= ~BRAKECHANGE_SUPTIME;
			}
			if ((motorRunTime == 0)&&(done == 0))
			{
				currentRead = ADCGetReading(ADC_INPUT_CURRENT);
				newCurrentThreshold = CurrentMotorCalculatedBreakaway();
				fsrRead = ADCGetReading(ADC_INPUT_FSR);
				newFSRThreshold = FsrMotorCalculatedBreakaway();				
#if FSR_USE				
				if (fsrRead > newFSRThreshold)   
#else
				if (currentRead > newCurrentThreshold)   	
#endif 				
				{
					MotorOff(1);		
					if (BrakeActuatorControl(BRAKE_RUN)==BRAKE_ERROR)
					{
						brakeState = BRAKESTATE_ERROR_RETRACT;
						brakeSupTime = 0;
					}
					else
					{
						brakeState = BRAKESTATE_ACTIVE_HOLD_BREAKAWAY;  			
					}
					brakeChange &= ~BRAKECHANGE_SUPTIME;
				}
			}			
			break;
		}
		case BRAKESTATE_ACTIVE_HOLD_BREAKAWAY:
		{
			done = 0;
			brakeStatus.BrakeState |= BRAKESTATE_BREAKAWAYTIP;
			//------------------------
			// you are braking ....  
			if ((breakawayRing_pressed == 0) || 
			   ((breakawayRing_pressed ==1) && (breakawayTip_pressed ==0)))
			{		
				//----------------------------
				// Hold until cleared by: break-away signal 
				// returning to normal, set-up button pressed 
				// on brake unit or cleared with remote.  	
				done = 1;
				MotorCCW();
				brakeSupTime = BRAKESUPTIME;
				brakeState = BRAKESTATE_END_RETRACT_BREAKAWAY;
				brakeChange &= ~BRAKECHANGE_SUPTIME;
				brakeStatus.BrakeState &= ~BRAKESTATE_BREAKAWAYTIP;
			}
			if (done == 0)
			{
				if ((button & KEY_SETUP)!=0)
				{
					if (setup_pressed == 1)
					{
						done = 1;
						brakeChange &= ~BRAKECHANGE_HOLDTIME;
						MotorCCW();
						brakeSupTime = BRAKESUPTIME;
						brakeState = BRAKESTATE_END_RETRACT_BREAKAWAY;
						brakeChange &= ~BRAKECHANGE_SUPTIME;
					}
				}	
			}		
			if (done == 0)
			{
				if ((remoteStatus & REMOTE_CLEARBREAKAWAY)!=0)
				{
						done = 1;
						brakeChange &= ~BRAKECHANGE_HOLDTIME;
						MotorCCW();
						brakeSupTime = BRAKESUPTIME;
						brakeState = BRAKESTATE_END_RETRACT_BREAKAWAY;
						brakeChange &= ~BRAKECHANGE_SUPTIME;
				}	
			}									
			break;
		}	
	//--------------------------------
	// MANUAL STATES. 
	//--------------------------------		
		case BRAKESTATE_ACTIVE_EXTEND_MANUAL:
		{
			if (((brakeChange & BRAKECHANGE_SUPTIME)!= 0)&&(done == 0))
			{
				done = 1;
				brakeChange &= ~BRAKECHANGE_SUPTIME;
				MotorOff(1);
				if (BrakeActuatorControl(BRAKE_RUN)==BRAKE_ERROR)
				{
					brakeState = BRAKESTATE_ERROR_RETRACT;
					brakeSupTime = 0;
				}
				else
				{
					MotorCCW();
					brakeSupTime = BRAKESUPTIME;
					brakeState = BRAKESTATE_END_RETRACT_MANUAL;
					brakeChange &= ~BRAKECHANGE_SUPTIME;
				}
			}			
			if ((flimitState == 0)&&(done == 0))
			{
				done = 1;
				if (BrakeActuatorControl(BRAKE_AWAY)==BRAKE_ERROR)
				{
					brakeState = BRAKESTATE_ERROR;
					brakeSupTime = 0;
				}
				else
				{
					MotorOff(1);
					brakeState = BRAKESTATE_ACTIVE_HOLD_MANUAL;
				}
				brakeChange &= ~BRAKECHANGE_SUPTIME;
			}
			if ((motorRunTime == 0)&&(done == 0))
			{
				currentRead = ADCGetReading(ADC_INPUT_CURRENT);
				newCurrentThreshold = CurrentMotorCalculatedManual();
				//---------------v1.05
				if (table0.Item.Hybrid == FALSE)
				{
					if (brakeCycleCount <2)
					{
						newCurrentThreshold = CurrentMotorCalculatedStartup(2);
					}
					if (brakeCycleCount == 2)
					{
						newCurrentThreshold = CurrentMotorCalculatedStartup(table0.Item.ForceMaxSet/2);
					}
				}	
				//V01_41 boc
				fsrRead = ADCGetReading(ADC_INPUT_FSR);
				newFSRThreshold = FsrMotorCalculatedManual();
				//---------------v1.05
				if (table0.Item.Hybrid == FALSE)
				{
					if (brakeCycleCount <2)
					{
						newFSRThreshold = FsrMotorCalculatedStartup(2);
					}
					if (brakeCycleCount == 2)
					{
						newFSRThreshold = FsrMotorCalculatedStartup(table0.Item.ForceMaxSet/2);
					}
				}
				//V01_41 eoc
				//--------------------v1.05
#if FSR_USE					
				if (fsrRead > newFSRThreshold)		
#else
				if (currentRead > newCurrentThreshold)
#endif									
				{
					MotorOff(1);
					//------------v1.05 boc
					if (brakeCycleCount <10)
					{
						brakeCycleCount++;
					}
					//--------------v1.05 eoc					
					if (BrakeActuatorControl(BRAKE_RUN)==BRAKE_ERROR)
					{
						brakeState = BRAKESTATE_ERROR_RETRACT;
						brakeSupTime = 0;
						brakeChange &= ~BRAKECHANGE_SUPTIME;
					}
					else
					{
						brakeState = BRAKESTATE_ACTIVE_HOLD_MANUAL;
					}
				}
			}
			//------------------------
			// you are braking ....
			if (done == 0)
			{
				//----------------------------
				// Hold until cleared by: break-away signal
				// returning to normal, set-up button pressed
				// on brake unit or cleared with remote.
				if ((remoteStatus & REMOTE_MANUALBRAKE_ACTIVE) == 0)
				{
					done = 1;
					MotorCCW();
					brakeSupTime = BRAKESUPTIME;
					brakeState = BRAKESTATE_END_RETRACT_MANUAL;
					brakeChange &= ~BRAKECHANGE_SUPTIME;
				}
			}			
			if (done == 0)
			{
				brakeStatus.BrakeState |= BRAKESTATE_MANUALBRAKE;
			}
			break;
		}	
		case BRAKESTATE_ACTIVE_HOLD_MANUAL:
		{
			done = 0;
			if (((brakeChange & BRAKECHANGE_SUPTIME)!= 0)&&(done == 0))
			{
				done = 1;
				brakeChange &= ~BRAKECHANGE_SUPTIME;
				MotorOff(1);
				if (BrakeActuatorControl(BRAKE_RUN)==BRAKE_ERROR)
				{
					brakeState = BRAKESTATE_ERROR_RETRACT;
					brakeSupTime = 0;
				}
				else
				{
					MotorCCW();
					brakeSupTime = BRAKESUPTIME;
					brakeState = BRAKESTATE_END_RETRACT_MANUAL;
					brakeChange &= ~BRAKECHANGE_SUPTIME;
				}
			}			
			//------------------------
			// you are braking ....
			if (done == 0)
			{
				//----------------------------
				// Hold until cleared by: break-away signal
				// returning to normal, set-up button pressed
				// on brake unit or cleared with remote.
				if ((remoteStatus & REMOTE_MANUALBRAKE_ACTIVE) == 0)
				{
					done = 1;
					MotorCCW();
					brakeSupTime = BRAKESUPTIME;
					brakeState = BRAKESTATE_END_RETRACT_MANUAL;
					brakeChange &= ~BRAKECHANGE_SUPTIME;
				}
			}
			if (done == 0)
			{
				brakeStatus.BrakeState |= BRAKESTATE_MANUALBRAKE;
			}
			break;
		}	
	}
	BrakeLEDControl();
}

//XXXXXXXXXXXXXXXXXXXXXXXXX DEACCELERATION DETECTION XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX 
int16_t motorTempDiffx = 0;

void MotorNeedNewBaseline(void)
{
	needNewBaseline=1;
	needNewBaselineTimer=NEEDNEWBASELINE_TIME; 
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:
//------------------------------------------------------------------------------
// This function
// NOTE: activeBraking does not matter swiched to the hlimitState to know this.
//==============================================================================
void MotorBuildGetAcc(void)
{
	int16_t mytempdiffx,newMotorx;
	uint8_t change;
	int32_t ltemp;
	
	mytempdiffx = 0;
	//------------------------
	// check accelerometer.
	// this is a non-zero return after every 20 sample average has been
	// attained.
	// needNewBaseline - allows the system to get a baseline reading in 10 seconds
	//   when some event requires this to happen -
	//   events noted today: power-up, just return to home, acceleration detected (-g)
	// DURING needNewBaseline - mytempdiffx returned will be zero 0000000000
	// needNewBaseline will be on a 10 second timer.
	//----------------------------------------
	if (AccelProvideReadingChange(&motorx,&motory,&motorz,&change)!= 0)
	{
//v01_26		if ((change != 0)&&(hlimitState == 0))
		if ((change != 0)&&((hlimitState == 0)||(homeLimit == HOME_IN)))		
		{
			newMotorx = (int16_t)motorx;
			ltemp = (int32_t)motorAccXBaseline;
			
			if (needNewBaseline != 0)
			{
				if (newMotorx >0)
				{
					ltemp = ltemp * 100;
					ltemp += newMotorx;
					ltemp = ltemp/101;
				}
				else
				{
					ltemp = ltemp * 100;
					ltemp += newMotorx;
					ltemp = ltemp/101;
				}
				if (motorAccBaseline == 0)
				{
					ltemp = newMotorx;
					motorAccBaseline = 1;
				}
				motorAccXBaseline = (int16_t)ltemp;
				motorAccYBaseline = (int16_t)motory;
				motorAccZBaseline = (int16_t)motorz;				
			}
			else
			{
				if (newMotorx >0)
				{
					ltemp = ltemp * 1000;
					ltemp += newMotorx;
					ltemp = ltemp/1001;
				}
				else
				{
					ltemp = ltemp * 5000;
					ltemp += newMotorx;
					ltemp = ltemp/5001;
				}
				if (motorAccBaseline == 0)
				{
					ltemp = newMotorx;
					motorAccBaseline = 1;
				}
				motorAccXBaseline = (int16_t)ltemp;
				motorAccYBaseline = (int16_t)motory;
				motorAccZBaseline = (int16_t)motorz;
			}
		}
		tempx = (int16_t)motorx;
		tempbasex = (int16_t) motorAccXBaseline;
		mytempdiffx = tempx- tempbasex;
		
	}
	if (needNewBaseline != 0)
	{
		mytempdiffx = 0; 
	}
	motorTempDiffx = mytempdiffx;
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:
//------------------------------------------------------------------------------
// This function
// NOTE: activeBraking does not matter swiched to the hlimitState to know this.
//==============================================================================
int16_t MotorGetAcc(uint8_t activeBraking)
{
	return motorTempDiffx;
}


//XXXXXXXXXXXXXXXXXXXXXXXXX MOTOR CURRENT G CALCULATIONS XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX 

#define FM 7
#define GAIN 5
#define RUSSELLAMPCHANGE	1

uint32_t ltemp;
uint8_t fm;
uint8_t gain;
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:
//------------------------------------------------------------------------------
// This function
//
//============================================================================== 
uint16_t CurrentMotorCalculated(void)
{
	uint32_t ltemp1,temp2;
	uint16_t returnCurrent;
	
	returnCurrent = 0;
	fm = table0.Item.ForceMaxSet;  
	gain = table0.Item.MaxForce;
	
	if ((fm<1)||(fm>9))
	{
		fm = 5;
	}
	if ((gain<1)||(gain>9))
	{
		gain = 5;
	}		 
	instantGain = MotorGetAcc(TRUE);
	if (instantGain > 0)
	{
//		instantGain = -1 * instantGain; 
//		instantGain =1; 
	}
	else
	{
		instantGain =0;  //1; 
//		instantGain = -1 * instantGain; 
	}
//	instantGain = ACC_POINT4_G;  //test
//	if (instantGain <= 0)
//	{
//		instantGain = -1 * instantGain; 
		//-----------------------
		// g' = g * G/5 and limite to .5g 
		// *multiply by 10 to get to whole numbers 
		temp2 = instantGain * (gain*10)/5;
		temp2 = temp2/10; 
		if (temp2 > ACC_HALF_G)
		{
			temp2 = ACC_HALF_G; 
		}
		ltemp = temp2;
		ltemp = ltemp * 100; 
		ltemp = ltemp/ACC_ONE_G;
		temp2 = ltemp; 		
		//----------------------------
		ltemp1 = temp2;
		gPrime = ltemp1; 
		ltemp1 = (ltemp1 * fm * 70 *RUSSELLAMPCHANGE);  //V00_82 from 100 to 70 V00_62  was *100 now *150
		ltemp1 = 19000 + ltemp1;  //V00_78
		ltemp1 = ltemp1 * 205; 
		ltemp1 = ltemp1/100;
		ltemp1 = ltemp1/100;
		returnCurrent = ltemp1; 
//	}
//	else
//	{
//		returnCurrent = 1; 
//	}
	return returnCurrent;
}
 
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:
//------------------------------------------------------------------------------
// This function
//
//==============================================================================
uint16_t FsrMotorCalculated(void)
{
	uint32_t ltemp1,temp2;
	uint16_t returnFsr;
	
	returnFsr = 0;
	fm = table0.Item.ForceMaxSet;
	gain = table0.Item.MaxForce;
	
	if ((fm<1)||(fm>9))
	{
		fm = 5;
	}
	if ((gain<1)||(gain>9))
	{
		gain = 5;
	}
	instantGain = MotorGetAcc(TRUE);
	if (instantGain > 0)
	{
		//		instantGain = -1 * instantGain;
		//		instantGain =1;
	}
	else
	{
		instantGain =0;  //1;
		//		instantGain = -1 * instantGain;
	}
	//	instantGain = ACC_POINT4_G;  //test
	//	if (instantGain <= 0)
	//	{
	//		instantGain = -1 * instantGain;
	//-----------------------
	// g' = g * G/5 and limite to .5g
	// *multiply by 10 to get to whole numbers
	temp2 = instantGain * (gain*10)/5;
	temp2 = temp2/10;
	if (temp2 > ACC_HALF_G)
	{
		temp2 = ACC_HALF_G;
	}
	ltemp = temp2;
	ltemp = ltemp * 100;
	ltemp = ltemp/ACC_ONE_G;
	temp2 = ltemp;
	//----------------------------
	ltemp1 = temp2;
	gPrime = ltemp1;
//	ltemp1 = (ltemp1 * fm * 2 *3686);   
	ltemp1 = (ltemp1 * fm * 2 *2600)+400;   	
	ltemp1 = ltemp1/9;  
	ltemp1 = ltemp1/100;
	
	returnFsr = ltemp1;
//	returnFsr = returnFsr/4; 
	returnFsr = returnFsr *2; 
	return returnFsr;
} 
 
 
uint16_t CurrentMotorCalculatedForG(uint16_t gin)
{
	
	uint32_t ltemp1,ltemp,temp2;
	uint8_t fm; 
	uint16_t returnCurrent,iGain;
	
	returnCurrent = 0;
	fm = FM;  
	fm = table0.Item.ForceMaxSet;
	gain = table0.Item.MaxForce;
	if ((fm<1)||(fm>9))
	{
		fm = 5;
	}
	if ((gain<1)||(gain>9))
	{
		gain = 5;
	}
			 
	iGain = gin;
	if (iGain != 0)
	{
		//-----------------------
		// g' = g * G/5 and limite to .5g 
		// *multiply by 10 to get to whole numbers 
		temp2 = iGain * (gain*10)/5;
		temp2 = temp2/10; 
		if (temp2 > ACC_HALF_G)
		{
			temp2 = ACC_HALF_G; 
		}
		ltemp = temp2;
		ltemp = ltemp * 100; 
		ltemp = ltemp/ACC_ONE_G;
		temp2 = ltemp; 		
		//----------------------------
		ltemp1 = temp2;
		ltemp1 = (ltemp1 * fm * 70*RUSSELLAMPCHANGE);  //V00_82 from 100 to 70 V00_62  was *100 now *150
		ltemp1 = 19000 + ltemp1;  //V00_78
		ltemp1 = ltemp1 * 205; 
		ltemp1 = ltemp1/100;
		ltemp1 = ltemp1/100;
		returnCurrent = ltemp1; 
	}
	return returnCurrent;
} 
 
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:
//------------------------------------------------------------------------------
// This function
// Imotor=1.5+Fm*0.56 
//==============================================================================
uint16_t CurrentMotorCalculatedStartup(uint8_t forceMax)
{
	uint32_t temp1;
	uint16_t returnCurrent;
	
	returnCurrent = 0;
//	fm = table0.Item.ForceMaxSet;
	fm = forceMax;
	if ((fm<1)||(fm>9))
	{
		fm = 5;
	}
	//----------------------------
	temp1 = 1500 + (fm * 560*RUSSELLAMPCHANGE);  //V00_62  was *375 now *560 //V00_78
	temp1 = temp1 * 205;
	temp1 = temp1/1000;
	returnCurrent = temp1;
	return returnCurrent;
} 
 
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:
//------------------------------------------------------------------------------
// This function
// Imotor=1.5+Fm*0.56
//==============================================================================
uint16_t FsrMotorCalculatedStartup(uint8_t forceMax)
{
	uint32_t temp1;
	uint16_t returnFSR;
	
	returnFSR = 0;
	//	fm = table0.Item.ForceMaxSet;
	fm = forceMax;
	if ((fm<1)||(fm>9))
	{
		fm = 5;
	}
	//----------------------------
	temp1 = (fm * 25* 3686);  //V00_62  was *375 now *560 //V00_78
	temp1 = temp1/9;
	temp1 = temp1/100;
	returnFSR = temp1;
//	returnFSR = returnFSR/4; 
	returnFSR = returnFSR *2; 
	return returnFSR;
} 
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:
//------------------------------------------------------------------------------
// This function
//
//============================================================================== 
uint16_t CurrentMotorCalculatedBreakaway(void)
{
	uint32_t temp1;
	uint16_t returnCurrent;
	
	returnCurrent = 0;
	fm = FM;  
	fm = table0.Item.ForceMaxSet;
	gain = table0.Item.MaxForce;
	if ((fm<1)||(fm>9))
	{
		fm = 5;
	}
	if ((gain<1)||(gain>9))
	{
		gain = 5;
	}
 	//----------------------------
	temp1 = 1900 + (fm * 375*RUSSELLAMPCHANGE);  //V00_62  was *375 now *560 //V00_78
	temp1 = temp1 * 205;
	temp1 = temp1/1000;
	returnCurrent = temp1;
	return returnCurrent;
} 
 
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:
//------------------------------------------------------------------------------
// This function
//
//==============================================================================
uint16_t FsrMotorCalculatedBreakaway(void)
{
	uint32_t temp1;
	uint16_t returnFsr;
	
	returnFsr = 0;
	fm = FM;
	fm = table0.Item.ForceMaxSet;
	gain = table0.Item.MaxForce;
	if ((fm<1)||(fm>9))
	{
		fm = 5;
	}
	if ((gain<1)||(gain>9))
	{
		gain = 5;
	}
	//----------------------------
	temp1 = (fm * 5*3686);  
	temp1 = temp1/9;
	temp1 = temp1/10;
	returnFsr = temp1;
//	returnFsr = returnFsr/4; 
	returnFsr = returnFsr *2; 
	return returnFsr;
} 
 
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:
//------------------------------------------------------------------------------
// This function
//
//============================================================================== 
uint16_t CurrentMotorCalculatedManual(void)
{
	uint32_t temp1;
	uint16_t returnCurrent;
	
	returnCurrent = 0;
	fm = table0.Item.ForceMaxSet;  
	gain = table0.Item.MaxForce;
	if ((fm<1)||(fm>9))
	{
		fm = 5;
	}
	if ((gain<1)||(gain>9))
	{
		gain = 5;
	}
	//----------------------------
	temp1 = 1900 + (fm * gain * 56*RUSSELLAMPCHANGE); //V00_62  was *56 now *84 //V00_78
	//---------------------
	// to make it 75% divide now by 100
	//---------------------
	temp1 = temp1 * 205;
	temp1 = temp1/1000;
	returnCurrent = temp1;
	return returnCurrent;
} 
 
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:
//------------------------------------------------------------------------------
// This function
//
//==============================================================================
uint16_t FsrMotorCalculatedManual(void)
{
	uint32_t temp1;
	uint16_t returnFsr;
	
	returnFsr = 0;
	fm = table0.Item.ForceMaxSet;
	gain = table0.Item.MaxForce;
	if ((fm<1)||(fm>9))
	{
		fm = 5;
	}
	if ((gain<1)||(gain>9))
	{
		gain = 5;
	}
	//----------------------------
	temp1 = (fm * gain * 3686); 
	//---------------------
	//  
	//---------------------
	temp1 = temp1/9;
	temp1 = temp1/9;
	returnFsr = temp1;
//	returnFsr = returnFsr/4; 
	returnFsr = returnFsr *2; 
	return returnFsr;
} 
 
//XXXXXXXXXXXXXXXXXXXXXXXXX MOTOR SETUP STUFF AND RUNNING XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX 

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:
//------------------------------------------------------------------------------
// This function
//==============================================================================
void ConfigureMotorFLimitChannel(void)
{
	struct extint_chan_conf config_extint_chan;
	extint_chan_get_config_defaults(&config_extint_chan);
	config_extint_chan.gpio_pin = PIN_PA21A_EIC_EXTINT5;
	config_extint_chan.gpio_pin_mux = MUX_PA21A_EIC_EXTINT5;
	config_extint_chan.gpio_pin_pull = EXTINT_PULL_NONE;
	config_extint_chan.detection_criteria = EXTINT_DETECT_BOTH;
	extint_chan_set_config(5, &config_extint_chan);
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:
//------------------------------------------------------------------------------
// This function
//==============================================================================
void ConfigureMotorFLimitCallbacks(void)
{
	extint_register_callback(MotorFLimitCallback,5,EXTINT_CALLBACK_TYPE_DETECT);
	extint_chan_enable_callback(5,EXTINT_CALLBACK_TYPE_DETECT);
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:
//------------------------------------------------------------------------------
// This function
//==============================================================================
void MotorFLimitCallback(void)
{
	schedByte |= SCHEDBYTE_MOTORFLIMIT;
	flimitState = port_pin_get_input_level(FLIMIT);		
	if ((flimitState == 0)&&(action!=RETRACTING))
	{
		MotorOff(0);
	}
}


//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:
//------------------------------------------------------------------------------
// This function
//==============================================================================
void ConfigureMotorHLimitChannel(void)
{
	struct extint_chan_conf config_extint_chan;
	extint_chan_get_config_defaults(&config_extint_chan);
	config_extint_chan.gpio_pin = PIN_PA20A_EIC_EXTINT4;   
	config_extint_chan.gpio_pin_mux = MUX_PA20A_EIC_EXTINT4;   
	config_extint_chan.gpio_pin_pull = EXTINT_PULL_NONE;
	config_extint_chan.detection_criteria = EXTINT_DETECT_BOTH; 
	extint_chan_set_config(4, &config_extint_chan);   
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:
//------------------------------------------------------------------------------
// This function
//==============================================================================
void ConfigureMotorHLimitCallbacks(void)
{
	extint_register_callback(MotorHLimitCallback,4,EXTINT_CALLBACK_TYPE_DETECT);
	extint_chan_enable_callback(4,EXTINT_CALLBACK_TYPE_DETECT);
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:
//------------------------------------------------------------------------------
// This function
//==============================================================================
void MotorHLimitCallback(void)
{
	schedByte |= SCHEDBYTE_MOTORHLIMIT;
	hlimitState = port_pin_get_input_level(HLIMIT);		
	
//	if ((brakeStatus.BrakeState & (BRAKESTATE_INPUTVOLTAGEBAD|BRAKESTATE_NOINPUTVOLTAGE))!= 0)
//	{
//	}
//	else
//	{
		if (hlimitState == 0)
		{
			if ((action != EXTENDING)&&(action !=EXTENDING_BY_ENCODER))
			{
				MotorOff(0);
				homeLimit = HOME_IN;
			}
			else
			{
				homeLimit = HOME_OUT;
			}
		}
		else
		{
			// added below logic v01_27
			if ((action != EXTENDING)&&(action !=EXTENDING_BY_ENCODER))
			{
	//			MotorOff();
				homeLimit = HOME_IN;
			}
			else
			{
				homeLimit = HOME_OUT;
			}
		}
//	}
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:
//------------------------------------------------------------------------------
// This function
//
//==============================================================================
void MotorHLimitTask(void)
{
	hlimitCount++;	
	hlimitState = port_pin_get_input_level(HLIMIT);	
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:
//------------------------------------------------------------------------------
// This function
//
//==============================================================================
void MotorFLimitTask(void)
{
	flimitCount++;
	flimitState = port_pin_get_input_level(FLIMIT);
	
	if ((flimitState == 0)&&(action!= RETRACTING))
	{
		MotorOff(1);
	}	
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:
//------------------------------------------------------------------------------
// This function
//
//==============================================================================
void MotorInit(void)
{
	hlimitCount = 0;
	flimitCount = 0; 
	
	ConfigureMotorHLimitChannel();
	ConfigureMotorHLimitCallbacks();
	
	ConfigureMotorFLimitChannel();
	ConfigureMotorFLimitCallbacks();	
	
	flimitState = port_pin_get_input_level(FLIMIT);
	hlimitState = port_pin_get_input_level(HLIMIT);	
	
	ConfigureEncoder();
	ConfigureEncoderCallbacks();
	encoderCount = 0; 
	buildTableOffset = 0;	
}
 
//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:   MotorOff
//------------------------------------------------------------------------------
// This function Initializes registers to allow button interrupts
//==============================================================================
void MotorOff(uint8_t useHoldOff)
{
	struct port_config pin_conf;
	port_get_config_defaults(&pin_conf);	
  
	motorOn = FALSE; 
	motorRunTime = 0;	
	pin_conf.direction  = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(ENa, &pin_conf);
	port_pin_set_output_level(ENa, false);
	port_pin_set_config(ENb, &pin_conf);
	port_pin_set_output_level(ENb, false);
	port_pin_set_config(INa, &pin_conf);
	port_pin_set_output_level(INa, false);
	port_pin_set_config(INb, &pin_conf);
	port_pin_set_output_level(INb, false);
	
	action = NONE; 
	if (useHoldOff != 0)
	{
		BrakeHoldOff(100);
	}
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:   MotorCW
//------------------------------------------------------------------------------
// This function
//==============================================================================
void RetractByEncoderCount(void)
{
	struct port_config pin_conf;
	port_get_config_defaults(&pin_conf);
	
	motorOn = TRUE; 
	port_pin_set_output_level(INa, false);
	port_pin_set_output_level(INb, true);
	pin_conf.direction  = PORT_PIN_DIR_INPUT;
	pin_conf.input_pull = PORT_PIN_PULL_NONE;
	port_pin_set_config(ENa, &pin_conf);
	port_pin_set_config(ENb, &pin_conf);
	action = RETRACTING_BY_ENCODER; 
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:   MotorCW
//------------------------------------------------------------------------------
// This function
//==============================================================================
void MotorCCW(void)
{
	struct port_config pin_conf;
	port_get_config_defaults(&pin_conf);
	
	motorOn = TRUE; 
	motorRunTime = MOTOR_RUN_TIME;	
	maxCurrentRead = 0; 
	maxFSRRead = 0;  //V01_41
	encoderCount = 0;
	buildTableOffset = 0;	
	port_pin_set_output_level(INa, false);
	port_pin_set_output_level(INb, true);
	pin_conf.direction  = PORT_PIN_DIR_INPUT;
	pin_conf.input_pull = PORT_PIN_PULL_NONE;
	port_pin_set_config(ENa, &pin_conf);
	port_pin_set_config(ENb, &pin_conf);
	action = RETRACTING; 
	prevEncoderCount = encoderCount; //01_38_#3
	actionEncoderSample = 0; //01_38_#3
	prevAction = action; 
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:   MotorCW
//------------------------------------------------------------------------------
// This function  
//==============================================================================
void MotorCW(void)
{
	struct port_config pin_conf;
	port_get_config_defaults(&pin_conf);	

	motorOn = TRUE; 
	motorRunTime = MOTOR_RUN_TIME;	
	maxCurrentRead = 0;
	maxFSRRead = 0;  //V01_41
	encoderCount = 0;	
	encoderFlip = 0; 
	buildTableOffset = 0;
	encoderTableOffset = 0; 
	port_pin_set_output_level(INa, true);
	port_pin_set_output_level(INb, false);
	pin_conf.direction  = PORT_PIN_DIR_INPUT;
	pin_conf.input_pull = PORT_PIN_PULL_NONE;
	port_pin_set_config(ENa, &pin_conf);
	port_pin_set_config(ENb, &pin_conf);
	action = EXTENDING; 
	homeLimit = HOME_OUT;
	prevEncoderCount = encoderCount; //01_38_#3
	actionEncoderSample = 0; //01_38_#3
	prevAction = action; 
}

//xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
// FUNCTION:   MotorCW
//------------------------------------------------------------------------------
// This function
//==============================================================================
void MotorExtendMore(void)
{
	struct port_config pin_conf;
	port_get_config_defaults(&pin_conf);

	motorOn = TRUE;
	motorRunTime = MOTOR_RUN_TIME;
	maxCurrentRead = 0;
	maxFSRRead = 0;  //V01_41
//	encoderCount = 0;
//	encoderFlip = 0;
	buildTableOffset = 0;
//	encoderTableOffset = 0;
	port_pin_set_output_level(INa, true);
	port_pin_set_output_level(INb, false);
	pin_conf.direction  = PORT_PIN_DIR_INPUT;
	pin_conf.input_pull = PORT_PIN_PULL_NONE;
	port_pin_set_config(ENa, &pin_conf);
	port_pin_set_config(ENb, &pin_conf);
	action = EXTENDING;
	
}


#endif
