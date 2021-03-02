//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//                FILE:
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// Processor:
// TOOLS:
// DATE:
// CONTENTS: This file contains
//------------------------------------------------------------------------------
// HISTORY: This file
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
#ifndef SAMD20_XPLAINED_PRO_H_INCLUDED
#define SAMD20_XPLAINED_PRO_H_INCLUDED

#include <conf_board.h>
#include <compiler.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


 
#define MAXSENSORS 12 
 
#define TRUE 1
#define FALSE 0

#define TESTUARTDATA 0
#define BLUETOOTH_TEST 0	 
#define REMOTEBOARD 0
#define BRAKEBOARD  1
#define TEST_TIMER 0
#define TEST_BRAKE_SUP 0
#define FSRTEST 1
#define FSR_USE 0

//#define RADIO_CONTINUOUS 1
#define TEST_RADIO_CONTINUOUS 0
#define TESTTIMER 1
#define REMOTE_TIREON 0

#define APP_SCRATCH_BASE 0x21000

#define HYBRID_FORCE_SETTING 2

#if REMOTEBOARD
//=========================REMOTE BOARD ======================
// v00_13 02 24 2016 - Added actuator failure values to new status screen
// v00_15 03 20 2016 - 1. moved lora restart on failures to left often on remote moved 
//                        to commFailureCount 
// V00_19 05 15 2016 - Added and turned on Continuous mode 
// V00_21 05 19 2016 - added code to send max force value in protocol to brake
// V00_34 07 28 2016 - made changes to pair setting to tell user to press setup button
// V00_35 09 07 2016 - Added checksum and lengths to protocol messages. 
// V00_40 10 20 2016 - baseline sent to russell
// V00_42 11 05 2016 - Fixed the acceleration trigger i hope - had to move 
//                        math variables to int32 in AccelProvideDecisions.
//                     Added a needNewBaseline for rolling averages to get initial 
//                        values faster and not return bad items too fast. 
// v00_45 11 11 2016 - Added green LCD backlight when brake active
//                     Added red LCD backlight when brake in error state
//                     Got graph on HOME screen running
// v00_46 11 11 2016 - added RESET BRAKE on home display when in error state
// 
// v00_53 11 20 2016 - Added FCC support of reading 3 inputs and performing 
// v00_54 11 20 2016 - Changed frequency to 922.33mhz
// v00_57 12 04 2016 - 1. changed text on error message from "LOW BATTERY" to "Check Power 
//                        Connection on Brake"
//                     2. fixed manual brake screen and fixed the flickering box issues. 
// v00_61 12 26 2016 - 1. 
// v00_63 12 30 2016 
// v00_64 12 30 2016
// v00_76 01 01 2017   1. Added H in FORCE on home screen if in HYBRID MODE. 
//                     2. made hybrid force setting to 2 
//                     3. It looks like the Brk/Away ready message appears when 
//                        you turn on the remote. Can you make sure that only appears 
//                        when the breakaway cable is actually plugged in?
//					   4. Moved thresholds for activation and deactivation back to .0625 and .05 
//                          respectively.
//					5.	addED an activation sensitivity setting.
// V00_77 01 02 2017 1. Moved hybrid setting to max force screen ... goes H-0-1 .... 9 will not rollover in 
//                      either direction as requested. 
//                   2. added use of the sensitivity adjustments on the brake 
//                   3. returned calculation force values back to september settings that have been used. 
// v00_78 01 03 2017 1. change to motor calculations
// V00_83 01 05 2017 1) When force is set to Hybrid mode, it overrides force and max force settings. (Force =3 Max force =4)
//					 2) On the brake you need to hold setup button for 5 seconds to enable Bluetooth mode
//					 3) Enable setup on production version
//					 4) Display current version of both remote and brake on remote screen 
//							somewhere (probably in the setup menu)
//					 6) There is a blinking text box on low battery error screen (top of the screen). Please fix.
// V00_84 01 05 2017 1. Watchdog enabled. 
// V00_86 01 06 2017 Can you have the FORCE reset to 5, the MAX FORCE reset to 7 and 
//                   SENSITIVITY reset to 9? Right now it mentions something about 
//                   hybrid mode, I don�t think this is necessary anymore
//					 Right now it just says setup brake to continue. Can you change this to:
//								REPOSITION BRAKE
//									THEN
//								RESTART UNIT
// v01_11 05 03 2017 1. BRAKE BOARD - changed the GREEN FLICKER to GREEN SOLID
//				     2. BRAKE BOARD - POWER UP on depress - change from on release
// V01_13 06 04 2017 1. change to 919.25mhz
// v01_20 09 09 2018 1. built on 01_14 ... basically 01_12 that is shipping 
// V01_28 11 04 2018 1. Added red light when load error 
// v01_40 05 09 2020 1. fills g prime with +10 
// v01_92 02 14 2021 1. added in conditional for brakeStatus & BRAKESTATE_ERRORLOADSET
//                        to display setup brake error and be red
#define FWVER3 '0'
#define FWVER2 '1'
#define FWVER1 '9'
#define FWVER0 '2'

#define MONTHMSB  '0'
#define MONTHLSB  '2'
#define DAYMSB    '1'
#define DAYLSB    '4'
#define YEARMSB   '2'
#define YEARLSB   '1'

#else
//=========================BRAKE BOARD ======================
// v00_13 02 22 2016 - INCREASED motor run time from 100msec to 400msec
// v00_14 02 22 2016 - moved current limits back to russells numbers for him
// V00_16 03 30 2016 - updated part of the motor brake algorithm
// V00_18 05 03 2016 - added the retract dither part of the operation
// V00_19 05 15 2016 - Added and turned on continuous mode 
// v00_20 05 15 2016 - added use of max force and update in table for use. 
//                   - made power key end anything. 
// V00_24 06 01 2016 - 1. accelerometer 20 msec average / 5 in a row for decisions
//                     
// v00_25 06 02 2016 - 1. made a change to accelerometer was not reading every 1mec
//                       faster response
//                     2. 100/500 for baseline
// v00_26 06 02 2016   1. made 50/250 for baseline
// v00_27 06 03 2016   1. 
// v00_28 06 04 2016 
// v00_29 06 06 2016   1. added in dynamic gain by receiving from remote
// v00_30 06 10 2016   1. loaded in initial baseline 
// v00_31 06 26 2016   1. dithering back in 
//                     2. added force max set use 
//                     3. updated equations. 
// V00_34 07 28 2016 - made changes to pair setting to tell user to press setup button
// V00_35 09 07 2016 - Added checksum and lengths to protocol messages. 
//                     added supercap logic of turning on, only if brake on and voltage >10.5
// V00_40 10 20 2016 - Updated motor equations to 1.50 added setup test back in for russell to ship
// V00_42 11 05 2016 - Fixed the acceleration trigger i hope - had to move
//                        math variables to int32 in AccelProvideDecisions.
//                     Added a needNewBaseline for rolling averages to get initial
//                        values faster and not return bad items too fast.
// V00_50 11 13 2016 - created a version that just transmits on the BRAKE 933mhz continuous
// v00_53 11 20 2016 - Added FCC support of reading 3 inputs and performing
// v00_54 11 20 2016 - Changed frequency to 922.33mhz
// v00_55 11 20 2016 - FCC moved to using power button on brake
// V00_56 11 21 2016
// v00_57 12 04 2016 - 1. Added 1/2 second timer before breakaway activates 
//                     2. moved the "setup" strokes from 3.75 to 4.5 amps to attempt 
//                         to add 30% more pressure. 
//                     3. When there is a 'check power connection' on brake - will not 
//                         allow brake to extend - goes to error state that must cycle
//                         power on. 
// V00_60 12 25 2016  
//						1. �Check power connection� error should have a yellow background 
//                          on the screen (currently has red). After a couple seconds, error 
//                          becomes non-reversible. (remote says RESET BRAKE) 
//							Can we return the brake to normal operation when power is restored.
//						2. Brake does not activate when it has a �check power connection� error which is good. 
//							Can you have it so that the breakaway still works?
//						3. fixed. If you have a breakaway event that goes away, the �Force� number on the remotes screen flickers. 
//						   While it is flickering the first button you press on the remote does not do anything. 
//						   The flickering then goes away and the remote works like normal. (example is breakaway goes off 
//						   and then goes away. If I press the �LIGHT� button, it will not turn the backlight on or off, 
//						   but it does clear the flickering)
//						4. fixed. If you have an error and then the brake turns off, the remote screen stays red 
//                         until you turn the brake back on. (or unplug the remote)
//						5. addED �BRAKE ACTIVE� text above the triangle graph when the brake activates?
//						6. Added the option �Backlight when braking� to control whether the 
//							backlight comes on when the brake activates?
//						7. Does the �Sensitivity� setting option change anything? NO. Put in Hydrid in place. 
//						8. Removed  the �BREAKAWAY MISSING� text? Only have text �BREAKAWAY READY� 
//							text show up when breakaway is plugged in
// v00_62 12 30 2016    1. New motor calculation values
// v00_65 12 30 2016
// v00_66 12 30 2016
// v00_74 12 31 2016    1. changing how downloads of remotes are handled. 
// v00_91 02 21 2017    1. changed output power from 10 to 20 
// v00_92 02 22 2017    1. changed to 15 from 20 ... remote was rebooting 
// v01_07 04 02 2017    1. ran testing with I2C found only 300khz works 100 and 200 are failing 
//                           not sure why.
//                      2. added recovery routine to i2c to cycle 16 clocks when an error is 
//                           detected. 
// V01_08 04 02 2017    1. Added back in additional bluetooth commands. 
// v01_20 09 09 2018 1. built on 01_14 ... basically 01_12 that is shipping
// v01_21 09 23 2018 1. fixed ADC reading for FSR was on the wrong channel 
// v01_22 10 09 2018 1. addded logic to setup and force detected 
// v01_23 10 14 2018 1. added items from Friday 10/12 to 10/14 
// v01_24 10 15 2018 1. had left testing "testvalue" in for on load cell readings 
// v01_25 10 16 2018 1. fixes from russel's email on 10/15
// v01_28 11 04 2018 1. in ACTIVE state will allow a load event that goes away will 
//                      allow activation again.
//                   2. moved load time to 1 second from 100msec continuous
// v0_31 04 13 2019  1. added fix to deal with home detection and loose thing


//brake board   was 01_37
// v01_39 03 08 2020 1. added in deacceleration check and retract if deaccelerating
// v01_40 05 05 2020 1. added *2 multiplier in current calculations
// v01_41 06 04 2020 1. change out equations for force 
// v01_43 06 29 2020 1. fixed the FsrMotorCalculated (needed a division by 100).
// v01_44 06 30 2020 1. reworked when seeing no change in encoder when moving to move to error 
//                       state 
// v01_45 07 05 202  1. added EXTENDING_BY_ENCODER in check for encoder counts
// brake board 
// v01_46 07 07 2020 1. increased the time for encoder checks to 3seconds so it extends some. 
//                   2. changed that the BRAKESTATE_ERROR_FINAL will make remote flash red screen
//                   3. changed that BRAKESTATE_ERROR_FINAL will flash the bi led RED
// v01_47 07 09 2020 1. added retracting ot be same for the non working encoder
// v01_48 07 12 2020 1. forcing setup FsrMotorCalculatedStartup to (3)
// v01_52 07 18 2020 1. disabled fsr and moved back to current use. 
// v01_53 07 19 2020 1. added BrakeHoldOff to be used after stopping motor
// v01_54 07 21 2020 1. removed *2 in current calculations & added holdoff check in motorOff
// v01_56 07 23 2020 1. added motor off pause after decleration and 
//                   2. will not allow setup if NO input voltage detected
// v01_57 08 09 2020 1. found in BRAKESTATE_ACTIVE_HOLD when accelerometer deaccelerates, was not doing 
//                      a motor off. fixed. 
//                   2. added yellow LED if no voltage input on during setup
//                   3. added a fix for a issue seen by the guys ... reworked that if extended 
//                      from acceleration and go to error - will start the retract 
// v01_58 09 16 2020 1. added in supervision of input voltage. if less than 10.2 and supercap not there
//                     indicate error
// V01_59 10 01 2020 1. LOTS of changes in error handling and checking on last action to see if 
//                         need to retract. 
// v01_60 10 04 2020 1. improved the ERROR retract portion
//                   2. on power up if no or low voltage will not enter setup
// v01_61 10 14 2020 1. reworked low power voltage issues and made sure that error states are all checked
//                          and updates LED to yellow when needed. 
// v01_62 10 17 2020 1. reworked low power in that even if OFF will read the ADC. 
//                   2. before extension, if low will not proceed
//                   3. will charge supercap before clearing a low input voltage. 
//                   4. all error states will be able to turn the unit off. 
// v01_63 10 19 2020 1. 500 msec input low error from 3 sec
//                   2. fresh on each turn off of low voltge errors
//                   3. added WAIT_ON_SETUP to 25msec sampling
//                   4. load wait back to ERROR 
// v01_64 10 20 2020 1. added clearing on errors when power is cycled. 
//                   2. 
// v01_65 10 29 2020 1. removed supercap value when in IDLE_SLEEP to be a certain value to 
//                        clear an input voltage error. 
//                   2. lowered voltage error back to 8V on input voltage errors. 
// v01_66 10 30 2020 1. reduced the 8.5v to 8v
// v01_67 11 02 2020 1. adjusted timing out for low voltage detet 
// v01_68 11 05 2020 1. moved voltage bad time to 3 seconds. 
// v01_70 11 05 2020 1. 2 sec 
// v01_70 11 05 2020 1. 1 sec
// V01_72 11 19 2020 1. NORMAL 8 VOLTS
// v01_73 11 19 2020 1. 7 volts 
// v01_74 11 19 2020 1. 6 volts
// v01_75 12 06 2020 1. 8 volts for pre-setup time and leaving at 6v for setup period
// v01_76 12 15 2020 1. added check for input voltage in 2 states before waiting for extension
//                      to complete. POWEREDUP0 and PRESET0
// V01_77 12 17 2020 1. DROPPED the 25msec to 15msec on input voltage bad
// V01_85 02 02 2021 1. FIXED supervisory task time - moved to 1mec execution to allow dropping to 
//                      15msec window. 
//                   2. fixed active operation was missing a "break"
// v01_86 02 03 2021 1. got rid of use of prevBrakeState when an extension has happened during a 
//                       ignored load error
// v01_87 02 03 2021 1. changed sup time after retract and home from 10sec to 2 sec
// v01_88 02 04 2021 1. changed hold off time after load error detected in BRAKESTATE_HOLDOFF_ACTIVE
//                           from 10 sec to 2 sec
// v01_89 02 04 2021
// v01_90 02 05 2021 1. undid the 01_86 to 01_87 change. 
// v01_93 02 14 2021 1. just retaining orange led when a load error 
//                         has been detected, even if it has cleared. 
// v01_94 02 17 2021 1. added patch to force BRAKE_ERROR state when load error bit is set
// v01_95 02 18 2021 1. checks for breakaway and manual and will not mask if those present. 
// v01_97 02 26 2021 1. fixed triangle on display 
//                   2. 
// v01_98 03 01 2021 1. fixed retract time out and broken triangle 

#define FWVER3 '0'
#define FWVER2 '1'
#define FWVER1 '9'
#define FWVER0 '8'

#define MONTHMSB  '0'
#define MONTHLSB  '3'
#define DAYMSB    '0'
#define DAYLSB    '1'
#define YEARMSB   '2'
#define YEARLSB   '1'
#endif

extern int wdogTimer;
 
void system_board_init(void);
 
extern uint8_t switchOnTransmit; 
 
#define WHICHRADIO_FSK	0x88
#define WHICHRADIO_LORA 0x99 
extern uint8_t whichRadio;  
 
extern unsigned int schedByte; 
#if BRAKEBOARD
#define SCHEDBYTE_BRAKESUP   			0x0001
#endif 

#if REMOTEBOARD
#define SCHEDBYTE_SCREENREFRESH			0x0002
#else
#define SCHEDBYTE_MOTORHLIMIT			0x0002
#endif
#define SCHEDBYTE_MOTORFLIMIT			0x0004
#define SCHEDBYTE_BUTTON				0x0008
#define SCHEDBYTE_RF433					0x0010
#define SCHEDBYTE_ADC					0x0020
#if BRAKEBOARD
#define SCHEDBYTE_BRAKETASK				0x0040
#endif
#define SCHEDBYTE_APPSCREENKEYCHANGE	0x0080
#define SCHEDBYTE_TESTSEND			    0x0080
#define SCHEDBYTE_UPDATEPRESSURE		0x0100
#define SCHEDBYTE_RFFSK					0x0200
#define SCHEDBYTE_RFLORA				0x0400
#if REMOTEBOARD
#define SCHEDBYTE_COMMTOBRAKE			0x0800
#endif
#define SCHEDBYTE_ACCELEROMETER			0x1000
#define SCHEDBYTE_COMMSUP				0x2000
#define SCHEDBYTE_DOWNLOAD_DONE			0x4000

extern uint8_t commErrorCount;
extern uint8_t commFailureCount;

	//-----------------------------
	// STATUS OF ALL THE INTERFACES. 
#define TIRERADIO_INIT			0x01
#define TIRERADIO_COMMWORKS		0x02
#define TIRERADIO_RXPACKET		0x04

 typedef struct
 {
	 uint8_t TireRadio;
	 uint8_t ExtRadio;
	 uint8_t EEPROM;
	 uint8_t Accelerometer; 
 }STATUSDATA;
 extern STATUSDATA statusData; 
extern STATUSDATA statusBrake; 

typedef struct
{
	uint8_t VoltageInput;
	uint8_t VoltageSupercap;
	uint8_t ActuatorStatus;
	#define ACTUATORSTATUS_HOMEONFAIL		0x01
	#define ACTUATORSTATUS_EXTENDONFAIL		0x02 
	#define ACTUATORSTATUS_HOMEOFFFAIL		0x04
	#define ACTUATORSTATUS_EXTENDOFFFAIL	0x08
	#define ACTUATORSTATUS_TIPERROR			0x10
	#define ACTUATORSTATUS_BOTHLIMITSACTIVE 0x20
	#define ACTUATORSTATUS_ENCODERFAIL		0x40
	#define ACTUATORSTATUS_EXTENDTRIGGEREDINSETUP 0x80
	
	uint8_t AccelerometerStatus;
	uint8_t BrakeState;
	#define BRAKESTATE_COMMERROR			0x01
	#define BRAKESTATE_BREAKAWAYTIP			0x02
	#define BRAKESTATE_NOTSETUP				0x04
	#define BRAKESTATE_INPUTVOLTAGEBAD      0x08
	#define BRAKESTATE_MANUALBRAKE			0x10
	#define BRAKESTATE_LOWSUPERCAP  		0x20
	#define BRAKESTATE_BREAKAWAYREADY		0x40
	#define BRAKESTATE_ERRORLOADSET			0x80  //01_28 changed
	uint8_t BrakeState2; 
	#define BRAKESTATE_ERRORLOADSET_VALUE	0x01
}BRAKEDATA;
extern BRAKEDATA brakeStatus;




#if BRAKEBOARD
#define BRAKECHANGE_ADCDONE		0x01
#define BRAKECHANGE_SUPTIME		0x02 
#define BRAKECHANGE_TABLESAMPLE 0x04
#define BRAKECHANGE_HOLDTIME	0x08
#define BRAKECHANGE_DITHER		0x10
extern uint8_t brakeChange;  
extern uint8_t poweredUp;
extern uint16_t loadTime;
#endif 


 #define INTERFACE_TIRERADIO  1
	#define STATUS_PARTTALKING		0x01
	#define STATUS_RXPACKET			0x02
	#define STATUS_COMMGOOD			0x04
	
 #define INTERFACE_EXTRADIO   2
 #define INTERFACE_EEPROM	  3
 	#define STATUS_PARTTALKING		0x01
	#define STATUS_RW				0x02
	#define STATUS_GOODHEADER		0x04
	
 #define INTERFACE_ACCELEROMETER 4
  	#define STATUS_PARTTALKING		0x01

	
	//-------------------------------
	//  values used above for each of the interfaces.
	//  if good then that will be set and the corresponding
	//    bit is set 
void AppStatusUpdate(uint8_t whichInterface,uint8_t statusUpdate,uint8_t good); 
 
 

#define CONSTANTTX	PIN_PA00
#define CONSTANTRX	PIN_PA01
#define CONSTANTCW	PIN_PB22

#if REMOTEBOARD
//	extern const uint8_t *FONTSMALL[16]; 
	//---------------------------
	// LCD PINS 
	//---------------------------
	#define RST			PIN_PA27    
	#define A0          PIN_PA24 
	#define RW          PIN_PA15
	#define E           PIN_PA14
	#define CSB			PIN_PA25
	#define DB0         PIN_PA16
	#define DB1         PIN_PA17
	#define DB2			PIN_PA18
	#define DB3			PIN_PA19 
	#define DB4			PIN_PA20
	#define DB5         PIN_PA21
	#define DB6			PIN_PA22
	#define DB7			PIN_PA23
	//---------------------------------
	// LEDs on the board
	// PB13 - BL RED
	// PB14 - BL BLUE
	// PB15 - BL GREEN 
	// PA03 - BL KEY
	//---------------------------------
	#define LED_RED_PIN                  PIN_PB13
	#define LED_RED_ACTIVE               true
	#define LED_RED_INACTIVE             !LED_RED_ACTIVE

	#define LED_BLUE_PIN                  PIN_PB14
	#define LED_BLUE_ACTIVE               true
	#define LED_BLUE_INACTIVE             !LED_BLUE_ACTIVE

	#define LED_GREEN_PIN                PIN_PB15
	#define LED_GREEN_ACTIVE             true
	#define LED_GREEN_INACTIVE           !LED_GREEN_ACTIVE
	
	#define BL_KEY_PIN					PIN_PA03
	#define BL_KEY_ACTIVE				false
	#define BL_KEY_INACTIVE				!BL_KEY_ACTIVE

	//---------------------------------
	// BUTTONs on the board
	// SW1 - SW4 is PB4, PB5, PB6, PB7
	//---------------------------------
	#define BUTTON_SW1 PIN_PB04
	#define BUTTON_SW2 PIN_PB05
	#define BUTTON_SW3 PIN_PB06
	#define BUTTON_SW4 PIN_PB07
#endif 



#if BRAKEBOARD 
extern uint8_t toggle;  	
	//-----------------------
	//motor pins 
	#define ENa	PIN_PA18
	#define ENb PIN_PA24
	#define INa PIN_PA19
	#define INb PIN_PA25
	
	#define BUTTON_SETUP			PIN_PA15
	#define BUTTON_POWER			PIN_PA14
	#define INPUT_BREAKAWAY_TIP		PIN_PA23
	#define INPUT_BREAKAWAY_RING	PIN_PA22
	
	
	#define FLIMIT PIN_PA21
	#define HLIMIT PIN_PA20
	#define ENCODER PIN_PB11
	#define IRLEDEN PIN_PA27
	
	#define SUPERCAPEN	PIN_PB05

	#define LED_BLUE_PIN	PIN_PB12
	#define LED_RED_PIN		PIN_PB13	
	#define LED_BIGREEN_PIN	PIN_PB14
	#define LED_BIRED_PIN		PIN_PB15
	
	#define BLUETOOTH_RESET PIN_PA09
	#define BLUETOOTH_PROG	PIN_PA08
	#define BLUETOOTH_TX	PIN_PA10
	#define BLUETOOTH_RX	PIN_PA11
	

#endif


//-------------------------COOMON ITEMS

	//---------------------
	// speaker settings 
	// PB16, PB17 
	//----------------------
	
	//----------------------
	//RF433 RADIO
	//---------------------- 
	#define TPS_POWERON					PIN_PB30
	#define TPS_IRQ						PIN_PB31 
	//--------------------------------
	//loRa radio
	//---------------------------------
	// X-RESET = output = PA3
	// X-NSSS = output = PA6
	// X-MOSI = output = PA4
	// X-MISO = input = PA7
	// X-SCK = output = PA5
	// X-DIO0 = input ? = PB9
	//---------------------------------
	#define LORA_XRESET                 PIN_PB08
	#define LORA_XNSSS					PIN_PA06
	#define LORA_XMOSI					PIN_PA04
	#define LORA_XMISO					PIN_PA07
	#define LORA_XSCK					PIN_PA05
	#define LORA_XDIO0					PIN_PB09
#if BRAKEBOARD 	
	#define LORA_XDIO1					PIN_PA17
#else
	#define LORA_XDIO1					PIN_PB11
#endif	
	
#define EXT2_SPI_MODULE              SERCOM0
#define EXT2_SPI_SERCOM_MUX_SETTING  SPI_SIGNAL_MUX_SETTING_D
#define EXT2_SPI_SERCOM_PINMUX_PAD0  PINMUX_PA04D_SERCOM0_PAD0
#define EXT2_SPI_SERCOM_PINMUX_PAD1  PINMUX_PA05D_SERCOM0_PAD1
#define EXT2_SPI_SERCOM_PINMUX_PAD2  PINMUX_PA06D_SERCOM0_PAD2
#define EXT2_SPI_SERCOM_PINMUX_PAD3  PINMUX_PA07D_SERCOM0_PAD3	


#define EXT3_SPI_MODULE              SERCOM5
#define EXT3_SPI_SERCOM_MUX_SETTING  SPI_SIGNAL_MUX_SETTING_D
#define EXT3_SPI_SERCOM_PINMUX_PAD0  PINMUX_PB02D_SERCOM5_PAD0
#define EXT3_SPI_SERCOM_PINMUX_PAD1  PINMUX_PB03D_SERCOM5_PAD1
#define EXT3_SPI_SERCOM_PINMUX_PAD2  PINMUX_PB00D_SERCOM5_PAD2
#define EXT3_SPI_SERCOM_PINMUX_PAD3  PINMUX_PB01D_SERCOM5_PAD3

 

/** Number of on-board buttons */
//#define BUTTON_COUNT 1
#if REMOTEBOARD
#define EXT1_SPI_MODULE              SERCOM2
#define EXT1_SPI_SERCOM_MUX_SETTING  SPI_SIGNAL_MUX_SETTING_D
#define EXT1_SPI_SERCOM_PINMUX_PAD0  PINMUX_PA08D_SERCOM2_PAD0
#define EXT1_SPI_SERCOM_PINMUX_PAD1  PINMUX_PA09D_SERCOM2_PAD1
#define EXT1_SPI_SERCOM_PINMUX_PAD2  PINMUX_PA10D_SERCOM2_PAD2
#define EXT1_SPI_SERCOM_PINMUX_PAD3  PINMUX_PA11D_SERCOM2_PAD3
#endif 

#define EXT2_I2C_MODULE              SERCOM4
#define EXT2_I2C_SERCOM_PINMUX_PAD0  PINMUX_PA12D_SERCOM4_PAD0
#define EXT2_I2C_SERCOM_PINMUX_PAD1  PINMUX_PA13D_SERCOM4_PAD1

extern uint8_t eventMessageReceived; 

extern uint16_t brakeHoldOffTime; 
extern uint16_t brakeSupTime; 
extern uint16_t wdog;

extern uint16_t commSupTimer; 
#define COMM_SUP_TIME 500   
#define COMM_SUP_TIME_REMOTE 100

	//----------------FCC TESTING
extern uint8_t setTXContinuous; 
extern uint8_t setCW;
extern uint8_t setRXContinuous; 

#if REMOTEBOARD
void BacklightSet(uint8_t state);
#endif

void CommInit(void);
void AppHandleRadioSwitch(uint8_t which);
uint8_t SwitchRadio(uint8_t *whichWay);
void HardDelay(void);

#endif  /* SAMD20_XPLAINED_PRO_H_INCLUDED */
