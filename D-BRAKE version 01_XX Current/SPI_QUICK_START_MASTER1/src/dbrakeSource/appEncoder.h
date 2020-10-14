//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//                FILE: APPENCODER.H
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
#ifndef __APPENCODER_H__
#define __APPENCODER_H__

 
#if BRAKEBOARD
 
#define ENCODER_EXTEND_STARTCOUNT	175
#define ENCODER_RETRACT_ACTIVECOUNT 75
 
#define NONE 0
#define EXTENDING 1
#define RETRACTING 2
#define RETRACTING_BY_ENCODER	3
#define EXTENDING_BY_ENCODER	4
 
//---------------------GLOBAL VARIABLES--------------------------
#define MAX_BUILDTABLE 1024
typedef struct
{
	uint16_t Current[MAX_BUILDTABLE];
	uint16_t EncoderCount[MAX_BUILDTABLE];
}BUILDTABLE;
extern BUILDTABLE buildTable;
extern BUILDTABLE encoderTable;
extern uint16_t buildTableOffset;
extern uint16_t encoderTableOffset;

extern uint16_t encoderCount;
extern uint8_t encoderFlip; 
extern uint16_t encoderFillOffset;
 
//---------------------GLOBAL PROTOTYPES--------------------------
uint16_t MotorFindEncoderMatch(uint16_t matchCurrent);
void ConfigureEncoder(void);
void ConfigureEncoderCallbacks(void);
void EncoderCallback(void); 
 
#endif
#endif
