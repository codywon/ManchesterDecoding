/*
Project : Coding.h
Date : 4/22/2009
Author : Toby Prescott
Company : Atmel
Comments:
/*---------------------------------------------------------------------*/
#ifndef CODING_H__
#define CODING_H__
// List your includes
#include <avr/io.h>
#include <avr/interrupt.h>
#include "Configuration.h"
// Declare your global function prototypes here
unsigned char Coding_ClkSync(unsigned char numSamples);
void Coding_SetClk(unsigned int clk, unsigned int shortL,
                   unsigned int shortH, unsigned int longL, unsigned int longH);
unsigned char Coding_ManchesterSync(unsigned char maxSample);
unsigned char Coding_ManchesterEncode(unsigned char numBits);
unsigned char Coding_ManchesterDecode(unsigned char cBit);
unsigned char Coding_BiPhase1Decode(void);
unsigned char Coding_BiPhase2Decode(void);
void Coding_DSP(unsigned char Encoding);
unsigned char Coding_ReadData(unsigned char mode, unsigned int numBits,
                              unsigned char startBit, unsigned char Encoding);

void Coding_TimerInit(unsigned int countVal, unsigned char mode);
void Coding_Timer_Stop(void);
unsigned int Coding_Timer_Poll(void);
// Declare your global sturctures here
struct DecodeTiming
{
    unsigned int ShortL;
    unsigned int ShortH;
    unsigned int LongL;
    unsigned int LongH;
};
// Declare your global definitions here
#define BUFFSIZE 128
#define UPPERTIMINGLMT 5000
#define SAMPLING 0
#define TIMING 1
#define MANCHESTER 0
#define BIPHASE1 1
#define BIPHASE2 2
#define INVERTED
//#define NONINVERTED
// Error codes
#define SyncErr 2
#define BitErr 3
#define TagErr 4
// Declare your global variables (extern) here
extern struct DecodeTiming DecodeReadTime;
extern volatile unsigned char cDataBuff[BUFFSIZE];
