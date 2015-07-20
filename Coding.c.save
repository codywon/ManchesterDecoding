/*
Project : Coding.c
Date : 4/22/2009
Author : Toby Prescott
Company : Atmel
Comments:
Chip type : ATmega128
Program type : Application
Clock frequency : 8.000000 MHz
Memory model : Small
External SRAM size : 0
Data Stack size : 1024
Revisions:
v1.0 - Started WINAVR
12
9164A–AUTO–09/09
Manchester Coding Basics
This source code provided via Atmel Corporation ("Atmel"), is a courtesy
to its valued customers and prospective customers and may be used for
limited purposes only.
-------------------------------------------------------------------------
Copyright (c) 2007 Toby Prescott
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in
the documentation and/or other materials provided with the
distribution.
* Neither the name of the copyright holders nor the names of
contributors may be used to endorse or promote products derived
from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
-----------------------------------------------------------------------*/
#include "Coding.h"
volatile unsigned char cDataBuff[BUFFSIZE] = {0}; // Read Data buffer
volatile unsigned char *cDataBuffPtr;
// Runtime values for Reader Timings //
unsigned int clk2T=0;
struct DecodeTiming DecodeReadTime = {0};
volatile unsigned char numSampleBits = 0;
volatile unsigned int RdTime = 0; //Global var used for edgetiming
unsigned char directionFlag = READ;


// **********************************************************************
// Routine to recover clock and timings
// *********************************************************************/
unsigned char Coding_ClkSync(unsigned char numSamples)
{
    unsigned int clkT=0;
    unsigned int tmp=0, average=0, sample=0;
    directionFlag = READ; // Set direction for timer interrupt
    Coding_TimerInit(0x00, TIMING); // Initiate timer w/ edge2edge measurement
    Coding_Timer_Poll(); // Wait for edge
    clkT = Coding_Timer_Poll(); // Set initial measurment as T time
    do
    {
        tmp = Coding_Timer_Poll(); // Catch next edge time
        if(tmp < UPPERTIMINGLMT) // Check if edge time is useable
        {
            if(tmp < (clkT*0.5))
            {
                clkT = tmp;   // Time below limit
            }
            else if((tmp >= (clkT*0.5)) && (tmp <= (clkT*1.5)))
            {
                average += tmp; // Accumulate
                sample++; // Inc sample count
                clkT = (average/sample); // Average
            }
            else if((tmp >= (clkT*1.5)) && (tmp <= (clkT*2.5)))
            {
                average += (tmp/2); // Accumulate but sample/2
                sample++; // Inc sample count
                clkT = (average/sample); // Average
            }
            else
            {
                clk2T = 128; // Force default to 2T = 256us
                break;
            }
        }
        else
        {
            clkT = 128; // Force default to 2T = 256us
                 break;
        }
    }
    while(sample < numSamples);  // Repeat while < sample size

    Coding_Timer_Stop(); // Stop timer
    clk2T = (clkT*2); // Compute 2T time
    DecodeReadTime.ShortL = (int)(clk2T*0.25); // Compute low T limit
    DecodeReadTime.ShortH = (int)(clk2T*0.75); // Compute high T limit
    DecodeReadTime.LongL = (int)(clk2T*0.75); // Compute low 2T limit
    DecodeReadTime.LongH = (int)(clk2T*1.25); // Compute high 2T limit
    if(sample == numSamples)
    {
        return SUCCESS0;
    }
    else
    {
        return TagErr;
    }
}
// **********************************************************************
// Routine to set clock and timings
// *********************************************************************/
void Coding_SetClk(unsigned int clk, unsigned int shortL,
                   unsigned int shortH, unsigned int longL, unsigned int longH)
{
    clk2T = clk; // Force 2T time
    DecodeReadTime.ShortL = shortL; // Force low T limit
    DecodeReadTime.ShortH = shortH; // Force high T limit
    DecodeReadTime.LongL = longL; // Force low 2T limit
    DecodeReadTime.LongH = longH; // Force high 2T limit
}
// **********************************************************************
// Routine to encode a Manchester data stream
// Pass in the number of bits to send
// Pulls from cDataBuff
// *********************************************************************/
unsigned char Coding_ManchesterEncode(unsigned char numBits)
{
    volatile unsigned int cNumBits = 0,i;
    cDataBuffPtr = &cDataBuff[0]; // Place pointer at beginning of
    buffer
    directionFlag = WRITE; // Set direction for timer interrupt
    Coding_TimerInit((clk2T/2), SAMPLING); // Init timer w/ periodic
    interrupt
    for(i=0; i<numBits; i++) // Repeat until all bits sent
    {
        if(cNumBits == 8) // When full byte is read
        {
            cDataBuffPtr++; // Increment pointer to next byte in
            buffer
            if(cDataBuffPtr == &cDataBuff[0])
            {
                i=numBits+1;
            }
            cNumBits = 0; // Clear bit counter
        }


        if((*cDataBuffPtr & 0x80) == 0x80) // Check bit value, process logic
            one
        {
            cbi(IOPORT,DATAOUT); // Set I/O low
            Coding_Timer_Poll(); // Catch next interrupt
            sbi(IOPORT,DATAOUT); // Set I/O high
            Coding_Timer_Poll(); // Catch next interrupt
        }
        else
        {
            sbi(IOPORT,DATAOUT); // Set I/O high
            Coding_Timer_Poll(); // Catch next interrupt
            cbi(IOPORT,DATAOUT); // Set I/O low
            Coding_Timer_Poll(); // Catch next interrupt
        }
        *cDataBuffPtr = *cDataBuffPtr<<1; // Shift buffer to get next bit
        cNumBits++; // Increment number of bits sent
    }
    return 0;
}
// **********************************************************************
// Routine to synchronize to Manchester edge
// *********************************************************************/
unsigned char Coding_ManchesterSync(unsigned char maxSample)
{
    unsigned char i=0;
    unsigned int tmp;
    unsigned char cOutput = SyncErr;
    directionFlag = READ; // Set direction for timer interrupt
    Coding_TimerInit(0x00, TIMING); // Init timer w/ edge-2-edge measurement
    tmp = Coding_Timer_Poll(); // Wait for edge
    while(i++ < maxSample) // Repeat until sample size is meet
    {
        tmp = Coding_Timer_Poll(); // Catch next edge time
        if(tmp > UPPERTIMINGLMT)
        {
            break;   // Check if edge time is usable
        }
        else if((tmp >= DecodeReadTime.LongL) &&
                (tmp <= DecodeReadTime.LongH))
        {
            //2T time found, check starting logic value
            if(tst(IOPIN,DATAIN) == 0)
            {
                cOutput = SUCCESS0;
            }
            else
            {
                cOutput = SUCCESS1;
            }
            break;
        }
    }
    return cOutput;
}
// **********************************************************************
// Routine to decode a Manchester bit
// Pass in the previous bit logic value
// *********************************************************************/
unsigned char Coding_ManchesterDecode(unsigned char cBit)
{
    unsigned char cOutput = BitErr;
    unsigned int tmp;
    tmp = Coding_Timer_Poll(); // Catch next edge time
    if(tmp < UPPERTIMINGLMT) // Check if edge time is usable
    {
        // Check edge time and determine next Logic value //
        if((tmp > DecodeReadTime.LongL) && (tmp < DecodeReadTime.LongH))
        {
            cOutput = cBit ^ 0x01;   // invert cBit for logical change
        }
        else if(tmp > DecodeReadTime.ShortL && tmp < DecodeReadTime.ShortH)
        // Next edge time is short
        {
            tmp = Coding_Timer_Poll();
            if(tmp > DecodeReadTime.ShortL &&
                    tmp < DecodeReadTime.ShortH)
            {
                cOutput = cBit;   // bit stays the same
            }
            else
            {
                cOutput = BitErr;   // Un-paired short time
            }
        }
        else
        {
            cOutput = BitErr;   // Edge time outside limits
        }
    }
    return cOutput;
}
// **********************************************************************
// Routine to decode a BiPhase1 bit
// *********************************************************************/
unsigned char Coding_BiPhase1Decode(void)
{
    unsigned char cOutput = BitErr;
    unsigned int tmp;
    tmp = Coding_Timer_Poll(); // Catch next edge time
    if(tmp < UPPERTIMINGLMT) // Check if edge time is useable
    {
// Check edge time and determine next Logic value //
        if(tmp > DecodeReadTime.LongL &&
                tmp < DecodeReadTime.LongH)

        {
            cOutput = 0;
        }
        else if(tmp > DecodeReadTime.ShortL && tmp < DecodeReadTime.ShortH)
        {
            tmp = Coding_Timer_Poll();
            if(tmp > DecodeReadTime.ShortL && tmp < DecodeReadTime.ShortH)
            {
                cOutput = 1;
            }
            else
            {
                cOutput = BitErr; // Un-paired short time between
            }
        }
        else
        {
            cOutput = BitErr;   // Edge time outside limits
        }
    }
    return cOutput;
}
// **********************************************************************
// Read Routine Using the U2270
// **********************************************************************
void Coding_DSP(unsigned char Encoding)
{
    unsigned char count=0, cLong=0, cShort=0;
    unsigned char i, logicFlag, cNumBit=0, syncFlag=0;
    unsigned char tmpData,j, bitVal=0;
    volatile unsigned char *cDSPBuffPtr = &cDataBuff[0];
    cDataBuffPtr = &cDataBuff[0]; // Place pointer at beginning
    of buffer
    if((*cDSPBuffPtr & 0x80) == 0x80)
    {
        logicFlag = 1;   // Initialize logic
    }
    flag
    else
    {
        logicFlag = 0;
    }
    for(j=0; j<BUFFSIZE; j++) // Process entire buffer
    {
        tmpData = *cDSPBuffPtr++; // Pull out working byte
        for(i=0; i<8; i++) // Process entire byte
        {
            if(!syncFlag)
            {
                if(logicFlag == 1 && (tmpData & 0x80) == 0x80)
                {
                    count++;
                }
                else if(logicFlag == 0 && (tmpData & 0x80) == 0x00)
                {
                    count++;
                }
                else
                {
                    logicFlag = logicFlag^0x01; // Current flag inverted
                    if(count > 4)

                    {
                        syncFlag=1; // 2T sync found
                        bitVal = logicFlag;
                    }
                    count=1; // Reset count
                }
            }
            else
            {
                if(logicFlag == 1 && (tmpData & 0x80) == 0x80)
                {
                    count++;
                }
                else if(logicFlag == 0 && (tmpData & 0x80) == 0x00)
                {
                    count++;
                }
                else
                {
// Check if count below threshold, inc short
                    if(count <=4)
                    {
                        cShort++;
                    }
                    else
                    {
                        cLong++;   // else inc long
                    }
                    count=1; // Reset count
                    logicFlag = logicFlag^0x01; // Current flag inverted
                    if(cLong == 1)
                    {
                        cLong = 0;
                        if(Encoding == MANCHESTER) // Decode Manchester
                        {
                            bitVal = bitVal^0x01;
                        }
                        else if(Encoding == BIPHASE1) // Decode BiPhase
                        {
                            bitVal = 0;
                        }
                        if(bitVal == 1)
                        {
                            *cDataBuffPtr = *cDataBuffPtr << 1;
                            *cDataBuffPtr = *cDataBuffPtr | 0x01;
                        }
                        else if(bitVal == 0)
                        {
                            *cDataBuffPtr = *cDataBuffPtr << 1;
                        }
                        cNumBit++;
                    }
                    else if(cShort == 2)
                    {
                        cShort = 0;
                        if(Encoding == MANCHESTER)
                        {
                            ;
                        }
                        else if(Encoding == BIPHASE1)
                        {
                            bitVal = 1;
                        }
                        if(bitVal == 1)

                        {
                            *cDataBuffPtr = *cDataBuffPtr << 1;
                            *cDataBuffPtr = *cDataBuffPtr | 0x01;
                        }
                        else if(bitVal == 0)
                        {
                            *cDataBuffPtr = *cDataBuffPtr << 1;
                        }
                        cNumBit++;
                    }
                    if(cNumBit == 8) // When full byte is read
                    {
                        cDataBuffPtr++; // Inc ptr to next byte
                        cNumBit = 0; // Clear bit counter
                    }
                }
            }
            tmpData = tmpData << 1; // Shift working byte to next
            bit
        }
    }
}
// **********************************************************************
// Read Routine Using
// Pass in the number of Tag Type, data encoding, and type of synch.
// Pass in the number of bits being sent and the data buffer
// *********************************************************************/
unsigned char Coding_ReadData(unsigned char mode, unsigned int numBits,
                              unsigned char startBit, unsigned char Encoding)
{
    unsigned char cBit = 2;
    volatile unsigned char cError = SUCCESS0;
    unsigned char cQuit = 0;
    unsigned int cNumBits = 0, cNumBytes = 0,i;
    cBit = startBit;
    if(mode == SAMPLING)
    {
        directionFlag = READ; // Set direction for timer
        interrupt
        cDataBuff[BUFFSIZE-1] = 0x00; // Clear buffer end byte
        Coding_TimerInit((clk2T/6), mode); // Init timer w/ periodic
        interrupt
        do

        {
        }
        while(cDataBuff[BUFFSIZE-1] == 0x00);// Buffer end byte accessed
        Coding_Timer_Stop(); // Stop Timer
        Coding_DSP(Encoding); // Run DSP processing on samples.
    }
    else
    {
        cBit = Coding_ManchesterSync(100);
        if(cBit == SUCCESS0 || cBit == SUCCESS1)
        {
            while(!cQuit)
            {
                for(i=0; i<(numBits*2)+10; i++)
                {
                    if(cNumBits == 8) // When full byte is read
                    {
                        cDataBuffPtr++; // Increment pointer to next
                        byte in buffer
                        cNumBits = 0; // Clear bit counter
                        cNumBytes++; // Increment byte counter
                    }
                    if(Encoding == MANCHESTER) // Decode the next bit
                        (Manchester)
                    {
                        cBit = Coding_ManchesterDecode(cBit);
                    }
                    else if(Encoding == BIPHASE1) // Decode the next bit
                        (BiPhase)
                    {
                        cBit = Coding_BiPhase1Decode();
                    }
                    if(cBit == 1)
                    {
                        *cDataBuffPtr = *cDataBuffPtr << 1;
                        *cDataBuffPtr = *cDataBuffPtr | 0x01;
                    }
                    else if(cBit == 0)
                    {
                        *cDataBuffPtr = *cDataBuffPtr << 1;
                    }
                    else
                    {
                        break;
                    }
                    cNumBits++; // Increment number of
                    bits read
                }
                cQuit = 1;
            }
            if((cNumBits+(8*cNumBytes)) == (numBits*2)+10)
            {
                cError = 0;
            }
            else
            {
                cError = BitErr;
            }

        }
        else
        {
            cError = SyncErr;
        }
        Coding_Timer_Stop();
    }
    if(cError != 0)
    {
        for(i=0; i < BUFFSIZE; i++)
        {
            cDataBuff[i]=0x00;   //Reset
        }
    }
    Buffer
    return cError; // Return error code (zero = successfull)
}
// **********************************************************************
// * RFIDTimer Initialization of for Read Routine
// **********************************************************************
void Coding_TimerInit(unsigned int countVal, unsigned char mode)
{
    CODINGTIMERMSK = 0x00; //Disable TC1 interrupts
    cDataBuffPtr = &cDataBuff[0]; // Place pointer at beginning of buffer
    OCR1A = countVal;
    CODINGTIMERCNT = 0x00; //Load TC1 count value
    if(mode == TIMING)
    {
        sbi(CODINGTIMERMSK,ICIE1); // Timer1 Input Capture & Interrupt Enable
        sbi(CODINGTIMERMSK,TOIE1);
    }
    else
    {
        sbi(CODINGTIMERMSK,OCIE1A);   // Timer/Counter1 Output Compare A
    }

    CODINGTIMERFLR |= 0x27; //clear interrupt flags for TC1  0010 0111
    CODINGTIMERCTRA = 0x00; //Normal mode
    cbi(CODINGTIMERCTRB,ICES1); //Look for Falling Edge on ICP1
    CODINGTIMERCTRB |= (1<<CS11); //prescale = clock source / 8 //exactly 1 us for every timer step
    CODINGTIMERCTRC = 0x00; //Normal mode
}

// **********************************************************************
// * Shutdown RFIDTimer
// **********************************************************************
void Coding_Timer_Stop(void)
{
    CODINGTIMERMSK &= ~0x27; //Disable TC1 interrupts
    CODINGTIMERCTRA = 0x00;
    CODINGTIMERCTRB = 0x00; //No clock source / Timer Stopped
    CODINGTIMERCTRC = 0x00;
    CODINGTIMERFLR |= 0x27; //clear interrupt flags for TC1
}
// **********************************************************************
// * Read Edge Time
// **********************************************************************
unsigned int Coding_Timer_Poll(void)
{
    asm("sei"); // Turn on global Interrupts
    RdTime = 0; // Clear timing measurement
    while(RdTime == 0) {} // Wait for interrupt to generate measurement
    return RdTime;
}
// **********************************************************************
// * RFIDTimer Output Compare Interrupt Routine
// **********************************************************************
ISR(CODINGTIMER_CMPA)
{
    CODINGTIMERCNT = 0x0000; //Reset TC1 Count value
    RdTime = 1; //Set Read Time = 1 (Timer Match)

    if(directionFlag == READ) //Only process level sampling on Read
{
    if(numSampleBits == 8) //Complete byte
        {
            numSampleBits = 0; //Reset bit counter
            cDataBuffPtr++; //Inc buffer ptr
        }
        *cDataBuffPtr = *cDataBuffPtr<<1; //Shift in new bit
        if(tst(IOPIN,DATAIN) == 1) //Check logic level
        {
            *cDataBuffPtr = *cDataBuffPtr|0x01; //Store one
        }
        numSampleBits++; //Inc bit count
    }
}
//
*************************************************************************
// * RFIDTimer Overflow Interrupt Routine
// **********************************************************************
ISR(CODINGTIMER_OVF)
{
    CODINGTIMERCNT = 0x0000; //Reset TC1 Count value
    RdTime = 0xFFFF; //Set Read Time = 0xFFFF (overflow)
}
// **********************************************************************
// * RFIDTimer Input Capture Interrupt Routine
// **********************************************************************
ISR(CODINGTIMER_IPC)
{
    CODINGTIMERCNT = 0x0000; //Reset TC1 Count value
    tgl(CODINGTIMERCTRB,ICES1); //change edge on ICP1 (XOR)
    RdTime = ICR1;
}
