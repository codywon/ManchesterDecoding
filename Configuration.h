/*
Project : Configuration.h
Date :
Author : Toby Prescott
Company : Atmel
Comments:
Chip type : ATmega128
Program type : Application
Clock frecquency : 8.000000 MHz
Memory model : Small
External SRAM size : 0
Data Stack size : 1024
Revisions:
v1.0 - Started WINAVR
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
9
9164A–AUTO–09/09
Manchester Coding Basics
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
-------------------------------------------------------------------------
*/
// List your includes
#include <avr/io.h>
//-----Hardware specific setup -----//
#define IOPORT PORTD
#define IOPIN PIND
#define IODDR DDRD
#define DATAIN PD4
#define DATAOUT PD6
#define DEBUGPORT PORTF
#define DEBUGDDR DDRF
#define DEBUGPIN PF1
#define CODINGTIMERCNT TCNT1
#define CODINGTIMERCTRA TCCR1A
#define CODINGTIMERCTRB TCCR1B
#define CODINGTIMERCTRC TCCR1C
#define CODINGTIMERMSK TIMSK
#define CODINGTIMERFLR TIFR
#define CODINGTIMER_OVF TIMER1_OVF_vect
#define CODINGTIMER_IPC TIMER1_CAPT_vect
#define CODINGTIMER_CMPA TIMER1_COMPA_vect
//---------------------------------//

//------ Define Macros
#define sbi(port,bit) (port |= (1<<bit)) // Set bit in port
#define cbi(port,bit) (port &= ~(1<<bit)) // Clear bit in port
#define tgl(port,bit) (port ^= (1<<bit)) // Toggle bit in port
#define tst(port,bit) (((port)&(1<<(bit)))>>(bit))// Test bit in port
#define DEBUG(state) if(state == CLEAR){cbi(DEBUGPORT,DEBUGPIN);}
else
{
    sbi(DEBUGPORT,DEBUGPIN);
}
#define TGLDEBUG() (tgl(DEBUGPORT,DEBUGPIN))
#define CLEAR 0
#define SET 1
#define WRITE 0
#define READ 1
// Error codes
#define SUCCESS0 0
#define SUCCESS1 1
