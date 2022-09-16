#include <XC.h>
#include <sys/attribs.h>
#include <stdio.h>
#include <stdlib.h>
 
// Configuration Bits (somehow XC32 takes care of this)
#pragma config FNOSC = FRCPLL       // Internal Fast RC oscillator (8 MHz) w/ PLL
#pragma config FPLLIDIV = DIV_2     // Divide FRC before PLL (now 4 MHz)
#pragma config FPLLMUL = MUL_20     // PLL Multiply (now 80 MHz)
#pragma config FPLLODIV = DIV_2     // Divide After PLL (now 40 MHz) 
#pragma config FWDTEN = OFF         // Watchdog Timer Disabled
#pragma config FPBDIV = DIV_1       // PBCLK = SYCLK
#pragma config FSOSCEN = OFF        // Turn off secondary oscillator on A4 and B4

// RB0+RB1 for left wheel
// RB2+RB3 for right wheel
#define SYSCLK 40000000L
#define FREQ 100000L // We need the ISR for timer 1 every 10 us
#define Baud2BRG(desired_baud)( (SYSCLK / (16*desired_baud))-1)
void moveforward();
void movebackward();
void wait_1ms(void);
void waitms(int len);
void main (void)
{
	volatile unsigned long t=0;
	
	DDPCON = 0;
	
	TRISBbits.TRISB0 = 0;
	LATBbits.LATB0 = 0;	
	INTCONbits.MVEC = 1;
	TRISBbits.TRISB1 = 0;
	LATBbits.LATB1 = 0;
	TRISBbits.TRISB2 = 0;
	LATBbits.LATB2 = 0;
	TRISBbits.TRISB3 = 0;
	LATBbits.LATB3 = 0;

	right_turn();
	TRISBbits.TRISB6 = 0;
	LATBbits.LATB6 = 0;	
	right_turn();
	while (1)
	{
		t++;
		if(t==500000)
		{
			t = 0;
			LATBbits.LATB6 = !LATBbits.LATB6; // Blink led on RB6
		}
	}
	
}

void movebackward(){
	
	LATBbits.LATB0 = 1;
	LATBbits.LATB1 = 0;
	LATBbits.LATB2 = 1;
	LATBbits.LATB3 = 0;

}

void moveforward(){
	
	LATBbits.LATB0 = 0;
	LATBbits.LATB1 = 1;
	LATBbits.LATB2 = 0;
	LATBbits.LATB3 = 1;

}
void left_turn(){
	LATBbits.LATB0 = 0;
	LATBbits.LATB1 = 0;
	LATBbits.LATB2 = 0;
	LATBbits.LATB3 = 1;


}
void right_turn(){
	LATBbits.LATB0 = 0;
	LATBbits.LATB1 = 1;
	LATBbits.LATB2 = 0;
	LATBbits.LATB3 = 0;


}

void stop_wheels(){
	LATBbits.LATB0 = 0;
	LATBbits.LATB1 = 0;
	LATBbits.LATB2 = 0;
	LATBbits.LATB3 = 0;


}
void wait_1ms(void)
{
	unsigned int ui;
	_CP0_SET_COUNT(0); // resets the core timer count

	// get the core timer count
	while ( _CP0_GET_COUNT() < (SYSCLK/(2*1000)) );
}

void waitms(int len)
{
	while(len--) wait_1ms();
}
