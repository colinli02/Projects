//March 2022

#include <stdio.h>
#include <at89lp51rd2.h>

// ~C51~ 
 
#define CLK 22118400L
#define BAUD 115200L
#define ONE_USEC (CLK/1000000L) // Timer reload for one microsecond delay
#define BRG_VAL (0x100-(CLK/(16L*BAUD)))

#define ADC_CE  P2_0
#define BB_MOSI P2_1
#define BB_MISO P2_2
#define BB_SCLK P2_3



//LCD Headers
#define CLK    22118400L // SYSCLK frequency in Hz
#define BAUD     115200L // Baud rate of UART in bps
#define ONE_USEC (CLK/1000000L) // Timer reload for one microsecond delay

#if (CLK/(16L*BAUD))>0x100
#error Can not set baudrate
#endif
#define BRG_VAL (0x100-(CLK/(16L*BAUD)))

//LCD Pins
#define LCD_RS P3_2
//#define LCD_RW PX_X // Not used in this code.  Connect pin to GND
#define LCD_E  P3_3
#define LCD_D4 P3_4
#define LCD_D5 P3_5
#define LCD_D6 P3_6
#define LCD_D7 P3_7
#define CHARS_PER_LINE 16

//======================================
//
//              Headers
//
//======================================
void LCD_pulse (void);
void LCD_byte (unsigned char x);
void WriteData (unsigned char x);
void WriteCommand (unsigned char x);
void LCD_4BIT (void);
void LCDprint(char * string, unsigned char line, bit clear);
  
float getHalfPeriod(void);
float PhaseDiffer(void);


//======================================
//
//             Timer Functions
//
//======================================
unsigned char SPIWrite(unsigned char out_byte)
{
	// In the 8051 architecture both ACC and B are bit addressable!
	ACC=out_byte;
	
	BB_MOSI=ACC_7; BB_SCLK=1; B_7=BB_MISO; BB_SCLK=0;
	BB_MOSI=ACC_6; BB_SCLK=1; B_6=BB_MISO; BB_SCLK=0;
	BB_MOSI=ACC_5; BB_SCLK=1; B_5=BB_MISO; BB_SCLK=0;
	BB_MOSI=ACC_4; BB_SCLK=1; B_4=BB_MISO; BB_SCLK=0;
	BB_MOSI=ACC_3; BB_SCLK=1; B_3=BB_MISO; BB_SCLK=0;
	BB_MOSI=ACC_2; BB_SCLK=1; B_2=BB_MISO; BB_SCLK=0;
	BB_MOSI=ACC_1; BB_SCLK=1; B_1=BB_MISO; BB_SCLK=0;
	BB_MOSI=ACC_0; BB_SCLK=1; B_0=BB_MISO; BB_SCLK=0;
	
	return B;
}

unsigned char _c51_external_startup(void)
{
	AUXR=0B_0001_0001; // 1152 bytes of internal XDATA, P4.4 is a general purpose I/O

	P0M0=0x00; P0M1=0x00;    
	P1M0=0x00; P1M1=0x00;    
	P2M0=0x00; P2M1=0x00;    
	P3M0=0x00; P3M1=0x00;
	
	// Initialize the pins used for SPI
	ADC_CE=0;  // Disable SPI access to MCP3008
	BB_SCLK=0; // Resting state of SPI clock is '0'
	BB_MISO=1; // Write '1' to MISO before using as input
	
	// Configure the serial port and baud rate
    PCON|=0x80;
	SCON = 0x52;
    BDRCON=0;
    #if (CLK/(16L*BAUD))>0x100
    #error Can not set baudrate
    #endif
    BRL=BRG_VAL;
    BDRCON=BRR|TBCK|RBCK|SPD;
    
	CLKREG=0x00; // TPS=0000B

    return 0;
}

void wait_us (unsigned char x)
{
	unsigned int j;
	
	TR0=0; // Stop timer 0
	TMOD&=0xf0; // Clear the configuration bits for timer 0
	TMOD|=0x01; // Mode 1: 16-bit timer
	
	if(x>5) x-=5; // Subtract the overhead
	else x=1;
	
	j=-ONE_USEC*x;
	TF0=0;
	TH0=j/0x100;
	TL0=j%0x100;
	TR0=1; // Start timer 0
	while(TF0==0); //Wait for overflow
}

void waitms (unsigned int ms)
{
	unsigned int j;
	unsigned char k;
	for(j=0; j<ms; j++)
		for (k=0; k<4; k++) wait_us(250);
}

/*Read 10 bits from the MCP3008 ADC converter*/
unsigned int volatile GetADC(unsigned char channel)
{
	unsigned int adc;
	unsigned char spid;

	ADC_CE=0; // Activate the MCP3008 ADC.
	
	SPIWrite(0x01);// Send the start bit.
	spid=SPIWrite((channel*0x10)|0x80);	//Send single/diff* bit, D2, D1, and D0 bits.
	adc=((spid & 0x03)*0x100);// spid has the two most significant bits of the result.
	spid=SPIWrite(0x00);// It doesn't matter what we send now.
	adc+=spid;// spid contains the low part of the result. 
	
	ADC_CE=1; // Deactivate the MCP3008 ADC.
		
	return adc;
}

//Gets half period of P1_0
float getHalfPeriod() 
{
  float Period = 0;
  float myof;
  
  TR0=0;
  TMOD&=0B_1111_0000;
  TMOD |=0B_0000_0001;
  TH0=0;
  TL0=0;
  myof=0;
  TF0=0;
  while(P1_0==1);
  while(P1_0==0);
  TR0=1;
  while(P1_0==1)// while(GetADC(P2_4)==1)
  {  
    if(TF0) {TF0=0; myof++;}
  }
  TR0=0;
  
  Period=(myof*65536.0+TH0*256.0+TL0)*2.0;
  
  //Period = Period* 0.59665871121; //debug scaling 838->500
  
  return Period;
}

//Gets Half period if pin 1_1
float getHalfPeriodOther() 
{
  float Period = 0;
  float myof;
  
  TR0=0;
  TMOD&=0B_1111_0000;
  TMOD |=0B_0000_0001;
  TH0=0;
  TL0=0;
  myof=0;
  TF0=0;
  while(P1_1==1);
  while(P1_1==0);
  TR0=1;
  while(P1_1==1)// while(GetADC(P2_4)==1)
  {  
    if(TF0) {TF0=0; myof++;}
  }
  TR0=0;
  
  Period=(myof*65536.0+TH0*256.0+TL0)*2.0;
  
  //Period = Period* 0.59665871121; //debug scaling 838->500
  
  return Period;
}
//Gets phase difference 
float PhaseDiffer()
{
	float myof;
	float PhaseDiff = 0;
		
	TR0=0;
	TMOD&=0B_1111_0000;
	TMOD |=0B_0000_0001;
	TH0=0;
	TL0=0;
	myof=0;
	TF0=0;
	
	while (P1_0==1);
	while (P1_0==0);
		
	if (P1_0==0) 
	{
	    TR0 = 1;
	    while (P1_0==0) 
	    {
	        if (TF0) 
	        {
	            TF0 = 0;
	            myof++;
	        }
	    }
	    TR0 = 0;
	    return PhaseDiff =(myof*65536.0+TH0*256.0+TL0)*0.00005085596*2.0;
	} 
	else 
	{
	    TR0 = 1;
	    while (P1_0==1) 
	    {
	        if (TF0) 
	        {
	            TF0 = 0;
	            myof++;
	        }
	    }
	    while (P1_0==0) 
	    {
	        if (TF0) 
	        {
	            TF0 = 0;
	            myof++;
	        }
	    }
	    TR0 = 0;
	    return PhaseDiff=(myof*65536.0+TH0*256.0+TL0)*0.00005085596*2.0;
	}


}

//======================================
//
//              LCD Code
//
//======================================
  
void LCD_pulse (void)
{
	LCD_E=1;
	wait_us(40);
	LCD_E=0;
}

void LCD_byte (unsigned char x)
{
	// The accumulator in the 8051 is bit addressable!
	ACC=x; //Send high nible
	LCD_D7=ACC_7;
	LCD_D6=ACC_6;
	LCD_D5=ACC_5;
	LCD_D4=ACC_4;
	LCD_pulse();
	wait_us(40);
	ACC=x; //Send low nible
	LCD_D7=ACC_3;
	LCD_D6=ACC_2;
	LCD_D5=ACC_1;
	LCD_D4=ACC_0;
	LCD_pulse();
}

void WriteData (unsigned char x)
{
	LCD_RS=1;
	LCD_byte(x);
	waitms(2);
}

void WriteCommand (unsigned char x)
{
	LCD_RS=0;
	LCD_byte(x);
	waitms(5);
}

void LCD_4BIT (void) //initializes lcd 
{
	LCD_E=0; // Resting state of LCD's enable is zero
	//LCD_RW=0; // We are only writing to the LCD in this program.  Connect pin to GND.
	waitms(20);
	// First make sure the LCD is in 8-bit mode and then change to 4-bit mode
	WriteCommand(0x33);
	WriteCommand(0x33);
	WriteCommand(0x32); // Change to 4-bit mode

	// Configure the LCD
	WriteCommand(0x28);
	WriteCommand(0x0c);
	WriteCommand(0x01); // Clear screen command (takes some time)
	waitms(20); // Wait for clear screen command to finsih.
}

void LCD_print(char * string, unsigned char line, bit clear)
{
	int j;

	WriteCommand(line==2?0xc0:0x80);
	waitms(5);
	for(j=0; string[j]!=0; j++)	WriteData(string[j]);// Write the message
	if(clear) for(; j<CHARS_PER_LINE; j++) WriteData(' '); // Clear the rest of the line
}


  
  
//======================================
//
//              Main
//
//======================================
#define VREF 4.096

//Button Pins
#define BUTTON1 P2_6
#define BUTTON2 P2_7
#define BUTTON3 P2_4

void main (void)
{
	char strBuffer[30]; //strbuffer empty for purposes of LCD displaying
	float HalfPeriod;
	float HalfPeriod2;

	int V516 = 0; //test if output is 420 is wrong
	int V200 = 0;
	int V1 = 0;
	int V2 = 0;
	float PhaseDiff = 0;
	
	//unsigned char i;
	//float halfPeriod; 

	waitms(500);	
	printf("\n\nAT89LP51Rx2 SPI ADC test program.\n");
	
	LCD_4BIT(); //lcd initialization 
	

//always running
while(1)
{
//=========================================================
// Putty terminal display (ctrl + T in cross IDE For putty
//=========================================================

//Gets Half Period	
	HalfPeriod = getHalfPeriod();
	HalfPeriod2 = getHalfPeriodOther();
	HalfPeriod = HalfPeriod * 2.24945563/1000/1000/100; //converts to 0.01, 2.24945563E-8 multiplier
	HalfPeriod2 = HalfPeriod2 * 2.24945563/1000/1000/100; //converts to 0.01, 2.24945563E-8 multiplier
    printf("\n Half Period 1 = %10.5f", HalfPeriod);
    printf("\n Half Period 2 = %10.5f", HalfPeriod2);
   
    
//Volt RMS
	while (P1_0 == 1);
	while (P1_0 == 0 );
	waitms(HalfPeriod * 1000); // 0.01/444552.0 //waits the halfperiod 	
	V516 = GetADC(0); //Will have to double check the math
	V516 = V516*0.03968253968/4.6; //5/126   factoring in Vdd
	printf("\n Voltage 1 = %i", V516);
	
	while (P1_1 == 1);
	while (P1_1 == 0 );
	waitms(HalfPeriod2 * 1000);
	V200 = GetADC(1);
	V200 = V200*0.03968253968/4.6;
	printf("\n Voltage 2 = %i", V200);
	    
		//for(i=0; i<8; i++)
		//{
		//y=(GetADC(i)*VREF)/1023.0; // Convert the 10-bit integer from the ADC to voltage
		//printf("V%d=%5.3f ", i, y);
		//}
	
	
//Phase Difference
PhaseDiff = PhaseDiffer();
printf("\n PhaseDiff = %f", PhaseDiff);

printf("\r"); // Carriage return only.
//=========================================================
// :LCD Display on Board
//=========================================================
	//Display, using an LCD, the magnitude of both inputs in volts RMS, and the phase
	//difference between the reference and test signals in degrees, taking care of displaying the
	//correct sign. Optionally, you can also display the frequency of the reference signal in Hz. 
    
//prints to LCD Screen
	
	//Prints "Press Button To Start" upon start 
	//if(BUTTON1 || BUTTON2 ||BUTTON3 == 0)
	//{
		//LCD_print("Press Button", 1, 1);	
		//LCD_print("To Start", 2, 1);
		
	//}
	//upon button press, prints period (half period * 2)
		if(BUTTON1==0)
		{
		LCD_print("HALF PERIOD.:", 1, 1);
		sprintf(strBuffer, "%.5f %.5f", HalfPeriod, HalfPeriod2);   //prints frequency (var y)
		LCD_print(strBuffer, 2, 1);
		}
	//prints inputs in volts RMS
		else if(BUTTON2==0)
		{
		LCD_print("VOLT RMS. :", 1, 1);
		//Will only update when number is non-zero
		if(V516 > 0)
		V1 = V516;
		if(V200 > 0)
		V2 = V200;
		sprintf(strBuffer, "%i, %i", V1, V2); 
		LCD_print(strBuffer, 2, 1);
		}
	//prints phase differnce between refernce and test signals in degrees
		else if(BUTTON3==0)
		{
		LCD_print("Phase Diff.:", 1, 1);
		sprintf(strBuffer, "%f", PhaseDiff); 
		LCD_print(strBuffer, 2, 1);
		}


		
	} //end while(1)
} //end main
