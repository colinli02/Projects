# Micro-controller Projects

## Robotic Coin Picker (March 2022 - April 2022)

- PIC32 micro-controller controlled and battery powered robot coin picker
- Powered by C software for robot wheel control, magnetic arm movement, and boundary perimeter detection of AC perimeter
- Debugged and optimised software and hardware issues

<div class="video-wrapper">
  <iframe width="1280" height="720" src="https://www.youtube.com/embed/euFSMJPZnGc" frameborder="0" allowfullscreen></iframe>
</div>

!!! quote "Links"
    [Github Code Link](https://github.com/colinli02/Projects/blob/main/Project_Repos/Robot_Base_Final/Robot_Base.c)

    [Raw Code Link](https://raw.githubusercontent.com/colinli02/Projects/main/Project_Repos/Robot_Base_Final/Robot_Base.c)

    [Wheel Code Separate](https://github.com/colinli02/Projects/blob/main/Project_Repos/Robot_Base_Final/Test_Codes/wheel.c)

    [Youtube Link](https://youtu.be/euFSMJPZnGc)

??? "Click to show code"

    ```c title="Robot_Base.c"
    // Robot_Base.c for PIC32 
    // Pin Assignments (PIC32)
    // RB0 RB1 - Wheels 
    // RA2 RA3 - wheels
    // RB2 RB3 - ADC 
    // RA3 RB4 - ?? unknown 
    // RB5 - getPeriod -> get frequency for coin detector 
    // RB13 - ElectroMagnet 
    // RB15 - Servo Base, RB14 Servo Arm 


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

    // Defines
    #define SYSCLK 40000000L
    #define FREQ 100000L // We need the ISR for timer 1 every 10 us#include <XC.h>
    #define Baud2BRG(desired_baud)( (SYSCLK / (16*desired_baud))-1)	

    //For the coin detector
    #define MINFREQ 90000 //FOR TESTING, not used. Uses dynamic freq init
    #define MAXFREQ 1000000

    //for perimeter detector
    #define VOLTRANGEMIN_PERIMETER 0.10
    #define VOLTRANGEMAX_PERIMETER 100

    #define MAXCOINS 20

    volatile int ISR_pw = 240, ISR_base = 60,ISR_cnt = 0, ISR_frc;

    // The Interrupt Service Routine for timer 1 is used to generate one or more standard
    // hobby servo signals.  The servo signal has a fixed period of 20ms and a pulse width
    // between 0.6ms and 2.4ms.
    
        // Use the core timer to wait for 1 ms.
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


    void __ISR(_TIMER_1_VECTOR, IPL5SOFT) Timer1_Handler(void)
    {
        IFS0CLR = _IFS0_T1IF_MASK; // Clear timer 1 interrupt flag, bit 4 of IFS0
        
        //waitms(5);
        ISR_cnt++;
        
        int flag = 0;
        if (ISR_cnt < ISR_pw)
        {
        LATBbits.LATB14 = 1;
        }
        else
        {
        LATBbits.LATB14 = 0;
        }

        if (ISR_cnt < ISR_base)
        {
        LATBbits.LATB15 = 1;
        }
        else
        {
        LATBbits.LATB15 = 0;
        }



        if (ISR_cnt >= 2000)
        {
        ISR_cnt = 0; // 2000 * 10us=20ms
        ISR_frc++;
        }
    }

    void SetupTimer1 (void)
    {
        // Explanation here: https://www.youtube.com/watch?v=bu6TTZHnMPY
        __builtin_disable_interrupts();
        PR1 =(SYSCLK/FREQ)-1; // since SYSCLK/FREQ = PS*(PR1+1)
        TMR1 = 0;
        T1CONbits.TCKPS = 0; // 3=1:256 prescale value, 2=1:64 prescale value, 1=1:8 prescale value, 0=1:1 prescale value
        T1CONbits.TCS = 0; // Clock source
        T1CONbits.ON = 1;
        IPC1bits.T1IP = 5;
        IPC1bits.T1IS = 0;
        IFS0bits.T1IF = 0;
        IEC0bits.T1IE = 1;

        INTCONbits.MVEC = 1; //Int multi-vector
        __builtin_enable_interrupts();
    }



    
    #define PIN_PERIOD (PORTB&(1<<5))

    // GetPeriod() seems to work fine for frequencies between 200Hz and 700kHz.
    long int GetPeriod (int n)
    {
        int i;
        unsigned int saved_TCNT1a, saved_TCNT1b;

        _CP0_SET_COUNT(0); // resets the core timer count
        while (PIN_PERIOD!=0) // Wait for square wave to be 0
        {
        if(_CP0_GET_COUNT() > (SYSCLK/4)) return 0;
        }

        _CP0_SET_COUNT(0); // resets the core timer count
        while (PIN_PERIOD==0) // Wait for square wave to be 1
        {
        if(_CP0_GET_COUNT() > (SYSCLK/4)) return 0;
        }

        _CP0_SET_COUNT(0); // resets the core timer count
        for(i=0; i<n; i++) // Measure the time of 'n' periods
        {
        while (PIN_PERIOD!=0) // Wait for square wave to be 0
        {
            if(_CP0_GET_COUNT() > (SYSCLK/4)) return 0;
        }
        while (PIN_PERIOD==0) // Wait for square wave to be 1
        {
            if(_CP0_GET_COUNT() > (SYSCLK/4)) return 0;
        }
        }

        return  _CP0_GET_COUNT();
    }

    void UART2Configure(int baud_rate)
    {
        // Peripheral Pin Select
        U2RXRbits.U2RXR = 4;    //SET RX to RB8
        RPB9Rbits.RPB9R = 2;    //SET RB9 to TX

        U2MODE = 0;         // disable autobaud, TX and RX enabled only, 8N1, idle=HIGH
        U2STA = 0x1400;     // enable TX and RX
        U2BRG = Baud2BRG(baud_rate); // U2BRG = (FPb / (16*baud)) - 1

        U2MODESET = 0x8000;     // enable UART2
    }

    void uart_puts(char * s)
    {
        while(*s)
        {
        putchar(*s);
        s++;
        }
    }

    void delay_ms(int msecs)
    {
        int ticks;
        ISR_frc = 0;
        ticks = msecs / 20;
        while (ISR_frc < ticks);
    }

    char HexDigit[]="0123456789ABCDEF";
    void PrintNumber(long int val, int Base, int digits)
    { 
        int j;
        #define NBITS 32
        char buff[NBITS+1];
        buff[NBITS]=0;

        j=NBITS-1;
        while ( (val>0) | (digits>0) )
        {
        buff[j--]=HexDigit[val%Base];
        val/=Base;
        if(digits!=0) digits--;
        }
        uart_puts(&buff[j+1]);
    }

    // Good information about ADC in PIC32 found here:
    // http://umassamherstm5.org/tech-tutorials/pic32-tutorials/pic32mx220-tutorials/adc
    void ADCConf(void)
    {
        AD1CON1CLR = 0x8000;    // disable ADC before configuration
        AD1CON1 = 0x00E0;       // internal counter ends sampling and starts conversion (auto-convert), manual sample
        AD1CON2 = 0;            // AD1CON2<15:13> set voltage reference to pins AVSS/AVDD
        AD1CON3 = 0x0f01;       // TAD = 4*TPB, acquisition time = 15*TAD 
        AD1CON1SET=0x8000;      // Enable ADC
    }

    int ADCRead(char analogPIN)
    {
        AD1CHS = analogPIN << 16;    // AD1CHS<16:19> controls which analog pin goes to the ADC

        AD1CON1bits.SAMP = 1;        // Begin sampling
        while(AD1CON1bits.SAMP);     // wait until acquisition is done
        while(!AD1CON1bits.DONE);    // wait until conversion done

        return ADC1BUF0;             // result stored in ADC1BUF0
    }

    void ConfigurePins(void)
    {
        // Configure pins as analog inputs
            //Modified jesus analog -> output 
            //analog pins must be init as 1 
                //ANSELBbits.ANSB2 = 1;   // set RB2 (AN4, pin 6 of DIP28) as analog pin
                //TRISBbits.TRISB2 = 1;   // set RB2 as an input
                //ANSELBbits.ANSB3 = 1;   // set RB3 (AN5, pin 7 of DIP28) as analog pin
                //TRISBbits.TRISB3 = 1;   // set RB3 as an input
                //ANSELBbits.ANSB12 = 1;   
                //TRISBbits.TRISB12 = 1;
                TRISBbits.TRISB2 = 1;
                LATBbits.LATB2 = 1;
                
                TRISBbits.TRISB3 = 1;
                LATBbits.LATB3 = 1;
        

        // Configure digital input pin to measure signal period
            ANSELB &= ~(1<<5); // Set RB5 as a digital I/O (pin 14 of DIP28)
            TRISB |= (1<<5);   // configure pin RB5 as input
            CNPUB |= (1<<5);   // Enable pull-up resistor for RB5

        // Configure output pins
            TRISAbits.TRISA0 = 0; // pin  2 of DIP28
            TRISAbits.TRISA1 = 0; // pin  3 of DIP28
            TRISBbits.TRISB0 = 0; // pin  4 of DIP28
            TRISBbits.TRISB1 = 0; // pin  5 of DIP28
            TRISAbits.TRISA2 = 0; // pin  9 of DIP28
            TRISAbits.TRISA3 = 0; // pin 10 of DIP28
            TRISBbits.TRISB4 = 0; // pin 11 of DIP28
        

        //These pins are added 
            TRISBbits.TRISB6 = 0;
            LATBbits.LATB6 = 0;
            TRISBbits.TRISB15 = 0; //14 / 15 servo arm
            LATBbits.LATB15 = 0;
            TRISBbits.TRISB14 = 0;
            LATBbits.LATB14 = 0;
            TRISBbits.TRISB13 = 0; //EM 
            LATBbits.LATB13 = 0;
        
            INTCONbits.MVEC = 1;
    }
    
    //==========================================================
    //
    //					Added Robot Functions
    //
    //==========================================================
        //Function for moving straight
        void moveStraight(void)
        {			
            LATBbits.LATB0 = 0;
            LATBbits.LATB1 = 1;
            TRISAbits.TRISA2 = 0; 
            TRISAbits.TRISA3 = 1; 		
            }
        
        void moveLeft(void)
        {			
            LATBbits.LATB0 = 0;
            LATBbits.LATB1 = 0;
            TRISAbits.TRISA2 = 0; 
            TRISAbits.TRISA3 = 1; 		
        }
        
        void moveRight(void)
        {			
            LATBbits.LATB0 = 0;
            LATBbits.LATB1 = 1;
            TRISAbits.TRISA2 = 0; 
            TRISAbits.TRISA3 = 0; 		
        }
        
        void moveBack()
        {
            LATBbits.LATB0 = 1;
            LATBbits.LATB1 = 0;
            TRISAbits.TRISA2 = 1; 
            TRISAbits.TRISA3 = 0; 
        }
        
        void moveStop()
        {
            LATBbits.LATB0 = 0;
            LATBbits.LATB1 = 0;
            TRISAbits.TRISA2 = 0; 
            TRISAbits.TRISA3 = 0; 
        }
    //==========================================================
    //
    //							MAIN
    //
    //==========================================================
    // In order to keep this as nimble as possible, avoid
    // using floating point or printf() on any of its forms!

    void main(void)
    {
        volatile unsigned long t=0;
        int adcval4, adcval5;
        long int v;
        unsigned long int count, f;
        unsigned char LED_toggle=0;
        float T;
        int i = 0;

        //for servo
        char buf[32];
        int pw;
        int base;
        
        //for adctest
        float voltage4, voltage5;
        
        CFGCON = 0;
        DDPCON = 0;
        unsigned long int freqInit;

        UART2Configure(115200);  // Configure UART2 for a baud rate of 115200
        ConfigurePins();
        SetupTimer1();

        ADCConf(); // Configure ADC
        
        waitms(500); // Give PuTTY time to start
        uart_puts("\x1b[2J\x1b[1;1H"); // Clear screen using ANSI escape sequence.
        uart_puts("\r\nPIC32 multi I/O example.\r\n");
        uart_puts("Measures the voltage at channels 4 and 5 (pins 6 and 7 of DIP28 package)\r\n");
        uart_puts("Measures period on RB5 (pin 14 of DIP28 package)\r\n");
        uart_puts("Toggles RA0, RA1, RB0, RB1, RA2 (pins 2, 3, 4, 5, 9, of DIP28 package)\r\n");
        uart_puts("Generates Servo PWM signals at RA3, RB4 (pins 10, 11 of DIP28 package)\r\n\r\n");
        
        count=GetPeriod(100);
        
        int coinCount = 0;
        
        //Initializes freqInit   
        if(count > 0)
        {
            freqInit = ((SYSCLK/2L)*100L)/count;
        }
        
        while(1)
        {
            count=GetPeriod(100);
            
            //Perimeter Detector 
            adcval4 = ADCRead(4); // reads AN4 aka RB12		
            voltage4=adcval4*3.3/1023.0;
            
            adcval5 = ADCRead(5); 	//reads AN5 aka 
            voltage5=adcval5*3.3/1023.0;
            
            //If current frequency fluctuates under freqInit, freqInit initializes again 
            if(f < freqInit)
            {
                freqInit = ((SYSCLK/2L)*100L)/count;
            }
        

        
        
            if(coinCount == MAXCOINS) //if has collected and counted max number of coins
            {
                    //does a lil dance upon win
                    moveBack();
                    waitms(500);
                    moveStop();
                    
                    moveStraight();
                    waitms(500);
                    moveStop();
                    
                    moveBack();
                    waitms(500);
                    moveStop();
                    
                    moveStraight();
                    waitms(500);
                    moveStop();
                    
                    moveLeft();
                    waitms(4000);
                    
                    moveStop();
                    waitms(8000);
                
            }
            
            //Perimeter Detector 
            adcval4 = ADCRead(4); // reads AN4 aka RB12		
            voltage4=adcval4*3.3/1023.0;
            
            adcval5 = ADCRead(5); 	//reads AN5 aka 
            voltage5=adcval5*3.3/1023.0;
            
            fflush(stdout);
            
        
            moveStraight();
            
            if((voltage4 > VOLTRANGEMIN_PERIMETER && voltage4 < VOLTRANGEMAX_PERIMETER) || (voltage5 > VOLTRANGEMIN_PERIMETER && voltage5 < VOLTRANGEMAX_PERIMETER))
            {
                //Do not put in a function. Too long runtime (calls a func within a func)
                moveStop();
                waitms(200);
                moveBack();
                waitms(1000);
                moveRight();
                waitms(1000); //45 deg
                moveStop();
            }
        
        
            
            if(count>0)
            {
                f=((SYSCLK/2L)*100L)/count;
                uart_puts("f=");
                PrintNumber(f, 10, 7);
                uart_puts("Hz, count=");
                PrintNumber(count, 10, 6);
                
                uart_puts(" freqinit=");
                PrintNumber(freqInit, 10, 7);
                uart_puts("          \r");

        //Coin detector system (added code)
        //Note to self: Turning / perimeter detector doesnt work when stuck here 
        //detects if frequency is within a range
            if (f > freqInit+1000 && f < MAXFREQ)
            { 
            
            //moves back to compensate for arm
                moveStop();
                moveBack(); 
                waitms(190);
                moveStop();
                waitms(200);
                
            pw = 60;
            base = 200;
            if ((pw >= 60) && (pw <= 240)&&(base>=60) &&(base<=240))
            {
                //grab arm drops down
                waitms(300);
                ISR_base = base;
                waitms(300);
                ISR_pw = pw+5;
                waitms(300);
            
                //turn on EM
                LATBbits.LATB13 = 1;
                //delay_ms(1000);
                
                //sweeps base a little while magnet is still on
                    waitms(2000);
                    ISR_base = 150; 
                    waitms(2000);
        
                //return to default 
                    //rotates arm
                while(ISR_pw <= 240){
                waitms(5);
                ISR_pw+=1;
                }
                // ISR_pw = 240; //rotates arm to beginning 
                    //delay_ms(2000);
                    //delay_ms(400);
                
                //rotates base to ISR_base = 70; //rotates base back to beginning
                    waitms(2000);
                    while(ISR_base >= 70) //deca
                    {
                    waitms(5);
                    ISR_base-=1;
                    }
                    
                    waitms(2000);
                                                                                                                                        
                //turn off EM
                LATBbits.LATB13 = 0;
                waitms(1000);
                
                
                
                //Reintializing the freqInit upon coin pickup
                waitms(2000);
                count=GetPeriod(100);
                if(count > 0)
                    {
                    freqInit = ((SYSCLK/2L)*100L)/count;
                    }
                    
                coinCount++;
                
            }
            else //if arm angles are out of range
            {
                printf("%d is out of the valid range\r\n", pw);
            }
        
            }
        
        } //if count bracket
        else //if cannot detect frequency 
        {
            uart_puts("NO SIGNAL                     \r");
        }

        waitms(200);
        } //while(1) bracket
    }

    ```

## Diode Heartrate Monitor (Feb. 2022 - March 2022)

This first video shows a similar project of a temperature monitor.

[Temperature Monitor Demo](https://youtu.be/DtizvK82OfQ)

[Heart-rate Monitor Demo](https://youtu.be/MpQabdz20nY)

??? "Click to show code (WIP)"
    To-do

- Developed AT89LP51RC2 micro-controller based photoelectric heart rate monitor
- Wrote software that would read micro-controller using PuTTY and C
- Further graphed data with Python using MatLab functions to graph heart rate

## AC Voltmeter (March 2022)

[Youtube Demo](https://youtu.be/pHFQ8-BD4R0)

??? "Click to show code (WIP)"
    To-do

- Built a software based AC voltmeter using C
- Understood AC circuit concepts to create voltmeter that determined magnitude, phase shift, and phase of AC signals produced by function generator
