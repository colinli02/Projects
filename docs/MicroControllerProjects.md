# Micro-controller Projects

## **Robotic Coin Picker (March 2022 - April 2022)**

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

## **Diode Heartrate Monitor (Feb. 2022 - March 2022)**

This first video shows a similar project of a temperature monitor using similar principles, based on the same code but with different hardware setups.

The circuit works by using the op-amp to amplify the weak signal emitted from the light passing through the tissue containing blood. 
Then this information is processed using the microcontroller circuit and 2 codes to generate a heart rate monitor.

[Temperature Monitor Demo](https://youtu.be/DtizvK82OfQ)

[Heart-rate Monitor Demo](https://youtu.be/MpQabdz20nY)
??? "Click to show C code"
    ```
    // This code is mostly from http://efundies.com/avr-and-printf/

    #define F_CPU 16000000UL

    #include <avr/io.h>
    #include <stdio.h>
    #include <stdbool.h>
    #include <util/delay.h>
    #include "usart.h"

    unsigned int ReadChannel(unsigned char mux)
    {
        ADCSRA = (1<<ADEN) | (1<<ADPS1) | (1<<ADPS0); // frequency prescaler
        ADMUX = mux; // channel select
        ADMUX |= (1<<REFS1) | (1<<REFS0); 
        ADCSRA |= (1<<ADSC); // Start conversion
        while ( ADCSRA & (1<<ADSC) ) ;
        ADCSRA |= (1<<ADSC); // a transformation “single conversion”
        while ( ADCSRA & (1<<ADSC) );
        ADCSRA &= ~(1<<ADEN); // Disable ADC
        return ADCW;
    }

    int main( void )
    {
        unsigned int adc;
        
        usart_init (); // configure the usart and baudrate
        DDRB |= 0x01;
        PORTB |= 0x01;

        printf("\nADC test\n");

        while(1)
        {
            adc=ReadChannel(0);
            //printf("ADC[0]=0x%03x, %fV\n", adc, (adc*5.0)/1023.0);
            printf("%f\n",(adc*5.0)/1023.0);
            PORTB ^= 0x01;
            _delay_ms(500);
        }
    }

    ```


??? "Click to show Py code"

    ``` py linenums="1" title="heart_mtr.py"
    import string


    import time
    import serial
    import serial.tools.list_ports



    import numpy as np
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    import sys, time, math

    xsize=900

    # configure the serial port
    try:
        ser = serial.Serial(
            port='COM4', # Change as needed
            baudrate=115200,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_TWO,
            bytesize=serial.EIGHTBITS
        )
        ser.isOpen()
    except:
        portlist=list(serial.tools.list_ports.comports())
        print ('Available serial ports:')
        for item in portlist:
        print (item[0])
        exit()

    
    def data_gen():
        t = data_gen.t
        while True:
        #val = ''
        t+=1
        
        val = ser.readline() #reads the port as a byte
        #print(type(val))#currently is a byte

        #convert byte to float
        #convert byte into string first
        val = val.decode('utf-8') 
        #print(type(val)) #currently is a string
        
        #removes all letters except beginning 
        #removes all letters
        val = val.replace('V0=', '')
        val = val.replace('V1=', '')
        val = val.replace('V2=', '')
        val = val.replace('V3=', '')
        val = val.replace('V4=', '')
        val = val.replace('V5=', '')
        val = val.replace('V6=', '')
        val = val.replace('V7=', '')
        val = val.replace('V', '')
        val = val.replace('=', '')
        #removes random left over characters that block float conversion 
        val = val.replace('=', '')
        val = val.replace(' ', '')
        val = val.replace(' '' ', '')
        
        val = val[0:5] #keeps first 4 digits of string

        #then converts string to float
        #print(type(val)) #currently is a string
        #print(len(val)) #for debugging
        val = float(val)
        val = val/45
        #val = val*2

        #appends t
        #time = "*t"
        #listOfStrings = [val,time]
        #val = "".join(listOfStrings)
        
        #print(type(val))
        #val=100.0*math.sin(t*2.0*3.1415/100.0) #this is a float
        print(val)
        yield t, val

    def run(data):
        # update the data
        t,y = data
        if t>-1:
            xdata.append(t)
            ydata.append(y)
            if t>xsize: # Scroll to the left.
                ax.set_xlim(t-xsize, t)
            line.set_data(xdata, ydata)

        return line,

    def on_close_figure(event):
        sys.exit(0)

    data_gen.t = -1
    fig = plt.figure()
    fig.canvas.mpl_connect('close_event', on_close_figure)
    ax = fig.add_subplot(111)
    line, = ax.plot([], [], lw=2)
    ax.set_ylim(0, 120)
    ax.set_xlim(0, xsize)
    ax.grid()
    xdata, ydata = [], []

    # Important: Although blit=True makes graphing faster, we need blit=False to prevent
    # spurious lines to appear when resizing the stripchart.
    ani = animation.FuncAnimation(fig, run, data_gen, blit=False, interval=50, repeat=False)
    plt.show()
    ```

- Developed AT89LP51RC2 micro-controller based photoelectric heart rate monitor
- Wrote software that would read micro-controller using PuTTY and C
- Further graphed data with Python using MatLab functions to graph heart rate

## **AC Voltmeter (March 2022)**

[Youtube Demo](https://youtu.be/pHFQ8-BD4R0)

??? "Click to show code"
    ``` c title="adc_voltmeter.c"
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
    ```

- Built a software based AC voltmeter using C
- Understood AC circuit concepts to create voltmeter that determined magnitude, phase shift, and phase of AC signals produced by function generator
