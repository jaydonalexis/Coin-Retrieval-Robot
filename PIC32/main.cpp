#include <XC.h>
#include <sys/attribs.h>
#include <stdio.h>
#include <stdlib.h>

// Configuration Bits
#pragma config FNOSC = FRCPLL
#pragma config FPLLIDIV = DIV_2
#pragma config FPLLMUL = MUL_20
#pragma config FPLLODIV = DIV_2
#pragma config FWDTEN = OFF
#pragma config FPBDIV = DIV_1
#pragma config FSOSCEN = OFF

// Defines
#define SYSCLK 40000000L
#define FREQ 100000L
#define NBITS 32
#define FREQUENCY_THRESHOLD 47600
#define VOLTAGE_THRESHOLD 0.8
#define FREQUENCY_CYCLE 30
#define PIN_PERIOD (PORTB & (1 << 5))
#define COIN_FLAG (PORTB & (1 << 4))
#define BAUD_TO_BRG(desiredBaud)((SYSCLK / (16 * desiredBaud)) - 1)

volatile int pulseWidthISROne = 150;
volatile int pulseWidthISRTwo = 150;
volatile int counterISR = 0;

// The interrupt service routine for timer 1 is used to generate one or more standard servo signals
void __ISR(_TIMER_1_VECTOR, IPL5SOFT) Timer1_Handler(void) {
    // Clear timer 1 interrupt flag, bit 4 of IFS0
    IFS0CLR = _IFS0_T1IF_MASK;
    
    counterISR++;
    
    if(counterISR == pulseWidthISROne) {
        LATAbits.LATA3 = 0;
    }
    
    if(counterISR == pulseWidthISRTwo) {
        LATBbits.LATB4 = 0;
    }
    
    // 2000 * 10us = 20ms
    if(counterISR >= 2000) {
        counterISR = 0;
        LATAbits.LATA3 = 1;
        LATBbits.LATB4 = 1;
    }
}

void Timer1_Setup(void) {
    __builtin_disable_interrupts();
    // Since SYSCLK/FREQ = PS * (PR1 + 1)
    PR1 = (SYSCLK / FREQ) - 1;
    TMR1 = 0;
    // 3 = 1:256 prescale value, 2 = 1:64 prescale value, 1 = 1:8 prescale value, 0 = 1:1 prescale value
    T1CONbits.TCKPS = 0;
    // Clock source
    T1CONbits.TCS = 0;
    T1CONbits.ON = 1;
    IPC1bits.T1IP = 5;
    IPC1bits.T1IS = 0;
    IFS0bits.T1IF = 0;
    IEC0bits.T1IE = 1;
    
    // Integer multi-vector
    INTCONbits.MVEC = 1;
    __builtin_enable_interrupts();
}

// Use the core timer to wait for 1 ms.
void Wait_Millisecond(void) {
    // Resets the core timer count
    _CP0_SET_COUNT(0);
    
    // Get the core timer count
    while (_CP0_GET_COUNT() < (SYSCLK / (2 * 1000)));
}

void Wait_Milliseconds(int timeLength) {
    while(timeLength--) Wait_Millisecond();
}

long int Get_Period(int perNo) {	
    // Resets the core timer count
    _CP0_SET_COUNT(0);
    
    // Wait for square wave to be 0
    while (PIN_PERIOD != 0) {
        if(_CP0_GET_COUNT() > (SYSCLK / 4)) return 0;
    }
    
    // Resets the core timer count
    _CP0_SET_COUNT(0);
    
    // Wait for square wave to be 1
    while (PIN_PERIOD == 0) {
        if(_CP0_GET_COUNT() > (SYSCLK / 4)) return 0;
    }
    
    // Resets the core timer count
    _CP0_SET_COUNT(0);
    
    // Measure the time of 'perNo' periods
    for(int iterator = 0; iterator < perNo; iterator++) {
        // Wait for square wave to be 0
        while (PIN_PERIOD != 0) {
            if(_CP0_GET_COUNT() > (SYSCLK / 4)) return 0;
        }
        
        // Wait for square wave to be 1
        while (PIN_PERIOD == 0) {
            if(_CP0_GET_COUNT() > (SYSCLK / 4)) return 0;
        }
    }
    
    return _CP0_GET_COUNT();
}

void Configure_UART2(int baudRate) {
    // SET RX to RB8
    U2RXRbits.U2RXR = 4;
    // SET RB9 to TX
    RPB9Rbits.RPB9R = 2;
    
    // Disable autobaud, TX and RX enabled only, 8N1, idle = HIGH
    U2MODE = 0;
    // Enable TX and RX
    U2STA = 0x1400;
    // U2BRG = (FPb / (16 * baud)) - 1
    U2BRG = BAUD_TO_BRG(baudRate);
    
    // Enable UART2
    U2MODESET = 0x8000;
}

void Print(char *character) {
    while(*character) {
        putchar(*character);
        character++;
    }
}

void Print_Number(long int value, int baseType, int digits) {
    int bufferIndex;
    char buffer[NBITS + 1];
    char hexDigit[] = "0123456789ABCDEF";

    buffer[NBITS] = 0;
    bufferIndex = NBITS - 1;
    
    while ((value > 0) | (digits > 0)) {
        buffer[bufferIndex--] = hexDigit[value % baseType];
        value /= baseType;
        if(digits != 0) digits--;
    }
    
    Print(&buffer[bufferIndex + 1]);
}

void Configure_ADC(void) {
    // Disable ADC before configuration
    AD1CON1CLR = 0x8000;
    // Internal counter ends sampling and starts conversion (auto-convert), manual sample
    AD1CON1 = 0x00E0;
    // AD1CON2<15:13> set voltage reference to pins AVSS/AVDD
    AD1CON2 = 0;
    // TAD = 4 * TPB, acquisition time = 15 * TAD
    AD1CON3 = 0x0f01;
    // Enable ADC
    AD1CON1SET = 0x8000;
}

int Read_ADC(char analogPin) {
    // AD1CHS<16:19> controls which analog pin goes to the ADC
    AD1CHS = analogPin << 16;
    
    // Begin sampling
    AD1CON1bits.SAMP = 1;
    // Wait until acquisition is done
    while(AD1CON1bits.SAMP);
    // Wait until conversion done
    while(!AD1CON1bits.DONE);
    
    // Result stored in ADC1BUF0
    return ADC1BUF0;
}

void Configure_Pins(void) {
    // Set RB2 (AN4, pin 6 of DIP28) as analog pin
    ANSELBbits.ANSB2 = 1;
    // Set RB2 as an input
    TRISBbits.TRISB2 = 1;
    // Set RB3 (AN5, pin 7 of DIP28) as analog pin
    ANSELBbits.ANSB3 = 1;
    // Set RB3 as an input
    TRISBbits.TRISB3 = 1;
    
    // Set RB5 as a digital I/O (pin 14 of DIP28)
    ANSELB &= ~(1 << 5);
    // Configure pin RB5 as input
    TRISB |= (1 << 5);
    // Enable pull-up resistor for RB5
    CNPUB |= (1 << 5);
    
    // Set RB4 as an input
    TRISBbits.TRISB4 = 1;
    // Configure as pull-down input
    CNPDB |= (1 << 4);
    // Confugre as pull-down input
    CNPUB &= ~(1 << 4);
    
    // Pin  2 of DIP28
    TRISAbits.TRISA0 = 0;
    // Pin  3 of DIP28
    TRISAbits.TRISA1 = 0;
    // Pin  4 of DIP28
    TRISBbits.TRISB0 = 0;
    // Pin  5 of DIP28
    TRISBbits.TRISB1 = 0;
    // Pin  9 of DIP28
    TRISAbits.TRISA2 = 0;
    // Pin 10 of DIP28
    TRISAbits.TRISA3 = 0;
    
    INTCONbits.MVEC = 1;
}

/*
 * This function will return 1 if the frequency of the Colpitts Oscillator circuit is above some threshold we set for it
 * This means that the function will only return 1 if there is a coin, or another pieace of magnetically permeable material 
 * in close proximity of the inductor
 */
int Check_Coins(void) {
    unsigned long int frequency, count;
    count = Get_Period(100);
    
    if(count > 0) {
        frequency = ((SYSCLK / 2L) * 100L) / count;
        
        if (frequency > FREQUENCY_THRESHOLD) {
            return 1;
        }
    }
    
    return 0;
}

/*
 * This function returns the frequency of the colpitts oscillator circuit
 * If there is no signal, the frequency is 0
 */
unsigned long int Coin_Frequency(void) {
    unsigned long int frequency, count;
    count = Get_Period(100);
    
    if(count > 0) {
        frequency = ((SYSCLK / 2L) * 100L) / count;
    }
    else {frequency = 0;}
    
    return frequency;
}

/*
 * This function checks if the robot's perimeter is nearby so that it can avoid it.
 * It does this by checking the output of the perimeter detection circuit.
 * If the voltage read is greater than 800 mV, then the robot has detected a magnetic
 * field which has magnetically coupled with the inductor and increased the voltage drop
 * across it
 */
void Check_Perimeter(void) {
    int valueOneADC, valueTwoADC;
    float voltageOne, voltageTwo;
    
    valueOneADC = Read_ADC(4);
    valueTwoADC = Read_ADC(5);
    
    voltageOne = (valueOneADC * 3.29) / 1023.0;
    voltageTwo = (valueTwoADC * 3.29) / 1023.0;
    
    if (voltageOne > VOLTAGE_THRESHOLD || voltageTwo > VOLTAGE_THRESHOLD) {
        LATAbits.LATA3 = 1;
    }
    else {
        LATAbits.LATA3 = 0;
    }
    
    return;
}

int Get_Coin_Value(unsigned long int frequency) {
    int iterator, coinValue;
    unsigned long int difference = 50000;
    int baseFrequencies[] = {47200, 47500, 47850, 48000, 48270, 48330};
    
    for (iterator = 0; iterator < 6; iterator++) {
        if(abs(frequency - baseFrequencies[iterator]) < difference) {
            difference = abs(frequency - baseFrequencies[iterator]);
            coinValue = iterator;
        }
    }
    
    return coinValue;
}

void Select_Pin(int pin) {
    if (pin == 1) {LATAbits.LATA1 = 1; Wait_Milliseconds(500); LATAbits.LATA1 = 0;}
    
    if (pin == 2) {LATAbits.LATA0 = 1; Wait_Milliseconds(500); LATAbits.LATA0 = 0;}
    
    if (pin == 3) {LATBbits.LATB0 = 1; Wait_Milliseconds(500); LATBbits.LATB0 = 0;}
    
    if (pin == 4) {LATBbits.LATB1 = 1; Wait_Milliseconds(500); LATBbits.LATB1 = 0;}
    
    if (pin == 5) {LATAbits.LATA2 = 1; Wait_Milliseconds(500); LATAbits.LATA2 = 0;}
    
    return;
}

void Differentiate_Coins(void) {
    unsigned long int frequency;
    unsigned long int maxFrequency = 0;
    int coin = 0;
    int coins[] = {0, 10, 5, 25, 100, 200};
    
    if (Check_Coins()) {
        for (int iterator = 0; iterator < FREQUENCY_CYCLE; iterator++) {
            frequency = Coin_Frequency();
            
            if (frequency > maxFrequency) {maxFrequency = frequency;}
        }
        
        coin = Get_Coin_Value(maxFrequency);
        printf("Coin detected is %i and f: %lu\r", coins[coin], maxFrequency);
        LATAbits.LATA0 = 1; Wait_Milliseconds(500); LATAbits.LATA0 = 0;
    }
}

void main(void) {
    int valueADC;
    long int convertedValueOne, convertedValueTwo;
    unsigned long int count, frequency;
    unsigned char toggleDiode = 0;
    
    CFGCON = 0;
    
    Configure_UART2(115200);
    Configure_Pins();
    Timer1_Setup();
    
    Configure_ADC();
    Wait_Milliseconds(500);
    
    while (1) {
        if (COIN_FLAG == 0) {
            Differentiate_Coins();
        }
    }
}

void Robot_Base(void) {
    int valueADC;
    long int convertedValueOne, convertedValueTwo;
    unsigned long int count, frequency;
    unsigned char toggleDiode = 0;
    
    CFGCON = 0;
    
    // Configure UART2 for a baud rate of 115200
    Configure_UART2(115200);
    Configure_Pins();
    Timer1_Setup();
    // Configure ADC
    Configure_ADC();
    
    // Give PuTTY time to start
    Wait_Milliseconds(500);
    // Clear screen using ANSI escape sequence.
    Print("\x1b[2J\x1b[1;1H");
    
    // Turn PIN A1 OFF
    LATAbits.LATA1 = 0;
    
    while(1) {
        valueADC = Read_ADC(4);
        Print("ADC[4] = 0x");
        Print_Number(valueADC, 16, 3);
        Print(", V = ");
        // 3.290 is VDD
        convertedValueOne = (valueADC * 3290L) / 1023L;
        Print_Number(convertedValueOne / 1000, 10, 1);
        Print(".");
        Print_Number(convertedValueOne % 1000, 10, 3);
        Print("V ");
        
        valueADC = Read_ADC(5);
        Print("ADC[5] = 0x");
        Print_Number(valueADC, 16, 3);
        Print(", V = ");
        // 3.290 is VDD
        convertedValueTwo = (valueADC * 3290L) / 1023L;
        Print_Number(convertedValueTwo / 1000, 10, 1);
        Print(".");
        Print_Number(convertedValueTwo % 1000, 10, 3);
        Print("V ");
        
        count = Get_Period(100);
        
        if(count > 0) {
            frequency = ((SYSCLK / 2L) * 100L) / count;
            Print("f = ");
            Print_Number(frequency, 10, 7);
            Print("Hz, count = ");
            Print_Number(count, 10, 6);
            Print(" \r");
        }
        else {
            Print("NO SIGNAL \t\t\t\r");
        }
        
        Check_Coins();
        Check_Perimeter();
        Wait_Milliseconds(200);
    }
}
