#include <xc.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "../printf_Write2UART.X/mcc_generated_files/system/clock.h"

// Configuration bits
//#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config FOSC = HS    // Oscillator Selection - HS= High Speed, 
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = ON        // Internal/External Switchover Mode (Internal/External Switchover Mode is enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// Define the clock frequency
#define _XTAL_FREQ 16000000

//#define BUFFER_SIZE 20

    //These ports/ function may mess you around.
#define TRISA TRISA 0xFF
#define TRISB TRISB 0x3B
#define LATA LATA 0x0
#define ANSELA 0x0
#define ANSELB 0x0
#define WPUA 0x20
#define WPUB 0xFF
#define OPTION_REGbits.nWPUEN 0x0
#define APFCON0 0x0 //RB1->EUSART:RX;
#define APFCON1 0x0 //RB2->EUSART:TX;
#define ADCON0 0x0

    volatile int numChars = 0;
    volatile char rxdChar[25];  //Character array
    volatile int num = 0; //Number in the rxdChar character array.
    volatile int readData = 0;  //We got data bit
    const char exp = 'LaneLights01\n\r   '; //Expected ID
   
void EUSART_Init(void) {
    // Set the baud rate to 1200
    SPBRGH = 0x34;
    SPBRGL = 0x10;
  
    // Enable serial port and configure as 8-bit asynchronous
    TXSTAbits.SYNC = 0; // Asynchronous mode
    TXSTAbits.BRGH = 1; // Low speed
    BAUDCONbits.BRG16 = 1; // USe 16 bits for accuracy.
    RCSTAbits.SPEN = 1; // Enable serial port

    // Enable transmission and reception
    TXSTAbits.TXEN = 1; // Enable transmission
    RCSTAbits.CREN = 1; // Enable continuous reception
    
}

void EUSART_Write(char data) {
    while (!PIR1bits.TXIF); // Wait until the transmit buffer is empty
    TXREG = data; // Transmit the data
}

void EUSART_WriteString(const char *str) {
    while (*str) {
        //LATAbits.LATA1 = 1;
        EUSART_Write(*str++);
    }
}


//Interrupt routine for ESUART  receiver
//void __interrupt(high_priority) ISR(numChars)h
 //__interrupt(high_priority) ISR(char *rxdChar)
__interrupt(high_priority) ISR(void) {
    LATAbits.LATA1 = 0;  //Reset ID match for new data.
    rxdChar[num] = RCREG;
    num++;
    if (rxdChar == '\n' || rxdChar == '\r' || num >= 16) {
            num = 0;
            readData=1;    //Set we got data bit.
            RCSTAbits.CREN = 0;
        }
    
}


void main(void) {
    //TRISBbits.TRISB2 = 0;   //Set the UART TX port as an output to stop an idle high state
    TRISAbits.TRISA1 = 0;   //Set Diag LED.
    TRISBbits.TRISB1 = 1;
    TRISBbits.TRISB2 = 0;   //Set the UART TX port as an output to stop an idle high state
    LATAbits.LATA1 = 0; //Turn LED on.
    //char buffer[BUFFER_SIZE];
    //char tx_buffer[BUFFER_SIZE];
    ANSELBbits.ANSB2 = 0;    //ADC can cause EUSART issues.
    ANSELBbits.ANSB1 = 0;    //ADC can cause EUSART issues.
    CPSCON0 = 0x00;
    CPSCON1 = 0x00;
    //Configure interrupt
    INTCONbits.GIE = 1;    //Global interrupt enabled
    INTCONbits.PEIE = 1;   //Peripheral interrupt enables
    PIE1bits.RCIE = 1;     //Receiver interrupt enabled
    RCSTAbits.CREN = 1; // Enable continuous reception
    
    // Initialize the EUSART
    EUSART_Init();

    // Main loop
    while (1) {
        if (readData == 1) {
        TXSTAbits.TXEN = 1; // Enable transmission
        EUSART_WriteString(rxdChar); // Write the received string back out through EUSART
        TXSTAbits.TXEN = 0; // Disable transmission*/
        if (exp == rxdChar) {
            LATAbits.LATA1 = 1; //Turn LED on if the ID is correct.
            //rxdChar[18] = 'Message Received\n\r';
            //EUSART_WriteString(rxdChar); // Write the string out through EUSART
        }
        RCSTAbits.CREN = 1; //Enable EUSART receiver continuous receive
        readData = 0; //Reset we got data bit.
        }
    }
}

