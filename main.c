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

    char rxdChar[25];  //Character array
    int num = 0; //Number in the rxdChar character array.
    int readData = 0;  //We got data bit
    char exp[] = "LaneLights01\n\r  "; //Expected ID
    char val;
    uint16_t start_addr = 0x00;
    int loc;
    int outputStatus = 0;
    char rxdData;
    char id1[16];
    char id2[16];
    char id3[16];
    char id4[16];
    
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

//Interrupt routine for ESUART  receiver
//void __interrupt(high_priority) ISR(numChars)h
 //__interrupt(high_priority) ISR(char *rxdChar)
__interrupt(high_priority) ISR(void) {
    rxdChar[num] = RCREG;
    num++;
    if (rxdChar == '\n' || rxdChar == '\r' || num >= 16) {
            num = 0;
            readData = 1;    //Set we got data bit.
            RCSTAbits.CREN = 0;
        }
    
}

//Write new IDs into eeprom
add_id(uint16_t start_addr, char rxdData){
            //EEDATA = 0x45;
            EEADR = start_addr;
            EEDATA = rxdData;
            EECON1bits.CFGS = 0;
            EECON1bits.EEPGD = 0;
            EECON1bits.WREN = 1;
            INTCONbits.GIE = 0;
            EECON2 = 0x55;
            EECON2 = 0xAA;
            EECON1bits.WR = 1;
            INTCONbits.GIE = 1;
            EECON1bits.WREN = 0;
            while (EECON1bits.WR == 1){
            }
            }

//Read from data in eeprom id locations
read_id(uint16_t start_addr){
    //EUSART_Write(rxdChar[count]); // Write the received string back out through EUSART
            EEADR = start_addr;
            EECON1bits.EEPGD = 0;
            EECON1bits.CFGS = 0;
            EECON1bits.RD = 1;
            return (EEDATA);
}


void main(void) {
    TRISAbits.TRISA1 = 0;   //Set Diag LED.
    TRISBbits.TRISB1 = 1;   //Set the UART RX port as an input
    TRISBbits.TRISB2 = 0;   //Set the UART TX port as an output to stop an idle high state
    TRISBbits.TRISB0 = 1;   //Pin used touch switch
    LATAbits.LATA1 = 0; //Turn LED on.
    ANSELBbits.ANSB2 = 0;    //ADC can cause EUSART issues.
    ANSELBbits.ANSB1 = 0;    //ADC can cause EUSART issues.
    CPSCON0 = 0x00;
    CPSCON1 = 0x00;
    //Configure interrupt
    INTCONbits.GIE = 1;    //Global interrupt enabled
    INTCONbits.PEIE = 1;   //Peripheral interrupt enabled
    PIE1bits.RCIE = 1;     //Receiver interrupt enabled
    RCSTAbits.CREN = 1; // Enable continuous reception
    
    // Initialize the EUSART
    EUSART_Init();
    
    start_addr = 0x00;
    for (uint8_t id_set=0; id_set<=16; id_set++){
    read_id(start_addr);
    id1[id_set]=EEDATA;
    //EUSART_Write(EEDATA);
    start_addr++;
    }
            

    // Main loop
    while (1) {
        if (readData == 1) {   //We have received data from UART.
        TXSTAbits.TXEN = 1; // Enable transmission
        uint8_t length = strlen(rxdChar);
        start_addr=0x00;
        for (int count=0; count<=length; count++){
            read_id(start_addr);
            EUSART_Write(EEDATA);
            //EUSART_Write(rxdChar[count]);  //Write to UART what we have received.
            __delay_ms(200);  //Do not remove!!!
                start_addr++;    
        }
        if (rxdChar[0,1,2,3,4,5,6,7,8,9,10,11,12] == id1[0,1,2,3,4,5,6,7,8,9,10,11,12]){
            if (outputStatus == 0){
            LATAbits.LATA1 = 1;
            outputStatus = 1;
        }
            else {
                LATAbits.LATA1 = 0;
                outputStatus = 0;
            }
        }
        TXSTAbits.TXEN = 0; // Disable transmission*/       
        RCSTAbits.CREN = 1; //Enable EUSART receiver continuous receive
        readData = 0; //Reset we got data bit.
        //LATAbits.LATA1 = 0; //Turn LED off again
        }
        //Set up ID or IDs 
        if (PORTBbits.RB0 == 1){     //Receiver touch switch
            LATAbits.LATA1 = 1; //Indicate that we are in the id routine.
            //if (readData == 1) {
                int length = strlen(rxdChar);
                start_addr=0x00;
                for (int count=0; count<=length; count++){
                    rxdData = rxdChar[count];
                    add_id(start_addr, rxdData);
                    start_addr++;
                }
                LATAbits.LATA1 = 0;
                readData = 0;
            //}
        }
    }
}

