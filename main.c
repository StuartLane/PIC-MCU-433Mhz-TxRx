#include <xc.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
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

    char rxdChar[16];  //Character array
    char idChar[16];   //Temporary index characters
    int num = 0; //Number in the rxdChar character array.
    int readData = 0;  //We got data bit
    char exp[] = "LaneLights01\n\r  "; //Expected ID
    char val;
    uint16_t start_addr = 0x00;
    int loc;
    int outputStatus = 0;
    //char rxdData;
    char index;
    char id0[16];
    char id1[16];
    char id2[16];
    char id3[16];
    
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
    //LATAbits.LATA1 = 0;
    rxdChar[num] = RCREG;
    num++;
    if (rxdChar == '\n' || rxdChar == '\r' || num >= 16) {
            num = 0;
            readData = 1;    //Set we got data bit.
            RCSTAbits.CREN = 0;
            LATAbits.LATA1 = 0;
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
//Read the eeprom id0 into RAM
set_index0() {
    start_addr = 0x0;
    for (uint8_t id_set=0; id_set<=12; id_set++){
        read_id(start_addr);
        id0[id_set]=EEDATA;
        start_addr++;
        }
}

//Read the eeprom id1 into RAM
set_index1() {
    start_addr = 0x10;
    for (uint8_t id_set=0; id_set<=16; id_set++){
        read_id(start_addr);
        id1[id_set]=EEDATA;
        start_addr++;
        }
}

//Read the eeprom id2 into RAM
set_index2() {
    start_addr = 0x20;
    for (uint8_t id_set=0; id_set<=16; id_set++){
        read_id(start_addr);
        id2[id_set]=EEDATA;
        start_addr++;
        }
}

//Read the eeprom id3 into RAM
set_index3() {
    start_addr = 0x30;
    for (uint8_t id_set=0; id_set<=16; id_set++){
        read_id(start_addr);
        id3[id_set]=EEDATA;
        start_addr++;
        }
}

//Check if stored id string is the same at the string received via ESUART
    bool compareIdToRecd(int id[], int rxd[], int compareLength){
        for(int i = 0; i <compareLength; i++){
            if(id[i] != rxd[i]){
                return false;  //id and received string are not the same
        }
    }
        return true; //id and received string are the same
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
    
    start_addr = 0x41;  //Last address for the index, load index for ids
    read_id(start_addr); //Let get the index number ie, how many transmitters talk to us.
    index=EEDATA;
    
    if (index != 0x00 || index != 0x10 || index != 0x20 || index != 0x30) {
                index = 0x00;     //One IDs saved
                start_addr = 0x41;
                add_id (start_addr, index);
            }
    
    if (index == 0x00){
        set_index0();
    }
    else if(index == 0x10){
        set_index0();
        set_index1();
    }
    else if(index == 0x20){
        set_index0();
        set_index1();
        set_index2();
    }
    else if(index == 0x30){
        set_index0();
        set_index1();
        set_index2();
        set_index3();
    }
    else {
        
    }
    
    
    
    // Main loop
    while (1) {
        if (readData == 1) {   //We have received data from UART.
            TXSTAbits.TXEN = 1; // Enable transmission
            int length = 15;  //strlen(rxdChar);
            start_addr=index;
                for (int count=0; count<=length; count++){
                    read_id(start_addr);
                    //EUSART_Write(EEDATA);
                    EUSART_Write(rxdChar[count]);  //Write to UART what we have received.
                    EUSART_Write(id0[count]);
                    __delay_ms(200);  //Do not remove!!!
                    start_addr++;  
                }
            //
            //if (id0[0,1,2,3,4,5,6,7,8,9,10,11] == rxdChar[0,1,2,3,4,5,6,7,8,9,10,11]){
            if (compareIdToRecd(id0, rxdChar,12)){
            //int id0_comp = strcmp(rxdChar, id0);
            //int id1_comp = strcmp(rxdChar, id1);
            //int id2_comp = strcmp(rxdChar, id2);
            //int id3_comp = strcmp(rxdChar, id3);
            //if (id0_comp == 0 || id1_comp == 0 || id2_comp == 0 || id3_comp == 0){
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
            //LATAbits.LATA1 = 0; //Turn LED off again*/
        }
        //Set up ID or IDs 
        if (PORTBbits.RB0 == 1){     //Receiver touch switch
            RCSTAbits.CREN = 1;
            LATAbits.LATA1 = 1; //Indicate that we are in the id routine.
            /*if (index != 0x00 || index != 0x10 || index != 0x20 || index != 0x30) {
                index = 0x00;     //One IDs saved
            }
            else if (index == 0x00) {
                index = 0x10;      //Two IDs saved
            }
            else if (index == 0x10) {
                index = 0x20;     //Three IDs saved
            }   
            else if (index == 0x20) {
                index = 0x30;    //Four IDs saved
            }
            else if (index == 0x30) {
                index = 0x00;       //Roll around and overwrite the first ID.
            }*/
           
            //start_addr = 0x41;
                //add_id(start_addr, index);  //Save the latest index value
            //LATAbits.LATA1 = 0;
                 //Enable EUSART receiver continuous receive
            start_addr = index;
            if (readData == 0) { 
                //int length = strlen(rxdChar);
                int length = 15;
                for (int count=0; count<=length; count++){
                    char rxdData = rxdChar[count];
                    add_id(start_addr, rxdData);
                    start_addr++;
                }
                
                //if (index == 0x00){
                    set_index0();
                    //memcpy(id0, rxdChar, sizeof(rxdChar));
                //}
            }
        }
    }
}