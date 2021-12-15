/*
 * File:   main.c
 * Author: Hunter Jans
 *
 * Created on December 1, 2021, 10:37 AM
 */


#include "xc.h"
#include "MIDI_Device.h"
#include "PIC24_MIDI_receiver.h"

// CW1: FLASH CONFIGURATION WORD 1 (see PIC24 Family Reference Manual 24.1)
#pragma config ICS = PGx1          // Comm Channel Select (Emulator EMUC1/EMUD1 pins are shared with PGC1/PGD1)
#pragma config FWDTEN = OFF        // Watchdog Timer Enable (Watchdog Timer is disabled)
#pragma config GWRP = OFF          // General Code Segment Write Protect (Writes to program memory are allowed)
#pragma config GCP = OFF           // General Code Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = OFF        // JTAG Port Enable (JTAG port is disabled)


// CW2: FLASH CONFIGURATION WORD 2 (see PIC24 Family Reference Manual 24.1)
#pragma config I2C1SEL = PRI       // I2C1 Pin Location Select (Use default SCL1/SDA1 pins)
#pragma config IOL1WAY = OFF       // IOLOCK Protection (IOLOCK may be changed via unlocking seq)
#pragma config OSCIOFNC = ON       // Primary Oscillator I/O Function (CLKO/RC15 functions as I/O pin)
#pragma config FCKSM = CSECME      // Clock Switching and Monitor (Clock switching is enabled, 
// Fail-Safe Clock Monitor is enabled)
#pragma config FNOSC = FRCPLL      // Oscillator Select (Fast RC Oscillator with PLL module (FRCPLL))



Channel ch[4];

void setup(void) {
    _RCDIV = 0; //16 MHz frequency
    AD1PCFG = 0xffff; //set all pins to digital  
    TRISB = 0;
    initChannel(&ch[0], 6);
    initChannel(&ch[1], 7);
    initChannel(&ch[2], 8);
    initChannel(&ch[3], 9);
}

int main(int argc, char** argv) {
    setup();
    MIDI_init(); //initialize midi receiver and pass the desired output midiDevice
    while (1) {

    }
    return 0;
}

