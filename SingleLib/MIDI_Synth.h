#ifndef MIDI_SYNTH_H
#define	MIDI_SYNTH_H
#ifdef	__cplusplus
extern "C" {
#endif

#include "xc.h"
#include "math.h"
#include "stdio.h"
#include "stdlib.h"

#define CHANNEL_LIMIT 4

    double log2(double x);

    /*
     * pointers to chip parameters are declared as volatile to tell the compiler
     * that they are subject to change even if the source code does not modify them
     */
    typedef struct Timers {
        volatile uint16_t* TxCON;
        volatile uint16_t* PRx;
        volatile uint16_t* TMRx;
        volatile uint16_t* OCxCON;
        volatile uint16_t* OCxRS;
    } Timer;
    /*
     * Pre-definitions of on board timers for easy assignment
     * 
     * static definitions of timers to prevent external changes
     */
    static Timer T2 = {.TxCON = &T2CON,
        .PRx = &PR2,
        .TMRx = &TMR2,
        .OCxCON = &OC1CON,
        .OCxRS = &OC1RS};
    static Timer T3 = {.TxCON = &T3CON,
        .PRx = &PR3,
        .TMRx = &TMR3,
        .OCxCON = &OC2CON,
        .OCxRS = &OC2RS};
    static Timer T4 = {.TxCON = &T4CON,
        .PRx = &PR4,
        .TMRx = &TMR4};
    static Timer T5 = {.TxCON = &T5CON,
        .PRx = &PR5,
        .TMRx = &TMR5};

    static Timer* channelTimers[] = {&T2, &T3, &T4, &T5};


    volatile static uint16_t* _RPOR_[] ={
        &RPOR0,
        &RPOR1,
        &RPOR2,
        &RPOR3,
        &RPOR4,
        &RPOR5,
        &RPOR6,
        &RPOR7
    };


    struct Channels; //forward declaration of Channel struct
    typedef struct Channels Channel;

    void initChannel(Channel*, uint16_t);
    typedef void (*ChannelFunc_Null)(Channel*);
    typedef void (*ChannelFunc_1Arg)(Channel*, uint16_t);

    void setPin(Channel * this, uint16_t pin);
    void setFreq(Channel * this, uint16_t freq);
    void stop(Channel * this);
    void start(Channel * this);

    struct Channels {
        //struct variables
        char avail;
        Timer timer;
        uint16_t pin, ocr, freq;
        Channel * this; //passed to pointer functions so they can modify the original parameters


        //struct functions/methods
        ChannelFunc_1Arg setPin;
        ChannelFunc_1Arg setFreq;
        ChannelFunc_Null stop;
        ChannelFunc_Null start;
    };     


    //source from midi reception library
    void MIDI_init(void);
    void turnOn(uint8_t note, uint8_t velocity);
    void turnOff(uint8_t note);
    void __attribute__((interrupt, auto_psv)) _U1RXInterrupt(void);
    void attachChannel(Channel*);
#ifdef	__cplusplus
}
#endif

#endif	/* MIDI_SYNTH_H */

