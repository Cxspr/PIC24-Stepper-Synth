#ifndef PIC24_MIDI_RECEIVER_H
#define	PIC24_MIDI_RECEIVER_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "xc.h"
#include <stdlib.h>
#include <stdio.h>
#include "MIDI_Device.h"

    void MIDI_init(void);
    void turnOn(uint8_t note, uint8_t velocity);
    void turnOff(uint8_t note);
    void __attribute__((interrupt, auto_psv)) _U1RXInterrupt(void);
    void attachChannel(Channel*);

#ifdef	__cplusplus
}
#endif

#endif	/* PIC24_MIDI_RECEIVER_H */

