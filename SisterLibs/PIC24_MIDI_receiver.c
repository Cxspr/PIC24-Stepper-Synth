///////////////////////////////////////////////////////
//developed by Abe Jaeger Mountain
///////////////////////////////////////////////////////


#include "PIC24_MIDI_receiver.h"
#include "MIDI_Device.h"

volatile uint8_t msgBuf[3];

Channel *chRefs[CHANNEL_LIMIT];
int _num_channels = 0;

void attachChannel(Channel *this)
{
    static int id = 0;
    chRefs[id] = this;
    this->avail = 1;
    id = (id + 1) % CHANNEL_LIMIT;
    _num_channels = (_num_channels >= CHANNEL_LIMIT) ? CHANNEL_LIMIT : _num_channels + 1;
}

//Index of the fundamental frequencies A0-G#1
float freqIndex[12] =
    {27.50, 29.14, 30.87, 32.70, 34.64, 36.71,
     38.89, 41.20, 43.65, 46.25, 49.00, 51.91};
uint16_t getFreq(uint8_t midiNote)
{
    midiNote = midiNote - 20;

    //measure of how many whole octaves above A0 note is
    int octave = (int)(midiNote / 12);

    int fundamentalNote = (int)midiNote % 12;

    float fundamentalFreq = freqIndex[fundamentalNote];

    //CPU-light power2 function
    int i;
    for (i = 0; i < octave; i++)
    {
        fundamentalFreq *= 2;
    }

    return (uint16_t)(fundamentalFreq);
}

void MIDI_init(void)
{

    __builtin_write_OSCCONL(OSCCON & 0xbf); //unlock PPS
    _U1RXR = 4;                             //set RB4 to UART1RX (receive)
    __builtin_write_OSCCONL(OSCCON | 0x40); //lock PPS
    TRISBbits.TRISB4 = 1;                   //set RB0 to input

    U1STA = 0;
    U1MODE = 0;              //clear settings
    U1MODEbits.BRGH = 0;     //normal clock speed
    U1MODEbits.PDSEL = 0b00; //8 bit data, no parity bit
    U1BRG = 31;              //31,250 baud rate
    U1MODEbits.UEN = 0b00;

    U1STAbits.URXISEL = 0b01; //interrupt every char
    IFS0bits.U1RXIF = 0;      //clear IF
    IEC0bits.U1RXIE = 1;      //enable U1RX interrupts

    U1MODEbits.UARTEN = 1; //enable operation

    _NSTDIS = 0; //enable nested interrupts
    _U1RXIP = 7; //set U1RX to highest priority
}

void turnOff(uint8_t note)
{
    uint16_t removeNote = getFreq(note);
    int i;

    for (i = 0; i < _num_channels; i++)
    {
        if (chRefs[i]->freq == removeNote)
        {
            chRefs[i]->setFreq(chRefs[i]->this, 0);
            chRefs[i]->avail = 1; //set to indicated channel available
        }
    }
}

void turnOn(uint8_t note, uint8_t velocity)
{
    //if the velocity is 0, this is actually a turn-off message
    if (velocity == 0)
    {
        turnOff(note);
        return;
    }

    uint16_t newNote = getFreq(note);
    int i, flag = 0;
    for (i = 0; i < _num_channels; i++)
    {
        if (chRefs[i]->freq == newNote)
        {
            flag = 1; //set to indicate that a channel is already playing the frequency
        }
    }

    if (flag != 1)
    {
        for (i = 0; i < _num_channels; i++)
        {
            if (chRefs[i]->avail == 1)
            {
                chRefs[i]->setFreq(chRefs[i]->this, newNote);

                chRefs[i]->avail = 0; //set to indicate channel is taken
                break;
            }
        }
    }
}

enum MidiType
{
    InvalidType = 0x00, ///< For notifying errors
    NoteOff = 0x80,     ///< Channel Message - Note Off
    NoteOn = 0x90,      ///< Channel Message - Note On
    Start = 0xFA,       ///< System Real Time - Start
    Continue = 0xFB,    ///< System Real Time - Continue
    Stop = 0xFC         ///< System Real Time - Stop
};

void __attribute__((interrupt, auto_psv)) _U1RXInterrupt(void)
{
    _U1RXIF = 0;
    static int flag = 0;
    static int byteIdx = 0;

    if (flag == 0)
    {
        msgBuf[0] = U1RXREG;
        if (msgBuf[0] == Stop)
        {
            int i;
            for (i = 0; i < _num_channels; i++)
            {
                chRefs[i]->setFreq(chRefs[i]->this, 0);
                chRefs[i]->avail = 1; //set to indicated channel available
            }
        }
        msgBuf[0] &= 0xf0;

        if (msgBuf[0] == NoteOff)
        {
            flag = 2;
        }
        else if (msgBuf[0] == NoteOn)
        {
            flag = 2;
        }
    }
    else
    {
        msgBuf[++byteIdx] = U1RXREG;
        if (byteIdx == flag)
        {
            if (msgBuf[0] == NoteOff)
            { //stop message
                turnOff(msgBuf[1]);
            }
            else if (msgBuf[0] == NoteOn)
            { //start message
                turnOn(msgBuf[1], msgBuf[2]);
            }
            flag = 0;
            byteIdx = 0;
        }
    }
}