#include "MIDI_Synth.h"


///////////////////////////////////////////////////////
//sourced from midi device library
//developed by Cxspr
///////////////////////////////////////////////////////


/**
 * @brief simple implementation of log10() from math.h to make a log base-2 function
 * 
 * @param x the x argument of log2(x)
 * @return the result of the log2 operation 
 */
double log2(double x) {
    double log;
    log = log10(x) / log10(2);
    return log;
}

/**
 * @brief sets the pin attached to the provided channel to the provided pin value, if the pin value had previously been set
 *        it resets the state of the previous pin value
 * 
 * @param this a pointer to the channel object
 * @param pin  the pin numver to attach to the channel
 */
void setPin(Channel *this, uint16_t pin) {
    static char first_time = 0;

    int idx = pin / 2;
    int mask = 18 + this->ocr;

    if (this->ocr < 2) { //PPS pin assignment for channels using output compare
        __builtin_write_OSCCONL(OSCCON & 0xBF); //unlock PPS

        if (!(first_time == 0)) {
            //reset prev rpor
            int idxp = this->pin / 2;
            volatile uint16_t *RPORp = _RPOR_[idxp];
            if ((this->pin / 2 % 2) == 1) { //reset MSB
                *RPORp &= 0x00FF;
            } else {
                *RPORp &= 0xFF00; //reset LSB
            }
        }

        volatile uint16_t *RPOR = _RPOR_[idx];
        if (pin % 2 == 1) { //do operations in MSB
            *RPOR &= 0x00FF; //reset MSB
            *RPOR |= (mask << 8);
        } else { //do operations in LSB
            *RPOR &= 0xFF00; //reset LSB
            *RPOR |= mask;
        }
        __builtin_write_OSCCONL(OSCCON | 0x40); //lock PPS
    } else { //pin assignment for channels using timer interrupt modulo toggle
        if (!(first_time == 0)) TRISB &= ~(1 << this->pin); //reverse previous pin's configuration
        TRISB |= (1 << pin); //configure newly declared pin for use
    }
    this->pin = pin; //store pin
    first_time = 1;
}

volatile Channel *_channel[CHANNEL_LIMIT]; //a global storage array for the channel object pointers
volatile int num_channels = 0;

/**
 * @brief initializes a channel object on the specified pin
 *        this method automatically handles pin assignment, timer attachment, and pointer function definition
 * 
 * @param channel a pointer to an externally declared channel object
 * @param pin the pin to attach the channel to
 */
void initChannel(Channel *channel, uint16_t pin) {
    static uint16_t ocr = 0;
    //    Channel channel = {.initFlag = 1}; //deprecated
    if (!(0 <= pin && pin <= 15) || (pin == 4)) { //early escape if pin value is invalid
        return;
    }
    channel->ocr = ocr; 
    channel->this = channel; //store recursive address
    if (ocr < 2) setPin(channel, pin); //dynamically preform PPS
    channel->timer = *channelTimers[ocr]; //attach timer
    if (ocr < 2) *channel->timer.OCxCON = ((ocr << 3) | 0b110); //attach correct timer and set to PWM w/o faults

    //associate pointer functions
    channel->stop = &stop;
    channel->start = &start;
    channel->setPin = &setPin;
    channel->setFreq = &setFreq;

    if (ocr == 2) { //enable timer interrupts for channel 3
        _T4IF = 0;
        _T4IE = 1;
    }
    if (ocr == 3) { //enable timer interrupts for channel 3
        _T5IF = 0;
        _T5IE = 1;
    }
    
    // attachChannel(channel);
    this->avail = 1;
    _channel[ocr] = channel;
    channel->avail = 1;//an initialized channel should always be available 'out of the box'
    
    num_channels = (num_channels >= CHANNEL_LIMIT) ? CHANNEL_LIMIT : num_channels + 1;
    ocr = (ocr + 1) % CHANNEL_LIMIT; //recursively increment
    return;
}

/**
 * @brief this method stops any playback on the channel
 * 
 * @param this a pointer to the channel to be modified
 */
void stop(Channel *this) {
    *this->timer.TxCON &= ~(0x8000); //turn off associated timer
    if (this->ocr < 2) {
        *this->timer.OCxCON &= ~(0x2000); //turn off associated output compare
    } else {
        LATB &= ~(1 << this->pin);
    }
}
/**
 * @brief this method starts playback on a channel
 * 
 * @param this a pointer to the channel to be modified
 */
void start(Channel *this) {
    *this->timer.TMRx = 0; //reset TMR count
    *this->timer.TxCON |= 0x8000; //turn on timer
    if (this->ocr < 2) {
        *this->timer.OCxCON |= 0x2000; //turn on output compare
    }
}

/**
 * @brief set a channel to play a specified frequency, if the frequency is zero, the playback will be stopped to avoid
 *        divide by zero error. if not zero, playback will auto start with the given frequency
 * 
 * @param this a pointer to the channel to be modified
 * @param freq the frequency to play
 */
void setFreq(Channel *this, uint16_t freq) {
    int freqMod = 1;
    if (freq == 0) { //if frequency is set to zero, don't play anything
        this->freq = 0;
        this->stop(this); //would inevitably cause a divide by zero error if let through
        return;
    }
    if (this->ocr >= 2) { //unique freq modification for
        freqMod = 4;
    }

    const float tcy = 0.0000000625;
    uint32_t eqFreq = freq * freqMod; //added to account for freq > ~16,000 on non OCR channels
    //on non OCR channels freqs > ~16,000 result in a equation frequency
    //greater than the maximum value for a 16 bit integer
    float per = 1.0 / eqFreq;
    double cycles = per / tcy;
    float bitsNeeded = log2(cycles);
    int preS;
    *this->timer.TxCON &= (0xffff - 0x30);
    if (bitsNeeded > 22) {
        *this->timer.TxCON |= 0x30; //1:256
        preS = 256;
    } else if (bitsNeeded > 19) {
        *this->timer.TxCON |= 0x20; //1:64
        preS = 64;
    } else if (bitsNeeded > 16) {
        *this->timer.TxCON |= 0x10; //1:8
        preS = 8;
    } else {
        *this->timer.TxCON |= 0x0; //1:1
        preS = 1;
    }

    float tcym = tcy * preS;
    uint16_t period = (per / tcym);
    this->freq = freq; //passes desired frequency to all channels [instead of modified freq on non OCR channels]
    *this->timer.PRx = (period - 1);
    *this->timer.OCxRS = period / 2;
    this->start(this);
}

/**
 * @brief a timer interrupt definition for a 3rd channel modulo toggle 
 */
void __attribute__((__interrupt__, __auto_psv__)) _T4Interrupt(void) {
    _T4IF = 0;
    static int toggle = 1;
    int mask = toggle << _channel[2]->pin;
    LATB ^= mask;
    toggle = (toggle + 1) % 2;
}
/**
 * @brief a timer interrupt definition for a 4th channel modulo toggle 
 */
void __attribute__((__interrupt__, __auto_psv__)) _T5Interrupt(void) {
    _T5IF = 0;
    static int toggle = 1;
    int mask = toggle << _channel[3]->pin;
    LATB ^= mask;
    toggle = (toggle + 1) % 2;
}


///////////////////////////////////////////////////////
//sourced from midi reception library
//developed by Abe Jaeger Mountain
///////////////////////////////////////////////////////


volatile uint8_t msgBuf[3];

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
        if (_channel[i]->freq == removeNote)
        {
            _channel[i]->setFreq(_channel[i]->this, 0);
            _channel[i]->avail = 1; //set to indicated channel available
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
    for (i = 0; i < num_channels; i++)
    {
        if (_channel[i]->freq == newNote)
        {
            flag = 1; //set to indicate that a channel is already playing the frequency
        }
    }

    if (flag != 1)
    {
        for (i = 0; i < num_channels; i++)
        {
            if (_channel[i]->avail == 1)
            {
                _channel[i]->setFreq(_channel[i]->this, newNote);

                _channel[i]->avail = 0; //set to indicate channel is taken
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
            for (i = 0; i < num_channels; i++)
            {
                chRefs[i]->setFreq(_channel[i]->this, 0);
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
