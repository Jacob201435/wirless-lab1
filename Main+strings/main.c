/**
 * main.c - IQ Modulation Lab - Cleaned Version
 * CSE-4377 Lab 1
 */
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "tm4c123gh6pm.h"
#include "clock80.h"
#include "gpio.h"
#include "nvic.h"
#include "spi1.h"
#include "wait.h"
#include "uart0.h"
#include "string.h"

// LDAC Bitband Alias
#define LDAC (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 4*4)))

// DAC Control Constants
#define DAC_A 0x3000  // DAC A (I channel)
#define DAC_B 0xB000  // DAC B (Q channel)

// Constants
#define FCYC 80e6
#define FDAC 20e6
#define LUT_SIZE 256
#define PI 3.14159265358979323846

// Gains and Offsets (from calibration)
#define GAIN_I 1977
#define GAIN_Q 1972
#define OFS_I 2120
#define OFS_Q 2078

// Global variables
int32_t i_value = 2048;
int32_t q_value = 2048;

// Mode control
typedef enum {
    MODE_STATIC,
    MODE_SINE,
    MODE_STREAM
} OperationMode;

OperationMode currentMode = MODE_STATIC;

// Modulation types
typedef enum {
    MOD_BPSK  = 1,
    MOD_QPSK  = 2,
    MOD_8PSK  = 3,
    MOD_16QAM = 4
} ModType;

ModType modType = MOD_QPSK;

// Phase accumulator for sine mode
uint32_t phase_accumulator = 0;
uint32_t phase_increment = 0;

USER_DATA data;

// Sine wave lookup table
int16_t sine_LUT[LUT_SIZE];
int16_t cosine_LUT[LUT_SIZE];

// Symbol stream data
#define STREAM_MAX 1024
uint8_t symbolStream[STREAM_MAX];
uint32_t symbolCount = 0;
uint32_t symbolIndex = 0;

// Symbol rate control
uint32_t symbolRate = 1000;
uint32_t sampleRate = 250000;
uint32_t samplesPerSymbol = 250;
uint32_t sampleCounter = 0;

// Current symbol values
int16_t current_i = 0;
int16_t current_q = 0;

//-----------------------------------------------------------------------------
// Constellation Lookup Tables
//-----------------------------------------------------------------------------

// BPSK: 2 points
static const int16_t bpskI[2] = { +GAIN_I, -GAIN_I };
static const int16_t bpskQ[2] = { 0, 0 };

// QPSK: 4 points (Gray-coded)
static const int16_t qpskI[4] = { +GAIN_I, -GAIN_I, -GAIN_I, +GAIN_I };
static const int16_t qpskQ[4] = { +GAIN_Q, +GAIN_Q, -GAIN_Q, -GAIN_Q };

// 8PSK: 8 points (circle)
static const int16_t psk8I[8] = {
    (int16_t)(+1.000f * GAIN_I),  // 0°
    (int16_t)(+0.707f * GAIN_I),  // 45°
    (int16_t)(+0.000f * GAIN_I),  // 90°
    (int16_t)(-0.707f * GAIN_I),  // 135°
    (int16_t)(-1.000f * GAIN_I),  // 180°
    (int16_t)(-0.707f * GAIN_I),  // 225°
    (int16_t)(+0.000f * GAIN_I),  // 270°
    (int16_t)(+0.707f * GAIN_I)   // 315°
};
static const int16_t psk8Q[8] = {
    (int16_t)(+0.000f * GAIN_Q),  // 0°
    (int16_t)(+0.707f * GAIN_Q),  // 45°
    (int16_t)(+1.000f * GAIN_Q),  // 90°
    (int16_t)(+0.707f * GAIN_Q),  // 135°
    (int16_t)(+0.000f * GAIN_Q),  // 180°
    (int16_t)(-0.707f * GAIN_Q),  // 225°
    (int16_t)(-1.000f * GAIN_Q),  // 270°
    (int16_t)(-0.707f * GAIN_Q)   // 315°
};

// 16QAM: 16 points (4x4 grid)
static const int16_t qam16I[16] = {
    -GAIN_I, -GAIN_I, -GAIN_I, -GAIN_I,
    -GAIN_I/3, -GAIN_I/3, -GAIN_I/3, -GAIN_I/3,
    +GAIN_I/3, +GAIN_I/3, +GAIN_I/3, +GAIN_I/3,
    +GAIN_I, +GAIN_I, +GAIN_I, +GAIN_I
};

static const int16_t qam16Q[16] = {
    -GAIN_Q, -GAIN_Q/3, +GAIN_Q/3, +GAIN_Q,
    -GAIN_Q, -GAIN_Q/3, +GAIN_Q/3, +GAIN_Q,
    -GAIN_Q, -GAIN_Q/3, +GAIN_Q/3, +GAIN_Q,
    -GAIN_Q, -GAIN_Q/3, +GAIN_Q/3, +GAIN_Q
};

//-----------------------------------------------------------------------------
// Update symbol rate calculations
//-----------------------------------------------------------------------------
void updateSymbolRate(void)
{
    if (symbolRate > 0 && symbolRate <= sampleRate)
    {
        samplesPerSymbol = sampleRate / symbolRate;
        if (samplesPerSymbol < 1)
            samplesPerSymbol = 1;
    }
    else
    {
        samplesPerSymbol = 250;
    }
    sampleCounter = 0;
}

//-----------------------------------------------------------------------------
// Generate Sine/Cosine Lookup Tables
//-----------------------------------------------------------------------------
void generateLUT()
{
    int i;
    for (i = 0; i < LUT_SIZE; i++)
    {
        float angle = (2.0 * PI * i) / LUT_SIZE;
        sine_LUT[i] = (int16_t)(sinf(angle) * GAIN_I);
        cosine_LUT[i] = (int16_t)(cosf(angle) * GAIN_Q);
    }
}

//-----------------------------------------------------------------------------
// Convert text to symbol stream
//-----------------------------------------------------------------------------
void textToSymbols(const char* text)
{
    uint8_t bitsPerSymbol;

    switch (modType)
    {
        case MOD_BPSK:  bitsPerSymbol = 1; break;
        case MOD_QPSK:  bitsPerSymbol = 2; break;
        case MOD_8PSK:  bitsPerSymbol = 3; break;
        case MOD_16QAM: bitsPerSymbol = 4; break;
        default: bitsPerSymbol = 1;
    }

    symbolCount = 0;
    uint8_t bitBuffer = 0;
    uint8_t bitCount = 0;

    uint32_t i;
    for (i = 0; text[i] != '\0' && symbolCount < STREAM_MAX; i++)
    {
        uint8_t c = (uint8_t)text[i];

        int8_t bit;
        for (bit = 7; bit >= 0; bit--)
        {
            bitBuffer = (bitBuffer << 1) | ((c >> bit) & 0x01);
            bitCount++;

            if (bitCount == bitsPerSymbol)
            {
                symbolStream[symbolCount++] = bitBuffer;
                bitBuffer = 0;
                bitCount = 0;

                if (symbolCount >= STREAM_MAX)
                    break;
            }
        }
    }

    if (bitCount > 0 && symbolCount < STREAM_MAX)
    {
        bitBuffer <<= (bitsPerSymbol - bitCount);
        symbolStream[symbolCount++] = bitBuffer;
    }

    symbolIndex = 0;
    sampleCounter = 0;
}

//-----------------------------------------------------------------------------
// Timer ISR - Outputs I and Q values to DAC
//-----------------------------------------------------------------------------
void timer1Isr()
{
    uint16_t command;
    int32_t dac_i, dac_q;

    LDAC = 0;
    LDAC = 1;

    if (currentMode == MODE_SINE)
    {
        uint8_t lut_index = phase_accumulator >> 24;
        dac_i = cosine_LUT[lut_index] + OFS_I;
        dac_q = sine_LUT[lut_index] + OFS_Q;
        phase_accumulator += phase_increment;
    }
    else if (currentMode == MODE_STREAM)
    {
        if (sampleCounter == 0)
        {
            if (symbolCount > 0)
            {
                uint8_t sym = symbolStream[symbolIndex];

                switch (modType)
                {
                    case MOD_BPSK:
                        current_i = bpskI[sym & 0x01];
                        current_q = bpskQ[sym & 0x01];
                        break;
                    case MOD_QPSK:
                        current_i = qpskI[sym & 0x03];
                        current_q = qpskQ[sym & 0x03];
                        break;
                    case MOD_8PSK:
                        current_i = psk8I[sym & 0x07];
                        current_q = psk8Q[sym & 0x07];
                        break;
                    case MOD_16QAM:
                        current_i = qam16I[sym & 0x0F];
                        current_q = qam16Q[sym & 0x0F];
                        break;
                    default:
                        current_i = 0;
                        current_q = 0;
                        break;
                }

                symbolIndex++;
                if (symbolIndex >= symbolCount)
                    symbolIndex = 0;
            }
            else
            {
                current_i = 0;
                current_q = 0;
            }

            sampleCounter = samplesPerSymbol;
        }

        sampleCounter--;
        dac_i = current_i + OFS_I;
        dac_q = current_q + OFS_Q;
    }
    else
    {
        dac_i = i_value;
        dac_q = q_value;
    }

    // Clamp to 12-bit range
    if (dac_i < 0) dac_i = 0;
    if (dac_i > 4095) dac_i = 4095;
    if (dac_q < 0) dac_q = 0;
    if (dac_q > 4095) dac_q = 4095;

    // Send to DAC
    command = DAC_A | (dac_i & 0x0FFF);
    SSI1_DR_R = command;
    while (SSI1_SR_R & SSI_SR_BSY);

    command = DAC_B | (dac_q & 0x0FFF);
    SSI1_DR_R = command;
    while (SSI1_SR_R & SSI_SR_BSY);

    TIMER1_ICR_R = TIMER_ICR_TATOCINT;
}

//-----------------------------------------------------------------------------
// Command Handler
//-----------------------------------------------------------------------------
void handleCommand()
{
    char buffer[20];

    getsUart0(&data);
    parseFields(&data);

    if (isCommand(&data, "set", 2))
    {
        currentMode = MODE_STATIC;
        i_value = getFieldInteger(&data, 1);
        q_value = getFieldInteger(&data, 2);

        if (i_value < 0) i_value = 0;
        if (i_value > 4095) i_value = 4095;
        if (q_value < 0) q_value = 0;
        if (q_value > 4095) q_value = 4095;

        putsUart0("\rSet: I=");
        intToStr(i_value, buffer);
        putsUart0(buffer);
        putsUart0(", Q=");
        intToStr(q_value, buffer);
        putsUart0(buffer);
        putsUart0("\n");
    }
    else if (isCommand(&data, "sine", 0))
    {
        currentMode = MODE_SINE;
        phase_accumulator = 0;
        phase_increment = 10000 * 17180;  // 10 kHz
        TIMER1_TAILR_R = 320;

        putsUart0("\rSINE mode: 10 kHz\n");
    }
    else if (isCommand(&data, "static", 0))
    {
        currentMode = MODE_STATIC;
        putsUart0("\rSTATIC mode\n");
    }
    else if (isCommand(&data, "start", 0))
    {
        TIMER1_CTL_R |= TIMER_CTL_TAEN;
        putsUart0("\rStarted\n");
    }
    else if (isCommand(&data, "stop", 0))
    {
        TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
        putsUart0("\rStopped\n");
    }
    else if (isCommand(&data, "freq", 1))
    {
        uint32_t freq = getFieldInteger(&data, 1);

        if (freq < 1 || freq > 100000)
        {
            putsUart0("\rFreq: 1-100000 Hz\n");
            return;
        }

        phase_increment = freq * 17180;
        putsUart0("\rFreq: ");
        intToStr(freq, buffer);
        putsUart0(buffer);
        putsUart0(" Hz\n");
    }
    else if (isCommand(&data, "mod", 1))
    {
        char *m = getFieldString(&data, 1);

        if      (strcmp(m, "bpsk") == 0)  modType = MOD_BPSK;
        else if (strcmp(m, "qpsk") == 0)  modType = MOD_QPSK;
        else if (strcmp(m, "psk8") == 0)  modType = MOD_8PSK;
        else if (strcmp(m, "qam16") == 0) modType = MOD_16QAM;
        else
        {
            putsUart0("\rMod: bpsk|qpsk|psk8|qam16\n");
            return;
        }

        putsUart0("\rMod: ");
        putsUart0(m);
        putsUart0("\n");
    }
    else if (isCommand(&data, "sym", 1))
    {
        uint32_t rate = getFieldInteger(&data, 1);

        if (rate < 1 || rate > 100000)
        {
            putsUart0("\rSym rate: 1-100000\n");
            return;
        }

        symbolRate = rate;
        updateSymbolRate();

        putsUart0("\rSym rate: ");
        intToStr(symbolRate, buffer);
        putsUart0(buffer);
        putsUart0(" sym/s (");
        intToStr(samplesPerSymbol, buffer);
        putsUart0(buffer);
        putsUart0(" samp/sym)\n");
    }
    else if (isCommand(&data, "stream", 1))
    {
        char* text = getFieldString(&data, 1);

        textToSymbols(text);
        currentMode = MODE_STREAM;
        sampleCounter = 0;

        putsUart0("\rStream: ");
        intToStr(symbolCount, buffer);
        putsUart0(buffer);
        putsUart0(" symbols\n");
    }
    else if (isCommand(&data, "constellation", 0))
    {
        currentMode = MODE_STREAM;
        sampleCounter = 0;
        symbolIndex = 0;

        uint8_t numSymbols;
        switch (modType)
        {
            case MOD_BPSK:  numSymbols = 2; break;
            case MOD_QPSK:  numSymbols = 4; break;
            case MOD_8PSK:  numSymbols = 8; break;
            case MOD_16QAM: numSymbols = 16; break;
            default: numSymbols = 4;
        }

        symbolCount = numSymbols;
        uint8_t i;
        for (i = 0; i < numSymbols; i++)
        {
            symbolStream[i] = i;
        }

        putsUart0("\rConstellation: ");
        intToStr(numSymbols, buffer);
        putsUart0(buffer);
        putsUart0(" points\n");
    }
    else if (isCommand(&data, "help", 0))
    {
        putsUart0("\r\n=== CSE-4377 Lab 1 ===\n");
        putsUart0("\rset <I> <Q>   - Set DAC values (0-4095)\n");
        putsUart0("\rsine          - 10 kHz sine wave\n");
        putsUart0("\rfreq <Hz>     - Set sine frequency\n");
        putsUart0("\rstatic        - Static mode\n");
        putsUart0("\rstart / stop  - Start/stop output\n");
        putsUart0("\r\n--- Modulation ---\n");
        putsUart0("\rmod <type>    - bpsk|qpsk|psk8|qam16\n");
        putsUart0("\rsym <rate>    - Symbol rate (symbols/sec)\n");
        putsUart0("\rstream <text> - Stream text data\n");
        putsUart0("\rconstellation - Test all points\n");
        putsUart0("\rhelp          - Show commands\n\n");
    }
    else
    {
        putsUart0("\rUnknown. Type 'help'\n");
    }
}

//-----------------------------------------------------------------------------
// Hardware Initialization
//-----------------------------------------------------------------------------
void initHW()
{
    initSystemClockTo80Mhz();

    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    enablePort(PORTC);
    enablePort(PORTD);

    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
    TIMER1_TAILR_R = 320;  // 250 kHz
    TIMER1_IMR_R = TIMER_IMR_TATOIM;
    NVIC_EN0_R = 1 << (INT_TIMER1A-16);

    selectPinPushPullOutput(PORTC, 4);

    initSpi1(USE_SSI_FSS);
    setSpi1BaudRate(FDAC, 80e6);

    updateSymbolRate();
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------
int main(void)
{
    initHW();
    initUart0();
    setUart0BaudRate(115200, 80e6);
    generateLUT();

    putsUart0("\r\n=== CSE-4377 Lab 1 ===\n");
    putsUart0("\rType 'help' for commands\n\n");

    while (true)
    {
        if (kbhitUart0())
        {
            handleCommand();
        }
    }
}
