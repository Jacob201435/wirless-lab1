/**
 * main.c - IQ Modulation Lab with RRC Filter
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

// bitband alias for LDAC pin (PC4) - allows single-bit manipulation
#define LDAC (*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 4*4)))

// DAC command format: 0x3000 = DAC A, 0xB000 = DAC B, plus 12-bit value
#define DAC_A 0x3000
#define DAC_B 0xB000

#define PI 3.14159265358979323846

// calibrated offset and gain values for DAC channels
// these center the output at 0V and scale to ±1V
#define GAIN_I 1977  // max deviation from center for I channel
#define GAIN_Q 1972  // max deviation from center for Q channel
#define OFS_I 2120   // DAC value that produces 0V on I channel
#define OFS_Q 2078   // DAC value that produces 0V on Q channel

// static DAC values for raw mode
int32_t i_value = 2048;  // default to mid-scale
int32_t q_value = 2048;

// clipping values
int16_t clip_level = GAIN_I;  // default: no clipping (full range)
bool clipping_enabled = false;

// operating modes
typedef enum
{
    MODE_RAW,     // output static I/Q values
    MODE_SINE,    // generate continuous sine wave
    TWO_TONE,
    MODE_STREAM   // output modulated symbol stream
} OperationMode;

OperationMode currentMode = MODE_RAW;

// digital modulation types
typedef enum
{
    MOD_BPSK = 1,   // 1 bit/symbol, 2 points
    MOD_QPSK = 2,   // 2 bits/symbol, 4 points
    MOD_8PSK = 3,   // 3 bits/symbol, 8 points
    MOD_16QAM = 4   // 4 bits/symbol, 16 points
} ModType;

ModType modType = MOD_BPSK;

// phase accumulator for sine wave generation
// uses 32-bit accumulator, top 8 bits index into LUT
uint32_t phase_accumulator = 0;
uint32_t phase_increment = 0;  // freq * 17180 = (freq * 2^32) / 250000

USER_DATA data;  // UART command parser structure

// sine/cosine lookup tables for waveform generation
#define LUT_SIZE 256
int16_t sine_LUT[LUT_SIZE];    // stores sine values scaled by GAIN_I
int16_t cosine_LUT[LUT_SIZE];  // stores cosine values scaled by GAIN_Q

// symbol stream buffer
#define STREAM_MAX 512  // max 512 symbols (reduced for memory)
uint8_t symbolStream[STREAM_MAX];
uint32_t symbolCount = 0;  // number of symbols in stream
uint32_t symbolIndex = 0;  // current position in stream

// symbol rate timing
uint32_t symbolRate = 1000;           // symbols per second
uint32_t samplesPerSymbol = 250; // 250000 / 1000 = 250 samples to hold each symbol
uint32_t sampleCounter = 0;           // countdown timer for symbol hold

// current symbol being output
int16_t current_i = 0;
int16_t current_q = 0;

//-----------------------------------------------------------------------------
// constellation lookup tables - pre-calculated I/Q values for each symbol
//-----------------------------------------------------------------------------

// BPSK: 2 points on I axis (±GAIN)
static const int16_t bpskI[2] = { GAIN_I, -GAIN_I };
static const int16_t bpskQ[2] = { 0, 0 };

// QPSK: 4 points in corners (Gray coded)
static const int16_t qpskI[4] = { GAIN_I, -GAIN_I, -GAIN_I, GAIN_I };
static const int16_t qpskQ[4] = { GAIN_Q, GAIN_Q, -GAIN_Q, -GAIN_Q };

// 8PSK: 8 points in circle, 45° apart
static const int16_t psk8I[8] = { GAIN_I, GAIN_I * 0.707, 0, -GAIN_I * 0.707,
                                  -GAIN_I, -GAIN_I * 0.707, 0, GAIN_I * 0.707 };
static const int16_t psk8Q[8] = { 0, GAIN_Q * 0.707, GAIN_Q, GAIN_Q * 0.707, 0,
                                  -GAIN_Q * 0.707, -GAIN_Q, -GAIN_Q * 0.707 };

// 16-QAM: 4x4 grid with levels at -GAIN, -GAIN/3, +GAIN/3, +GAIN
static const int16_t qam16I[16] = { -GAIN_I, -GAIN_I, -GAIN_I, -GAIN_I, -GAIN_I
                                            / 3,
                                    -GAIN_I / 3, -GAIN_I / 3, -GAIN_I / 3,
                                    GAIN_I / 3,
                                    GAIN_I / 3, GAIN_I / 3, GAIN_I / 3,
                                    GAIN_I,
                                    GAIN_I, GAIN_I, GAIN_I };

static const int16_t qam16Q[16] = { -GAIN_Q, -GAIN_Q / 3, GAIN_Q / 3, GAIN_Q,
                                    -GAIN_Q, -GAIN_Q / 3, GAIN_Q / 3, GAIN_Q,
                                    -GAIN_Q, -GAIN_Q / 3, GAIN_Q / 3, GAIN_Q,
                                    -GAIN_Q, -GAIN_Q / 3, GAIN_Q / 3, GAIN_Q };

//-----------------------------------------------------------------------------
// RRC filter - root raised cosine for bandwidth limiting
//-----------------------------------------------------------------------------
#define H_GAIN 65536  // Q16 fixed-point scale factor (2^16)
bool rrc_enabled = false;

// 33-tap RRC filter coefficients, alpha=0.25, generated with rcosdesign(0.25,8,4)
const int16_t rrcFilter[33] = { (0.0106 * H_GAIN), (0.0058 * H_GAIN), (-0.0097
                                        * H_GAIN),
                                (-0.0214 * H_GAIN), (-0.0188 * H_GAIN), (0.0030
                                        * H_GAIN),
                                (0.0327 * H_GAIN), (0.0471 * H_GAIN), (0.0265
                                        * H_GAIN),
                                (-0.0275 * H_GAIN), (-0.0852 * H_GAIN), (-0.0994
                                        * H_GAIN),
                                (-0.0321 * H_GAIN), (0.1190 * H_GAIN), (0.3110
                                        * H_GAIN),
                                (0.4717 * H_GAIN), (0.5343 * H_GAIN), // center tap
                                (0.4717 * H_GAIN), (0.3110 * H_GAIN), (0.1190
                                        * H_GAIN),
                                (-0.0321 * H_GAIN), (-0.0994 * H_GAIN), (-0.0852
                                        * H_GAIN),
                                (-0.0275 * H_GAIN), (0.0265 * H_GAIN), (0.0471
                                        * H_GAIN),
                                (0.0327 * H_GAIN), (0.0030 * H_GAIN), (-0.0188
                                        * H_GAIN),
                                (-0.0214 * H_GAIN), (-0.0097 * H_GAIN), (0.0058
                                        * H_GAIN),
                                (0.0106 * H_GAIN) };

// buffers for RRC filtering - hold upsampled and filtered data
#define UPSAMPLED_MAX 2048  // 512 symbols * 4 (upsampling factor)
static int16_t upsampledData_I[UPSAMPLED_MAX];  // symbols with 3 zeros inserted
static int16_t upsampledData_Q[UPSAMPLED_MAX];
static int16_t filteredData_I[UPSAMPLED_MAX];   // convolved output
static int16_t filteredData_Q[UPSAMPLED_MAX];
uint32_t filteredSize = 0;

//-----------------------------------------------------------------------------
// map symbol value to constellation point I/Q coordinates
//-----------------------------------------------------------------------------
void getConstellation(uint8_t sym, int16_t *out_i, int16_t *out_q)
{
    switch (modType)
    {
    case MOD_BPSK:
        *out_i = bpskI[sym & 0x01];  // use only bit 0
        *out_q = bpskQ[sym & 0x01];
        break;
    case MOD_QPSK:
        *out_i = qpskI[sym & 0x03];  // use bits 0-1
        *out_q = qpskQ[sym & 0x03];
        break;
    case MOD_8PSK:
        *out_i = psk8I[sym & 0x07];  // use bits 0-2
        *out_q = psk8Q[sym & 0x07];
        break;
    case MOD_16QAM:
        *out_i = qam16I[sym & 0x0F];  // use bits 0-3
        *out_q = qam16Q[sym & 0x0F];
        break;
    default:
        *out_i = 0;
        *out_q = 0;
    }
}

//-----------------------------------------------------------------------------
// apply RRC filter to symbol stream
// upsample by 4x (insert 3 zeros between symbols)
//  convolve with 33-tap filter
// store filtered samples for ISR to output
//-----------------------------------------------------------------------------
void applyRRCFilter(void)
{
    // clear all buffers
    memset(upsampledData_I, 0, sizeof(upsampledData_I));
    memset(upsampledData_Q, 0, sizeof(upsampledData_Q));
    memset(filteredData_I, 0, sizeof(filteredData_I));
    memset(filteredData_Q, 0, sizeof(filteredData_Q));

    // upsample: place each symbol at index i*4, leaving 3 zeros after it
    uint32_t i;
    for (i = 0; i < symbolCount && i < STREAM_MAX; i++)
    {
        int16_t sym_i, sym_q;
        getConstellation(symbolStream[i], &sym_i, &sym_q);

        uint32_t idx = i * 4;
        if (idx < UPSAMPLED_MAX)
        {
            upsampledData_I[idx] = sym_i;
            upsampledData_Q[idx] = sym_q;
        }
    }

    uint32_t upsampledSize = symbolCount * 4;
    if (upsampledSize > UPSAMPLED_MAX)
        upsampledSize = UPSAMPLED_MAX;

    // convolution: each output = weighted sum of inputs
    uint32_t j;
    for (i = 0; i < upsampledSize; i++)
    {
        int64_t sum_i = 0;
        int64_t sum_q = 0;

        // multiply input samples by filter taps and accumulate
        for (j = 0; j < 33 && j <= i; j++)
        {
            sum_i += (int64_t) upsampledData_I[i - j] * rrcFilter[j];
            sum_q += (int64_t) upsampledData_Q[i - j] * rrcFilter[j];
        }

        // scale back from Q16 fixed-point (divide by 65536)
        filteredData_I[i] = (int16_t) ((sum_i) / H_GAIN);
        filteredData_Q[i] = (int16_t) ((sum_q) / H_GAIN);
    }

    filteredSize = upsampledSize;
}

//-----------------------------------------------------------------------------
// update timing based on symbol rate
// calculates how many timer ticks to hold each symbol
//-----------------------------------------------------------------------------
void updateSymbolRate(void)
{
    if (symbolRate > 0 && symbolRate <= 250000)
    {
        samplesPerSymbol = 250000 / symbolRate;  // 250 kHz sample rate
        if (samplesPerSymbol < 1)
            samplesPerSymbol = 1;
    }
    else
    {
        samplesPerSymbol = 250;  // default
    }
    sampleCounter = 0;
}

//-----------------------------------------------------------------------------
// generate sine and cosine lookup tables
// called once at startup
//-----------------------------------------------------------------------------
void generateLUT()
{
    int i;
    for (i = 0; i < LUT_SIZE; i++)
    {
        float angle = (2.0f * PI * i) / LUT_SIZE;  // 0 to 2pi in 256 steps
        sine_LUT[i] = (int16_t) (sinf(angle) * GAIN_I);
        cosine_LUT[i] = (int16_t) (cosf(angle) * GAIN_Q);
    }
}

//-----------------------------------------------------------------------------
// convert ASCII text to symbol stream
// packs bits into symbols based on modulation type
//-----------------------------------------------------------------------------
void textToSymbols(const char *text)
{
    // determine bits per symbol for current modulation
    uint8_t bitsPerSymbol;
    switch (modType)
    {
    case MOD_BPSK:
        bitsPerSymbol = 1;
        break;
    case MOD_QPSK:
        bitsPerSymbol = 2;
        break;
    case MOD_8PSK:
        bitsPerSymbol = 3;
        break;
    case MOD_16QAM:
        bitsPerSymbol = 4;
        break;
    default:
        bitsPerSymbol = 1;
    }

    symbolCount = 0;
    uint8_t bitBuffer = 0;
    uint8_t bitCount = 0;

    // process each character in the string
    uint32_t i;
    for (i = 0; text[i] != '\0' && symbolCount < STREAM_MAX; i++)
    {
        uint8_t c = (uint8_t) text[i];

        // extract bits MSB first
        int8_t bit;
        for (bit = 7; bit >= 0; bit--)
        {
            bitBuffer = (bitBuffer << 1) | ((c >> bit) & 0x01);
            bitCount++;

            // when we have enough bits, store as symbol
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

    // pad final partial symbol if needed
    if (bitCount > 0 && symbolCount < STREAM_MAX)
    {
        bitBuffer <<= (bitsPerSymbol - bitCount);
        symbolStream[symbolCount++] = bitBuffer;
    }

    symbolIndex = 0;
    sampleCounter = 0;
}

//-----------------------------------------------------------------------------
// timer ISR - called at 250 kHz to output samples to DAC
// handles three modes: raw static values, sine wave, or symbol stream
//-----------------------------------------------------------------------------
void timer1Isr()
{
    int32_t dac_i, dac_q;

    // toggle LDAC to latch new DAC values
    LDAC = 0;
    LDAC = 1;

    if (currentMode == MODE_SINE)
    {
        // sine mode: use phase accumulator to index into LUT
                uint8_t lut_index = phase_accumulator >> 24;  // extract top 8 bits
                int16_t raw_i = cosine_LUT[lut_index];
                int16_t raw_q = sine_LUT[lut_index];

                // apply clipping if enabled
                if (clipping_enabled)
                {
                    if (raw_i > clip_level) raw_i = clip_level;
                    if (raw_i < -clip_level) raw_i = -clip_level;
                    if (raw_q > clip_level) raw_q = clip_level;
                    if (raw_q < -clip_level) raw_q = -clip_level;
                }

                dac_i = raw_i + OFS_I;
                dac_q = raw_q + OFS_Q;
                phase_accumulator += phase_increment;  // auto-wraps at 2^32

    }
    else if (currentMode == TWO_TONE)
        {
        // two-tone mode: I=cos, Q=0 for double sideband
                uint8_t lut_index = phase_accumulator >> 24;
                int16_t raw_i = cosine_LUT[lut_index];

                // apply clipping if enabled
                if (clipping_enabled)
                {
                    if (raw_i > clip_level) raw_i = clip_level;
                    if (raw_i < -clip_level) raw_i = -clip_level;
                }

                dac_i = raw_i + OFS_I;
                dac_q = OFS_Q;  // 0V output
                phase_accumulator += phase_increment;
        }
    else if (currentMode == MODE_STREAM)
    {
        // symbol stream mode: hold each symbol for samplesPerSymbol ticks
        if (sampleCounter == 0)
        {
            if (rrc_enabled && filteredSize > 0)
            {
                // output pre-filtered samples (no holding, already upsampled)
                current_i = filteredData_I[symbolIndex];
                current_q = filteredData_Q[symbolIndex];
                symbolIndex++;
                if (symbolIndex >= filteredSize)
                    symbolIndex = 0;
            }
            else if (symbolCount > 0)
            {
                // output unfiltered constellation points (with holding)
                getConstellation(symbolStream[symbolIndex], &current_i,
                                 &current_q);
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
        // raw mode: output static values
        dac_i = i_value;
        dac_q = q_value;
    }

    // clamp to 12-bit DAC range (0-4095)
    if (dac_i < 0)
        dac_i = 0;
    if (dac_i > 4095)
        dac_i = 4095;
    if (dac_q < 0)
        dac_q = 0;
    if (dac_q > 4095)
        dac_q = 4095;

    // send values to DAC via SPI
    SSI1_DR_R = DAC_A | (dac_i & 0x0FFF);  // I channel
    while (SSI1_SR_R & SSI_SR_BSY)
        ;
    SSI1_DR_R = DAC_B | (dac_q & 0x0FFF);  // Q channel

    TIMER1_ICR_R = TIMER_ICR_TATOCINT;  // clear interrupt flag
}

//-----------------------------------------------------------------------------
// command handler - processes UART commands
//-----------------------------------------------------------------------------
void handleCommand()
{
    char buffer[20];

    getsUart0(&data);
    parseFields(&data);

    if (isCommand(&data, "set", 2))
    {
        // set raw I/Q DAC values
        i_value = getFieldInteger(&data, 1);
        q_value = getFieldInteger(&data, 2);
        if (i_value < 0)
            i_value = 0;
        if (i_value > 4095)
            i_value = 4095;
        if (q_value < 0)
            q_value = 0;
        if (q_value > 4095)
            q_value = 4095;
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
        // generate 10 kHz sine wave
        currentMode = MODE_SINE;
        phase_accumulator = 0;
        phase_increment = 10000 * 17180;  // 10000 * (2^32 / 250000)
        TIMER1_TAILR_R = 320;  // 80 MHz / 320 = 250 kHz
        putsUart0("\rSINE 10kHz\n");
    }
    else if (isCommand(&data, "twotone", 0))
    {
        // Two tone: I=cos, Q=0 (both sidebands)
        currentMode = TWO_TONE;
        phase_accumulator = 0;
        phase_increment = 10000 * 17180;  // 10 kHz
        putsUart0("\rTwo tone (double sideband)\n");
    }
    else if (isCommand(&data, "clip", 1))
    {
        // Set clipping level as percentage (0-100%)
        uint32_t percent = getFieldInteger(&data, 1);

        if (percent > 0 && percent <= 100)
        {
            clip_level = (GAIN_I * percent) / 100;
            clipping_enabled = true;

            intToStr(percent, buffer);
            putsUart0("\rClipping at ");
            putsUart0(buffer);
            putsUart0("%\n");
        }
        else
        {
            putsUart0("\rClip: 1-100%\n");
        }
    }
    else if (isCommand(&data, "clipoff", 0))
    {
        clipping_enabled = false;
        clip_level = GAIN_I;
        putsUart0("\rClipping OFF\n");
    }
    else if (isCommand(&data, "start", 0))
    {
        // enable timer to start outputting
        TIMER1_CTL_R |= TIMER_CTL_TAEN;
        putsUart0("\rON\n");
    }
    else if (isCommand(&data, "stop", 0))
    {
        // disable timer to stop outputting
        TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
        putsUart0("\rOFF\n");
    }
    else if (isCommand(&data, "freq", 1))
    {
        // set sine wave frequency
        uint32_t freq = getFieldInteger(&data, 1);
        if (freq > 0 && freq <= 100000)
        {
            phase_increment = freq * 17180;  // freq * (2^32 / 250000)
            putsUart0("\rFreq: ");
            intToStr(freq, buffer);
            putsUart0(buffer);
            putsUart0(" Hz\n");
        }
    }
    else if (isCommand(&data, "mod", 1))
    {
        // select modulation type
        char *m = getFieldString(&data, 1);
        if (strcmp(m, "bpsk") == 0)
            modType = MOD_BPSK;
        else if (strcmp(m, "qpsk") == 0)
            modType = MOD_QPSK;
        else if (strcmp(m, "psk8") == 0)
            modType = MOD_8PSK;
        else if (strcmp(m, "qam16") == 0)
            modType = MOD_16QAM;
        putsUart0("\rMod: ");
        putsUart0(m);
        putsUart0("\n");
    }
    else if (isCommand(&data, "sym", 1))
    {
        // set symbol rate (symbols per second)
        symbolRate = getFieldInteger(&data, 1);
        if (symbolRate > 0 && symbolRate <= 100000)
        {
            updateSymbolRate();
            intToStr(symbolRate, buffer);
            putsUart0("\rSymRate: ");
            putsUart0(buffer);
            putsUart0("\n");
        }
    }
    else if (isCommand(&data, "stream", 1))
    {
        // convert text to symbols and prepare for streaming
        char *text = getFieldString(&data, 1);
        textToSymbols(text);
        currentMode = MODE_STREAM;
        sampleCounter = 0;
        symbolIndex = 0;

        // apply RRC filter if enabled
        if (rrc_enabled)
        {
            applyRRCFilter();
            intToStr(filteredSize, buffer);
            putsUart0("\rFiltered: ");
            putsUart0(buffer);
            putsUart0(" samp\n");
        }
        else
        {
            intToStr(symbolCount, buffer);
            putsUart0("\rSyms: ");
            putsUart0(buffer);
            putsUart0("\n");
        }
    }
    else if (isCommand(&data, "constellation", 0))
    {
        // load all constellation points (for testing)
        currentMode = MODE_STREAM;
        sampleCounter = 0;
        symbolIndex = 0;

        uint8_t numSymbols;
        switch (modType)
        {
        case MOD_BPSK:
            numSymbols = 2;
            break;
        case MOD_QPSK:
            numSymbols = 4;
            break;
        case MOD_8PSK:
            numSymbols = 8;
            break;
        case MOD_16QAM:
            numSymbols = 16;
            break;
        default:
            numSymbols = 4;
        }

        symbolCount = numSymbols;
        uint8_t i;
        for (i = 0; i < numSymbols; i++)
            symbolStream[i] = i;  // symbols 0,1,2,...,N-1

        putsUart0("\rConstellation: ");
        intToStr(numSymbols, buffer);
        putsUart0(buffer);
        putsUart0(" points\n");
    }
    else if (isCommand(&data, "rrc", 1))
    {
        // enable/disable RRC filter
        char *state = getFieldString(&data, 1);

        if (strcmp(state, "on") == 0)
        {
            rrc_enabled = true;
            // re-filter existing data if present
            if (symbolCount > 0)
            {
                applyRRCFilter();
                symbolIndex = 0;
            }
            putsUart0("\rRRC ON\n");
        }
        else if (strcmp(state, "off") == 0)
        {
            rrc_enabled = false;
            symbolIndex = 0;
            putsUart0("\rRRC OFF\n");
        }
        else
        {
            putsUart0("\rUse: rrc on|off\n");
        }
    }
    else if (isCommand(&data, "help", 0))
    {
        // show available commands
        putsUart0("\r\n=== CSE-4377 Lab 1 ===\n");
        putsUart0("\rset <I> <Q>   - Set DAC values (0-4095)\n");
        putsUart0("\rsine          - 10 kHz sine wave\n");
        putsUart0("\rfreq <Hz>     - Set sine frequency\n");
        putsUart0("\rstart / stop  - Start/stop output\n");
        putsUart0("\r\n--- Modulation ---\n");
        putsUart0("\rmod <type>    - bpsk|qpsk|psk8|qam16\n");
        putsUart0("\rsym <rate>    - Symbol rate (symbols/sec)\n");
        putsUart0("\rstream <text> - Stream text data\n");
        putsUart0("\rconstellation - Test all points\n");
        putsUart0("\rrrc on|off    - Enable/disable RRC filter\n");
        putsUart0("\rhelp          - Show commands\n\n");
    }
    else
    {
        putsUart0("\rUnknown. Type 'help'\n");
    }
}

//-----------------------------------------------------------------------------
// hardware initialization
//-----------------------------------------------------------------------------
void initHW()
{
    initSystemClockTo80Mhz();
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    enablePort(PORTC);
    enablePort(PORTD);

    // configure timer1 for periodic interrupts at 250 kHz
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;
    TIMER1_TAILR_R = 320;  // 80 MHz / 320 = 250 kHz sample rate
    TIMER1_IMR_R = TIMER_IMR_TATOIM;
    NVIC_EN0_R = 1 << (INT_TIMER1A - 16);

    // configure LDAC pin and SPI
    selectPinPushPullOutput(PORTC, 4);
    initSpi1(USE_SSI_FSS);
    setSpi1BaudRate(20e6, 80e6);  // 20 MHz SPI clock
    updateSymbolRate();
}

//-----------------------------------------------------------------------------
// main - initialize and run command loop
//-----------------------------------------------------------------------------
int main(void)
{
    initHW();
    initUart0();
    setUart0BaudRate(115200, 80e6);
    generateLUT();

    putsUart0("\rCSE-4377 Lab1\n");

    while (true)
    {
        if (kbhitUart0())
        {
            handleCommand();
        }
    }
}
