#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>

/* TI Drivers */
#include <ti/drivers/GPIO.h>

/*
 * Tell DriverLib exactly which MSP432E4 device
 * we are compiling for.
 *
 * This MUST appear before driverlib.h.
 */
#ifndef __MSP432E401Y__
#define __MSP432E401Y__
#endif

#include <ti/devices/msp432e4/driverlib/driverlib.h>

/* Driver configuration */
#include "ti_drivers_config.h"


/* ============================================================
 * SYSTEM CONFIGURATION
 * ============================================================
 */

#define SYSTEM_CLOCK_HZ          120000000U


/* ============================================================
 * AUDIO CONFIGURATION
 * ============================================================
 */

#define DAC_SPI_BIT_RATE_HZ      2000000U

/*
 * DAC8311 midpoint.
 *
 * DAC range = 0 ... 16383
 */
#define DAC_IDLE_CODE            8192U


/*
 * Audio sample rate:
 *
 * 120 MHz / 6000 = 20,000 samples/second.
 */
#define AUDIO_SAMPLE_RATE_HZ     20000U
#define AUDIO_TIMER_COUNTS       6000U


#define SINE_TABLE_SIZE          256U
#define SINE_TABLE_BITS          8U


/* ============================================================
 * MIDI SUPPORT RANGE
 * ============================================================
 *
 * For now support:
 *
 * MIDI 48 = C3
 * through
 * MIDI 84 = C6
 *
 * This is plenty for our simple monophonic tests.
 */

#define MIDI_NOTE_MIN            48U
#define MIDI_NOTE_MAX            84U
#define MIDI_NOTE_NONE           0xFFU


/* ============================================================
 * UART / MIDI FILE BUFFER
 * ============================================================
 *
 * Deliverable 3: reserve room for a complete MIDI payload up to 16 KiB.
 * The separate parsed-note array below has its own capacity guard.
 */

/* SMID v2: byte 5 is opcode (0=MIDI, 1=STOP); v1 MIDI remains accepted.
 * STOP has zero length/CRC and is recognized only at a header boundary.
 * MIDI uses a 16-byte header, then an acknowledged raw MIDI payload.
 * Multi-byte transport fields are little-endian; MIDI itself is unchanged.
 * Keep these limits coordinated with newmiditst.py.
 */
#define MAX_MIDI_FILE_SIZE       16384U
#define SMID_HEADER_SIZE         16U
#define SMID_PROTOCOL_VERSION    2U
#define SMID_LEGACY_VERSION      1U
#define SMID_COMMAND_MIDI        0U
#define SMID_COMMAND_STOP        1U
#define RX_HEADER_TIMEOUT_MS     1000U
#define RX_PAYLOAD_IDLE_MS       2000U
#define RX_PAYLOAD_TOTAL_MS      10000U
#define RX_RECOVERY_QUIET_MS      250U
#define RX_SERVICE_BYTE_BUDGET   64U

static uint8_t fileBuffer[MAX_MIDI_FILE_SIZE];
static uint8_t headerBytes[SMID_HEADER_SIZE];
static uint32_t headerBytesReceived = 0U;
static uint32_t magicBytesMatched = 0U;
static uint32_t expectedLength = 0U;
static uint32_t bytesReceived = 0U;
static uint32_t expectedCrc32 = 0U;
static uint32_t headerStartedMs = 0U;
static uint32_t payloadStartedMs = 0U;
static uint32_t lastRxByteMs = 0U;

typedef enum
{
    WAIT_MAGIC = 0,
    READ_HEADER,
    READ_PAYLOAD,
    VALIDATE,
    RECOVER
} ReceiverState;

static ReceiverState receiverState = WAIT_MAGIC;

/* Timer4 supplies time even when Timer2 (audio) is stopped. */
static volatile uint32_t g_transferMilliseconds = 0U;

/* Optional CCS watch variables. These describe the most recent attempt. */
volatile uint32_t g_transferSuccessCount = 0U;
volatile uint32_t g_transferErrorCount = 0U;
volatile uint32_t g_lastTransferLength = 0U;
volatile uint32_t g_lastExpectedCrc32 = 0U;
volatile uint32_t g_lastCalculatedCrc32 = 0U;
static bool playbackCompletionPending = false;



/* ============================================================
 * SINE-WAVE TABLE
 * ============================================================
 */

/* 256 samples of one cycle: round(8192 + 2500*sin(2*pi*i/256)).
 * Precomputed constants: no runtime floating-point sine calculation.
 * The audio ISR retains its existing x3 gain and 20 kHz update rate.
 */
static const uint16_t sineTable[SINE_TABLE_SIZE] =
{
    8192, 8253, 8315, 8376, 8437, 8498, 8559, 8619,
    8680, 8740, 8799, 8859, 8918, 8976, 9034, 9092,
    9149, 9205, 9261, 9316, 9370, 9424, 9477, 9529,
    9581, 9632, 9681, 9730, 9778, 9825, 9871, 9916,
    9960, 10003, 10044, 10085, 10125, 10163, 10200, 10236,
    10271, 10304, 10336, 10367, 10397, 10425, 10452, 10478,
    10502, 10524, 10546, 10566, 10584, 10601, 10617, 10631,
    10644, 10655, 10665, 10673, 10680, 10685, 10689, 10691,
    10692, 10691, 10689, 10685, 10680, 10673, 10665, 10655,
    10644, 10631, 10617, 10601, 10584, 10566, 10546, 10524,
    10502, 10478, 10452, 10425, 10397, 10367, 10336, 10304,
    10271, 10236, 10200, 10163, 10125, 10085, 10044, 10003,
    9960, 9916, 9871, 9825, 9778, 9730, 9681, 9632,
    9581, 9529, 9477, 9424, 9370, 9316, 9261, 9205,
    9149, 9092, 9034, 8976, 8918, 8859, 8799, 8740,
    8680, 8619, 8559, 8498, 8437, 8376, 8315, 8253,
    8192, 8131, 8069, 8008, 7947, 7886, 7825, 7765,
    7704, 7644, 7585, 7525, 7466, 7408, 7350, 7292,
    7235, 7179, 7123, 7068, 7014, 6960, 6907, 6855,
    6803, 6752, 6703, 6654, 6606, 6559, 6513, 6468,
    6424, 6381, 6340, 6299, 6259, 6221, 6184, 6148,
    6113, 6080, 6048, 6017, 5987, 5959, 5932, 5906,
    5882, 5860, 5838, 5818, 5800, 5783, 5767, 5753,
    5740, 5729, 5719, 5711, 5704, 5699, 5695, 5693,
    5692, 5693, 5695, 5699, 5704, 5711, 5719, 5729,
    5740, 5753, 5767, 5783, 5800, 5818, 5838, 5860,
    5882, 5906, 5932, 5959, 5987, 6017, 6048, 6080,
    6113, 6148, 6184, 6221, 6259, 6299, 6340, 6381,
    6424, 6468, 6513, 6559, 6606, 6654, 6703, 6752,
    6803, 6855, 6907, 6960, 7014, 7068, 7123, 7179,
    7235, 7292, 7350, 7408, 7466, 7525, 7585, 7644,
    7704, 7765, 7825, 7886, 7947, 8008, 8069, 8131
};


/* ============================================================
 * MIDI NOTE FREQUENCY TABLE
 * ============================================================
 *
 * Frequencies are stored in milli-Hz.
 *
 * Example:
 *
 * MIDI 60:
 * 261.626 Hz
 * =
 * 261626 milli-Hz
 *
 * This avoids using floating point.
 */

static const uint32_t midiFrequencyMilliHz[] =
{
    130813,     /* 48 C3  */
    138591,     /* 49 C#3 */
    146832,     /* 50 D3  */
    155563,     /* 51 D#3 */
    164814,     /* 52 E3  */
    174614,     /* 53 F3  */
    184997,     /* 54 F#3 */
    195998,     /* 55 G3  */
    207652,     /* 56 G#3 */
    220000,     /* 57 A3  */
    233082,     /* 58 A#3 */
    246942,     /* 59 B3  */

    261626,     /* 60 C4  */
    277183,     /* 61 C#4 */
    293665,     /* 62 D4  */
    311127,     /* 63 D#4 */
    329628,     /* 64 E4  */
    349228,     /* 65 F4  */
    369994,     /* 66 F#4 */
    391995,     /* 67 G4  */
    415305,     /* 68 G#4 */
    440000,     /* 69 A4  */
    466164,     /* 70 A#4 */
    493883,     /* 71 B4  */

    523251,     /* 72 C5  */
    554365,     /* 73 C#5 */
    587330,     /* 74 D5  */
    622254,     /* 75 D#5 */
    659255,     /* 76 E5  */
    698456,     /* 77 F5  */
    739989,     /* 78 F#5 */
    783991,     /* 79 G5  */
    830609,     /* 80 G#5 */
    880000,     /* 81 A5  */
    932328,     /* 82 A#5 */
    987767,     /* 83 B5  */

    1046502     /* 84 C6  */
};


/* ============================================================
 * AUDIO STATE
 * ============================================================
 */

volatile uint16_t g_lastDacCode = DAC_IDLE_CODE;
volatile uint32_t g_audioInterruptCount = 0;

volatile uint32_t g_phaseAccumulator = 0;
volatile uint32_t g_phaseIncrement = 0;

volatile bool g_noteActive = false;

volatile uint8_t g_currentMidiNote =
    MIDI_NOTE_NONE;


/* ============================================================
 * PARSED MELODY
 * ============================================================
 *
 * The MIDI parser fills this array.
 *
 * Each item says:
 *
 *      which MIDI note?
 *      how long should it play?
 *      how long should we rest afterwards?
 */

typedef struct
{
    uint8_t midiNote;

    uint32_t durationMs;

    uint32_t restAfterMs;

} MelodyEvent;


#define MAX_PARSED_NOTES         1024U

static MelodyEvent g_melody[MAX_PARSED_NOTES];

volatile uint32_t g_melodyLength = 0;

/* Reason for the most recent parse failure; reset on every parser entry. */
static bool g_noteCapacityExceeded = false;


/* ============================================================
 * PLAYBACK SCHEDULER STATE
 * ============================================================
 */

typedef enum
{
    PLAYBACK_IDLE = 0,

    PLAYBACK_NOTE,

    PLAYBACK_REST,

    PLAYBACK_COMPLETE

} PlaybackState;


volatile PlaybackState g_playbackState =
    PLAYBACK_IDLE;

volatile uint32_t g_melodyIndex = 0;

volatile bool g_playbackActive = false;


/* ============================================================
 * FORWARD DECLARATIONS
 * ============================================================
 */

void audioStopMidiNote(void);

static void PlaybackTimer_armMs(
    uint32_t milliseconds
);

static void playbackStartCurrentEvent(void);

static void playbackStartMelody(void);
static void playbackStop(void);


/* ============================================================
 * BYTE-ORDER HELPERS
 * ============================================================
 *
 * Standard MIDI files store their normal
 * multi-byte fields big-endian.
 */

static uint16_t readBigEndian16(
    const uint8_t *data
)
{
    return
        ((uint16_t)data[0] << 8) |
        ((uint16_t)data[1]);
}


static uint32_t readBigEndian32(
    const uint8_t *data
)
{
    return
        ((uint32_t)data[0] << 24) |
        ((uint32_t)data[1] << 16) |
        ((uint32_t)data[2] << 8) |
        ((uint32_t)data[3]);
}


/* ============================================================
 * MIDI VARIABLE-LENGTH QUANTITY
 * ============================================================
 *
 * MIDI delta times are stored using VLQs.
 *
 * Example:
 *
 *      480 ticks
 *
 * is:
 *
 *      83 60
 */

static bool readMidiVLQ(
    const uint8_t *data,
    uint32_t end,
    uint32_t *index,
    uint32_t *value
)
{
    uint32_t result = 0;

    uint32_t byteCount = 0;

    uint8_t byte;


    do
    {
        /*
         * Protect against reading outside
         * the MIDI track.
         */
        if (*index >= end)
        {
            return false;
        }


        /*
         * Standard MIDI VLQ is at most
         * four bytes long.
         */
        if (byteCount >= 4U)
        {
            return false;
        }


        byte =
            data[*index];


        (*index)++;

        byteCount++;


        /*
         * Every VLQ byte contributes
         * seven data bits.
         */
        result =
            (result << 7) |
            (byte & 0x7FU);

    }
    while ((byte & 0x80U) != 0U);


    *value = result;

    return true;
}


/* ============================================================
 * TIME CONVERSION
 * ============================================================
 */

static uint32_t microsecondsToMilliseconds(
    uint64_t microseconds
)
{
    uint32_t milliseconds;


    /*
     * +500 gives basic rounding instead
     * of always truncating downward.
     */
    milliseconds =
        (uint32_t)(
            (microseconds + 500ULL)
            /
            1000ULL
        );


    /*
     * Avoid accidentally turning a real
     * non-zero note into 0 ms.
     */
    if ((microseconds > 0ULL) &&
        (milliseconds == 0U))
    {
        milliseconds = 1U;
    }


    return milliseconds;
}


/* ============================================================
 * MIDI NOTE -> FREQUENCY
 * ============================================================
 */

static uint32_t midiNoteToFrequencyMilliHz(
    uint8_t midiNote
)
{
    if ((midiNote < MIDI_NOTE_MIN) ||
        (midiNote > MIDI_NOTE_MAX))
    {
        return 0;
    }


    return midiFrequencyMilliHz[
        midiNote - MIDI_NOTE_MIN
    ];
}


/* ============================================================
 * DAC8311 WRITE
 * ============================================================
 */

void DAC8311_write(
    uint16_t sample
)
{
    uint32_t dummyData;


    /*
     * DAC8311 is 14-bit.
     */
    sample &= 0x3FFFU;


    /*
     * Begin transaction.
     *
     * DAC SYNC is active LOW.
     *
     * Your working jumper setup uses PN3.
     */
    GPIOPinWrite(
        GPIO_PORTN_BASE,
        GPIO_PIN_3,
        0
    );


    /*
     * Send complete 16-bit DAC word.
     */
    SSIDataPut(
        SSI2_BASE,
        (uint32_t)sample
    );


    /*
     * Wait until SPI transmission has finished.
     */
    while (SSIBusy(SSI2_BASE))
    {
    }


    /*
     * Finish DAC transaction.
     */
    GPIOPinWrite(
        GPIO_PORTN_BASE,
        GPIO_PIN_3,
        GPIO_PIN_3
    );


    /*
     * Drain dummy RX data.
     */
    while (SSIDataGetNonBlocking(
               SSI2_BASE,
               &dummyData))
    {
    }


    g_lastDacCode =
        sample;
}


/* ============================================================
 * TIMER2 AUDIO ISR
 * ============================================================
 *
 * Timer2 has ONE job:
 *
 * generate audio samples at 20 kHz.
 */

void timer2ISR(void)
{
    uint32_t tableIndex;

    uint16_t sample;

    int32_t centeredSample;


    TimerIntClear(
        TIMER2_BASE,
        TIMER_TIMA_TIMEOUT
    );


    if (g_noteActive)
    {
        /*
         * Use upper 8 phase bits to select
         * one of 256 sine values.
         */
        tableIndex =
            g_phaseAccumulator >>
            (32U - SINE_TABLE_BITS);


        /*
         * Shift sine table around zero.
         */
        centeredSample =
            (int32_t)sineTable[tableIndex]
            -
            DAC_IDLE_CODE;


        /*
         * Keep the quieter amplitude that
         * worked well in your testing.
         */
        centeredSample *= 3;


        /*
         * Shift waveform back to DAC midpoint.
         */
        sample =
            (uint16_t)(
                DAC_IDLE_CODE +
                centeredSample
            );


        DAC8311_write(
            sample
        );


        /*
         * Advance oscillator phase.
         */
        g_phaseAccumulator +=
            g_phaseIncrement;
    }


    g_audioInterruptCount++;
}


/* ============================================================
 * AUDIO HARDWARE INITIALIZATION
 * ============================================================
 */

void Audio_init(void)
{
    uint32_t dummyData;


    /*
     * Required GPIO / SSI hardware.
     */
    SysCtlPeripheralEnable(
        SYSCTL_PERIPH_GPIOD
    );

    SysCtlPeripheralEnable(
        SYSCTL_PERIPH_GPION
    );

    SysCtlPeripheralEnable(
        SYSCTL_PERIPH_GPIOH
    );

    SysCtlPeripheralEnable(
        SYSCTL_PERIPH_SSI2
    );


    while (!SysCtlPeripheralReady(
               SYSCTL_PERIPH_GPIOD))
    {
    }


    while (!SysCtlPeripheralReady(
               SYSCTL_PERIPH_GPION))
    {
    }


    while (!SysCtlPeripheralReady(
               SYSCTL_PERIPH_GPIOH))
    {
    }


    while (!SysCtlPeripheralReady(
               SYSCTL_PERIPH_SSI2))
    {
    }


    /*
     * PD3 = SSI2CLK
     */
    GPIOPinConfigure(
        GPIO_PD3_SSI2CLK
    );


    /*
     * PD1 = SSI2XDAT0 / MOSI
     */
    GPIOPinConfigure(
        GPIO_PD1_SSI2XDAT0
    );


    GPIOPinTypeSSI(
        GPIO_PORTD_BASE,
        GPIO_PIN_1 |
        GPIO_PIN_3
    );


    /*
     * PN3 = DAC SYNC.
     */
    GPIOPinTypeGPIOOutput(
        GPIO_PORTN_BASE,
        GPIO_PIN_3
    );


    /*
     * PH2 = amplifier shutdown control.
     *
     * This is the corrected pin that removed
     * the static in your setup.
     */
    GPIOPinTypeGPIOOutput(
        GPIO_PORTH_BASE,
        GPIO_PIN_2
    );


    /*
     * DAC SYNC idles HIGH.
     */
    GPIOPinWrite(
        GPIO_PORTN_BASE,
        GPIO_PIN_3,
        GPIO_PIN_3
    );


    /*
     * Amplifier starts OFF.
     *
     * TPA301 shutdown is active HIGH.
     */
    GPIOPinWrite(
        GPIO_PORTH_BASE,
        GPIO_PIN_2,
        GPIO_PIN_2
    );


    /*
     * Configure SSI2.
     */
    SSIDisable(
        SSI2_BASE
    );


    SSIConfigSetExpClk(
        SSI2_BASE,
        SYSTEM_CLOCK_HZ,
        SSI_FRF_MOTO_MODE_1,
        SSI_MODE_MASTER,
        DAC_SPI_BIT_RATE_HZ,
        16
    );


    /*
     * Clear receive FIFO.
     */
    while (SSIDataGetNonBlocking(
               SSI2_BASE,
               &dummyData))
    {
    }


    SSIEnable(
        SSI2_BASE
    );


    /*
     * Start DAC at midpoint / silence.
     */
    DAC8311_write(
        DAC_IDLE_CODE
    );
}


/* ============================================================
 * TIMER2 INITIALIZATION
 * ============================================================
 */

void AudioTimer_init(void)
{
    SysCtlPeripheralEnable(
        SYSCTL_PERIPH_TIMER2
    );


    while (!SysCtlPeripheralReady(
               SYSCTL_PERIPH_TIMER2))
    {
    }


    TimerDisable(
        TIMER2_BASE,
        TIMER_A
    );


    TimerConfigure(
        TIMER2_BASE,
        TIMER_CFG_PERIODIC
    );


    /*
     * 120 MHz / 6000 =
     * 20,000 samples/sec.
     */
    TimerLoadSet(
        TIMER2_BASE,
        TIMER_A,
        AUDIO_TIMER_COUNTS - 1U
    );


    TimerIntRegister(
        TIMER2_BASE,
        TIMER_A,
        timer2ISR
    );


    TimerIntClear(
        TIMER2_BASE,
        TIMER_TIMA_TIMEOUT
    );


    TimerIntEnable(
        TIMER2_BASE,
        TIMER_TIMA_TIMEOUT
    );


    /*
     * Audio starts silent.
     *
     * Timer2 does NOT start yet.
     */
    g_phaseAccumulator = 0;

    g_phaseIncrement = 0;

    g_noteActive = false;

    g_currentMidiNote =
        MIDI_NOTE_NONE;
}


/* ============================================================
 * NOTE OFF
 * ============================================================
 */

void audioStopMidiNote(void)
{
    /*
     * Stop waveform generation first.
     */
    TimerDisable(
        TIMER2_BASE,
        TIMER_A
    );


    TimerIntClear(
        TIMER2_BASE,
        TIMER_TIMA_TIMEOUT
    );


    /*
     * No active note.
     */
    g_noteActive = false;

    g_currentMidiNote =
        MIDI_NOTE_NONE;


    /*
     * Reset oscillator.
     */
    g_phaseAccumulator = 0;

    g_phaseIncrement = 0;


    /*
     * Return DAC to midpoint.
     */
    DAC8311_write(
        DAC_IDLE_CODE
    );


    /*
     * Shut down amplifier.
     *
     * HIGH = amplifier OFF.
     */
    GPIOPinWrite(
        GPIO_PORTH_BASE,
        GPIO_PIN_2,
        GPIO_PIN_2
    );
}


/* Cancel the entire song, including a scheduled rest. Preserve the caller's
 * interrupt-mask state; ACK_STOPPED is sent only after this function returns.
 * Timer4 remains running so transfer timeouts continue to work in silence.
 */
static void playbackStop(void)
{
    bool interruptsWereDisabled = IntMasterDisable();

    TimerDisable(TIMER3_BASE, TIMER_A);
    TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
    g_playbackActive = false;
    g_playbackState = PLAYBACK_IDLE;
    g_melodyIndex = 0U;
    g_melodyLength = 0U;
    playbackCompletionPending = false;
    audioStopMidiNote();

    /* Read back the peripheral clears before clearing latched NVIC requests. */
    (void)TimerIntStatus(TIMER3_BASE, false);
    (void)TimerIntStatus(TIMER2_BASE, false);
    IntPendClear(INT_TIMER3A);
    IntPendClear(INT_TIMER2A);

    if (!interruptsWereDisabled)
    {
        IntMasterEnable();
    }
}


/* ============================================================
 * NOTE ON
 * ============================================================
 */

void audioStartMidiNote(
    uint8_t midiNote
)
{
    uint32_t frequencyMilliHz;


    frequencyMilliHz =
        midiNoteToFrequencyMilliHz(
            midiNote
        );


    /*
     * Unsupported note.
     */
    if (frequencyMilliHz == 0U)
    {
        audioStopMidiNote();

        return;
    }


    /*
     * Defensive protection:
     *
     * never leave an old note sounding when
     * starting a new one.
     */
    if (g_noteActive)
    {
        audioStopMidiNote();
    }


    TimerDisable(
        TIMER2_BASE,
        TIMER_A
    );


    /*
     * DDS phase increment:
     *
     * frequency * 2^32
     * ----------------
     * sample rate
     *
     * Frequency is stored in milli-Hz,
     * therefore sample rate is multiplied
     * by 1000.
     */
    g_phaseIncrement =
        (uint32_t)(
            ((uint64_t)frequencyMilliHz << 32)
            /
            (
                (uint64_t)AUDIO_SAMPLE_RATE_HZ *
                1000ULL
            )
        );


    g_phaseAccumulator = 0;


    g_currentMidiNote =
        midiNote;


    g_noteActive = true;


    /*
     * Start from silent DAC level.
     */
    DAC8311_write(
        DAC_IDLE_CODE
    );


    /*
     * Enable amplifier.
     *
     * LOW = amplifier ON.
     */
    GPIOPinWrite(
        GPIO_PORTH_BASE,
        GPIO_PIN_2,
        0
    );


    TimerIntClear(
        TIMER2_BASE,
        TIMER_TIMA_TIMEOUT
    );


    /*
     * Begin generating sine samples.
     */
    TimerEnable(
        TIMER2_BASE,
        TIMER_A
    );
}


/* ============================================================
 * TIMER3 PLAYBACK TIMER
 * ============================================================
 *
 * Timer3 has ONE job:
 *
 * musical/event timing.
 */

static void PlaybackTimer_armMs(
    uint32_t milliseconds
)
{
    uint64_t timerCounts;


    if (milliseconds == 0U)
    {
        milliseconds = 1U;
    }


    /*
     * Convert milliseconds to timer counts.
     */
    timerCounts =
        (
            (uint64_t)SYSTEM_CLOCK_HZ *
            milliseconds
        )
        /
        1000ULL;


    /*
     * Protect 32-bit Timer3 load register.
     */
    if (timerCounts >
        0xFFFFFFFFULL)
    {
        timerCounts =
            0xFFFFFFFFULL;
    }


    if (timerCounts == 0ULL)
    {
        timerCounts = 1ULL;
    }


    TimerDisable(
        TIMER3_BASE,
        TIMER_A
    );


    TimerIntClear(
        TIMER3_BASE,
        TIMER_TIMA_TIMEOUT
    );


    TimerLoadSet(
        TIMER3_BASE,
        TIMER_A,
        (uint32_t)timerCounts - 1U
    );


    TimerEnable(
        TIMER3_BASE,
        TIMER_A
    );
}


/* ============================================================
 * START CURRENT PARSED NOTE
 * ============================================================
 */

static void playbackStartCurrentEvent(void)
{
    const MelodyEvent *event;


    /*
     * End of parsed melody.
     */
    if ((g_melodyIndex >= g_melodyLength) ||
        (g_melodyIndex >= MAX_PARSED_NOTES))
    {
        /*
         * Extra safety Note Off.
         */
        audioStopMidiNote();


        g_playbackActive =
            false;


        g_playbackState =
            PLAYBACK_COMPLETE;


        return;
    }


    event =
        &g_melody[g_melodyIndex];


    /*
     * NOTE ON.
     */
    audioStartMidiNote(
        event->midiNote
    );


    g_playbackState =
        PLAYBACK_NOTE;


    /*
     * Timer3 determines when the
     * NOTE OFF occurs.
     */
    PlaybackTimer_armMs(
        event->durationMs
    );
}


/* ============================================================
 * TIMER3 ISR
 * ============================================================
 */

void timer3ISR(void)
{
    const MelodyEvent *event;


    TimerIntClear(
        TIMER3_BASE,
        TIMER_TIMA_TIMEOUT
    );


    /* A late interrupt after STOP must not advance/restart the melody. */
    if (!g_playbackActive)
    {
        TimerDisable(TIMER3_BASE, TIMER_A);
        return;
    }
    if ((g_melodyLength > MAX_PARSED_NOTES) ||
        (g_melodyIndex >= g_melodyLength) ||
        (g_melodyIndex >= MAX_PARSED_NOTES))
    {
        playbackStop();
        return;
    }

    /*
     * ----------------------------------------
     * NOTE duration just finished.
     * ----------------------------------------
     */
    if (g_playbackState ==
        PLAYBACK_NOTE)
    {
        event =
            &g_melody[g_melodyIndex];


        /*
         * IMPORTANT:
         *
         * Explicitly turn the note OFF.
         */
        audioStopMidiNote();


        /*
         * Is there a rest after this note?
         */
        if (event->restAfterMs > 0U)
        {
            g_playbackState =
                PLAYBACK_REST;


            /*
             * Timer3 now times silence.
             */
            PlaybackTimer_armMs(
                event->restAfterMs
            );
        }
        else
        {
            /*
             * No rest.
             *
             * Go directly to next note.
             */
            g_melodyIndex++;


            playbackStartCurrentEvent();
        }
    }


    /*
     * ----------------------------------------
     * REST just finished.
     * ----------------------------------------
     */
    else if (g_playbackState ==
             PLAYBACK_REST)
    {
        g_melodyIndex++;


        playbackStartCurrentEvent();
    }


    /*
     * Unexpected state.
     */
    else
    {
        audioStopMidiNote();


        TimerDisable(
            TIMER3_BASE,
            TIMER_A
        );


        g_playbackActive =
            false;


        g_playbackState =
            PLAYBACK_IDLE;
    }
}


/* ============================================================
 * TIMER3 INITIALIZATION
 * ============================================================
 */

void PlaybackTimer_init(void)
{
    SysCtlPeripheralEnable(
        SYSCTL_PERIPH_TIMER3
    );


    while (!SysCtlPeripheralReady(
               SYSCTL_PERIPH_TIMER3))
    {
    }


    TimerDisable(
        TIMER3_BASE,
        TIMER_A
    );


    /*
     * One-shot because every note/rest
     * can have a different duration.
     */
    TimerConfigure(
        TIMER3_BASE,
        TIMER_CFG_ONE_SHOT
    );


    TimerIntRegister(
        TIMER3_BASE,
        TIMER_A,
        timer3ISR
    );


    TimerIntClear(
        TIMER3_BASE,
        TIMER_TIMA_TIMEOUT
    );


    TimerIntEnable(
        TIMER3_BASE,
        TIMER_TIMA_TIMEOUT
    );


    g_playbackState =
        PLAYBACK_IDLE;


    g_playbackActive =
        false;


    g_melodyIndex = 0;
}


/* ============================================================
 * START PARSED MELODY
 * ============================================================
 */

static void playbackStartMelody(void)
{
    bool interruptsWereDisabled = IntMasterDisable();

    TimerDisable(TIMER3_BASE, TIMER_A);
    TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
    audioStopMidiNote();
    (void)TimerIntStatus(TIMER3_BASE, false);
    (void)TimerIntStatus(TIMER2_BASE, false);
    IntPendClear(INT_TIMER3A);
    IntPendClear(INT_TIMER2A);

    g_melodyIndex = 0U;
    g_playbackState = PLAYBACK_IDLE;
    g_playbackActive = false;
    if ((g_melodyLength > 0U) && (g_melodyLength <= MAX_PARSED_NOTES))
    {
        g_playbackActive = true;
        playbackStartCurrentEvent();
    }

    if (!interruptsWereDisabled)
    {
        IntMasterEnable();
    }
}


/* ============================================================
 * DELIVERABLE 5: BOUNDED TYPE-0 MIDI PARSER
 * ============================================================
 * Contract: one track, PPQN, channel 0, monophonic notes 48..84.
 * Decode channel running status; Note On velocity 0 ends a note.
 * Consume/ignore A/B/C/D/E channel events while retaining elapsed time.
 * Read tempo/EOT; skip other bounded meta events. SysEx remains unsupported.
 * This parser accepts a complete file before audio playback can start.
 */

/* CCS diagnostics for the most recent parse attempt; no extra UART lines. */
volatile uint32_t g_lastMidiRunningStatusCount = 0U;
volatile uint32_t g_lastMidiSkippedChannelEvents = 0U;
volatile uint32_t g_lastMidiSkippedMetaEvents = 0U;

/* Validate common fixed-size meta events before accessing/skipping their data.
 * Other meta types are opaque, length-delimited data under this contract.
 */
static bool midiMetaLengthValid(uint8_t type, uint32_t length)
{
    switch (type)
    {
        case 0x00U: return length == 2U;  /* sequence number */
        case 0x20U:                     /* channel prefix */
        case 0x21U: return length == 1U;  /* MIDI port */
        case 0x2FU: return length == 0U;  /* end of track */
        case 0x51U: return length == 3U;  /* tempo */
        case 0x54U: return length == 5U;  /* SMPTE offset metadata */
        case 0x58U: return length == 4U;  /* time signature */
        case 0x59U: return length == 2U;  /* key signature */
        default: return true;            /* text/opaque metadata */
    }
}

static bool midiTimeFitsMilliseconds(uint64_t us)
{
    /* Keep the existing millisecond scheduler, but prevent narrowing wrap. */
    return us <= ((uint64_t)UINT32_MAX * 1000ULL + 499ULL);
}

static bool parseSimpleMidi(const uint8_t *data, uint32_t fileLength)
{
    uint32_t index;
    uint32_t trackEnd;
    uint32_t tempoUsPerQuarter = 500000U;
    uint32_t tickFraction = 0U;
    uint16_t division;
    uint64_t currentTimeUs = 0ULL;
    uint64_t noteStartTimeUs = 0ULL;
    uint64_t lastNoteOffTimeUs = 0ULL;
    uint8_t runningStatus = 0U;
    uint8_t activeNote = MIDI_NOTE_NONE;
    bool parserNoteActive = false;

    g_melodyLength = 0U;
    g_noteCapacityExceeded = false;
    g_lastMidiRunningStatusCount = 0U;
    g_lastMidiSkippedChannelEvents = 0U;
    g_lastMidiSkippedMetaEvents = 0U;

    /* Project subset: standard six-byte header, exactly one Type-0 track. */
    if ((fileLength < 22U) || (memcmp(data, "MThd", 4U) != 0) ||
        (readBigEndian32(&data[4]) != 6U) ||
        (readBigEndian16(&data[8]) != 0U) ||
        (readBigEndian16(&data[10]) != 1U))
    {
        return false;
    }
    division = readBigEndian16(&data[12]);
    if ((division == 0U) || ((division & 0x8000U) != 0U) ||
        (memcmp(&data[14], "MTrk", 4U) != 0))
    {
        return false;
    }
    index = 22U;
    /* Exact equality also rejects extra chunks/bytes outside our one track. */
    if (readBigEndian32(&data[18]) != (fileLength - index))
    {
        return false;
    }
    trackEnd = fileLength;

    while (index < trackEnd)
    {
        uint32_t deltaTicks;
        uint64_t numerator;
        uint64_t deltaUs;
        uint8_t status;
        uint8_t kind;
        uint32_t dataLength;
        uint32_t i;
        uint8_t first;
        uint8_t second;

        if (!readMidiVLQ(data, trackEnd, &index, &deltaTicks))
        {
            return false;
        }
        /* Include every event's delta, even if its musical effect is ignored.
         * Carry the fractional microsecond: subdividing a wait into controller
         * events must not lose one rounding remainder per ignored event.
         */
        numerator = (uint64_t)deltaTicks * tempoUsPerQuarter + tickFraction;
        deltaUs = numerator / division;
        tickFraction = (uint32_t)(numerator % division);
        if (deltaUs > (UINT64_MAX - currentTimeUs))
        {
            return false;
        }
        currentTimeUs += deltaUs;
        if (index >= trackEnd)
        {
            return false;
        }

        status = data[index];
        if (status < 0x80U)
        {
            if (runningStatus == 0U)
            {
                return false;
            }
            status = runningStatus;
            /* The current byte is the first data byte; do not consume it yet. */
            g_lastMidiRunningStatusCount++;
        }
        else
        {
            index++;
            if (status < 0xF0U)
            {
                runningStatus = status; /* includes message kind AND channel */
            }
            else
            {
                /* In SMF, meta/SysEx events cancel channel running status. */
                runningStatus = 0U;
            }
        }

        if (status == 0xFFU)
        {
            uint8_t metaType;
            uint32_t metaLength;
            if (index >= trackEnd)
            {
                return false;
            }
            metaType = data[index++];
            if ((metaType >= 0x80U) ||
                !readMidiVLQ(data, trackEnd, &index, &metaLength) ||
                (metaLength > (trackEnd - index)) ||
                !midiMetaLengthValid(metaType, metaLength))
            {
                return false;
            }
            if (metaType == 0x51U)
            {
                tempoUsPerQuarter = ((uint32_t)data[index] << 16) |
                                    ((uint32_t)data[index + 1U] << 8) |
                                    (uint32_t)data[index + 2U];
                if (tempoUsPerQuarter == 0U)
                {
                    return false;
                }
            }
            else if (metaType == 0x2FU)
            {
                /* EOT must be the final event, with every note already ended. */
                return (!parserNoteActive && (g_melodyLength > 0U) &&
                        (index == trackEnd));
            }
            else
            {
                if (((metaType == 0x20U) && (data[index] > 15U)) ||
                    ((metaType == 0x21U) && (data[index] > 127U)))
                {
                    return false;
                }
                g_lastMidiSkippedMetaEvents++;
            }
            index += metaLength;
            continue;
        }

        /* Channel messages only. F0/F7 SysEx and raw system statuses are not
         * part of the agreed input subset, so reject instead of guessing.
         * Channel 0 in bytes corresponds to channel 1 in many music programs.
         */
        if ((status >= 0xF0U) || ((status & 0x0FU) != 0U))
        {
            return false;
        }
        kind = status & 0xF0U;
        dataLength = ((kind == 0xC0U) || (kind == 0xD0U)) ? 1U : 2U;
        if (dataLength > (trackEnd - index))
        {
            return false;
        }
        for (i = 0U; i < dataLength; i++)
        {
            if (data[index + i] >= 0x80U)
            {
                return false;
            }
        }
        first = data[index];
        second = (dataLength == 2U) ? data[index + 1U] : 0U;
        index += dataLength;

        if ((kind != 0x80U) && (kind != 0x90U))
        {
            /* A0/B0/C0/D0/E0: pressure, controllers, programs, pitch bend.
             * Consume correctly but do not implement their synthesis effects.
             */
            g_lastMidiSkippedChannelEvents++;
            continue;
        }
        if ((kind == 0x90U) && (second != 0U))
        {
            uint64_t restUs;
            if (parserNoteActive || (first < MIDI_NOTE_MIN) || (first > MIDI_NOTE_MAX))
            {
                return false;
            }
            if (g_melodyLength > 0U)
            {
                restUs = currentTimeUs - lastNoteOffTimeUs;
                if (!midiTimeFitsMilliseconds(restUs))
                {
                    return false;
                }
                g_melody[g_melodyLength - 1U].restAfterMs =
                    microsecondsToMilliseconds(restUs);
            }
            activeNote = first;
            noteStartTimeUs = currentTimeUs;
            parserNoteActive = true;
        }
        else
        {
            uint64_t durationUs;
            /* Both Note Off encodings use this one bounded append path. */
            if (!parserNoteActive || (first != activeNote))
            {
                return false;
            }
            if (g_melodyLength >= MAX_PARSED_NOTES)
            {
                g_noteCapacityExceeded = true;
                return false;
            }
            durationUs = currentTimeUs - noteStartTimeUs;
            if (!midiTimeFitsMilliseconds(durationUs))
            {
                return false;
            }
            g_melody[g_melodyLength].midiNote = activeNote;
            g_melody[g_melodyLength].durationMs = microsecondsToMilliseconds(durationUs);
            g_melody[g_melodyLength].restAfterMs = 0U;
            g_melodyLength++;
            parserNoteActive = false;
            activeNote = MIDI_NOTE_NONE;
            lastNoteOffTimeUs = currentTimeUs;
        }
    }
    return false; /* missing End Of Track */
}


/* ============================================================
 * UART INITIALIZATION
 * ============================================================
 */

static void UART0_init(void)
{
    /*
     * Enable UART0 and GPIO Port A.
     */
    SysCtlPeripheralEnable(
        SYSCTL_PERIPH_UART0
    );


    SysCtlPeripheralEnable(
        SYSCTL_PERIPH_GPIOA
    );


    while (!SysCtlPeripheralReady(
               SYSCTL_PERIPH_UART0))
    {
    }


    while (!SysCtlPeripheralReady(
               SYSCTL_PERIPH_GPIOA))
    {
    }


    /*
     * PA0 = UART RX
     * PA1 = UART TX
     */
    GPIOPinConfigure(
        GPIO_PA0_U0RX
    );


    GPIOPinConfigure(
        GPIO_PA1_U0TX
    );


    GPIOPinTypeUART(
        GPIO_PORTA_BASE,
        GPIO_PIN_0 |
        GPIO_PIN_1
    );


    /*
     * Use 16 MHz alternate UART clock.
     */
    UARTClockSourceSet(
        UART0_BASE,
        UART_CLOCK_ALTCLK
    );


    /*
     * 115200 baud
     * 8 data bits
     * 1 stop bit
     * no parity
     */
    UARTConfigSetExpClk(
        UART0_BASE,
        16000000U,
        115200,
        UART_CONFIG_WLEN_8 |
        UART_CONFIG_STOP_ONE |
        UART_CONFIG_PAR_NONE
    );


    UARTEnable(
        UART0_BASE
    );
}


/* ============================================================
 * UART PRINT
 * ============================================================
 */

static void UART0_writeString(
    const char *text
)
{
    while (*text != '\0')
    {
        UARTCharPut(
            UART0_BASE,
            *text
        );


        text++;
    }
}


/* ============================================================
 * DELIVERABLE 2: TRANSFER CLOCK, CRC AND RECOVERABLE UART RECEIVER
 * ============================================================ */

static void transferTimer4ISR(void)
{
    TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
    g_transferMilliseconds++;
}

static void TransferClock_init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER4))
    {
    }

    TimerDisable(TIMER4_BASE, TIMER_A);
    TimerConfigure(TIMER4_BASE, TIMER_CFG_PERIODIC);
    TimerClockSourceSet(TIMER4_BASE, TIMER_CLOCK_SYSTEM);
    TimerLoadSet(TIMER4_BASE, TIMER_A, (SYSTEM_CLOCK_HZ / 1000U) - 1U);
    TimerIntRegister(TIMER4_BASE, TIMER_A, transferTimer4ISR);
    TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
    g_transferMilliseconds = 0U;
    TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER4_BASE, TIMER_A);
}

static uint32_t readLittleEndian32(const uint8_t *data)
{
    return ((uint32_t)data[0]) |
           ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) |
           ((uint32_t)data[3] << 24U);
}

/* CRC-32/ISO-HDLC, matching zlib.crc32(data).
 * ASCII "123456789" must produce 0xCBF43926.
 */
static uint32_t midiPayloadCrc32(const uint8_t *data, uint32_t length)
{
    uint32_t crc = 0xFFFFFFFFU;
    uint32_t i;
    uint32_t bit;

    for (i = 0U; i < length; i++)
    {
        crc ^= (uint32_t)data[i];
        for (bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 1U) != 0U)
            {
                crc = (crc >> 1U) ^ 0xEDB88320U;
            }
            else
            {
                crc >>= 1U;
            }
        }
    }
    return crc ^ 0xFFFFFFFFU;
}

/* Small formatters avoid pulling printf into the embedded build. */
static void UART0_writeDecimal(uint32_t value)
{
    char digits[10];
    uint32_t count = 0U;
    do
    {
        digits[count++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while (value != 0U);

    while (count != 0U)
    {
        UARTCharPut(UART0_BASE, digits[--count]);
    }
}

static void UART0_writeHex32(uint32_t value)
{
    static const char hex[] = "0123456789ABCDEF";
    uint32_t shift;
    for (shift = 32U; shift != 0U; shift -= 4U)
    {
        UARTCharPut(UART0_BASE, hex[(value >> (shift - 4U)) & 0xFU]);
    }
}

static void UART0_writeTransferAck(const char *name,
                                  uint32_t length,
                                  uint32_t crc)
{
    UART0_writeString(name);
    UARTCharPut(UART0_BASE, ' ');
    UART0_writeDecimal(length);
    UARTCharPut(UART0_BASE, ' ');
    UART0_writeHex32(crc);
    UART0_writeString("\r\n");
}

static void UARTReceiver_reset(void)
{
    headerBytesReceived = 0U;
    magicBytesMatched = 0U;
    expectedLength = 0U;
    bytesReceived = 0U;
    expectedCrc32 = 0U;
    headerStartedMs = 0U;
    payloadStartedMs = 0U;
    lastRxByteMs = g_transferMilliseconds;
    receiverState = WAIT_MAGIC;
}

static void UARTReceiver_fail(const char *error)
{
    /* One error line per failed attempt, even if more bad bytes arrive.
     * Never stop an already playing melody in this routine: ERR_BUSY and
     * UART noise during playback must not modify the active melody.
     */
    if (receiverState == RECOVER)
    {
        lastRxByteMs = g_transferMilliseconds;
        return;
    }

    g_transferErrorCount++;
    UARTReceiver_reset();
    receiverState = RECOVER;
    UART0_writeString(error);
    UART0_writeString("\r\n");
    lastRxByteMs = g_transferMilliseconds;
}

static void UARTReceiver_checkTimeout(void)
{
    uint32_t now = g_transferMilliseconds;

    /* Unsigned subtraction also works when the millisecond counter wraps. */
    if (((receiverState == READ_HEADER) ||
         ((receiverState == WAIT_MAGIC) && (magicBytesMatched != 0U))) &&
        ((uint32_t)(now - headerStartedMs) >= RX_HEADER_TIMEOUT_MS))
    {
        UARTReceiver_fail("ERR_TIMEOUT");
    }
    else if ((receiverState == READ_PAYLOAD) &&
             (((uint32_t)(now - lastRxByteMs) >= RX_PAYLOAD_IDLE_MS) ||
              ((uint32_t)(now - payloadStartedMs) >= RX_PAYLOAD_TOTAL_MS)))
    {
        UARTReceiver_fail("ERR_TIMEOUT");
    }
    else if ((receiverState == RECOVER) &&
             !UARTCharsAvail(UART0_BASE) &&
             ((uint32_t)(now - lastRxByteMs) >= RX_RECOVERY_QUIET_MS))
    {
        UARTReceiver_reset();
        /* IDLE means the receiver accepts another header. Playback may
         * still be active after ERR_BUSY; that next header will be refused.
         */
        UART0_writeString("IDLE\r\n");
    }
}

static void UARTReceiver_acceptHeader(void)
{
    uint8_t version = headerBytes[4];
    uint8_t command = headerBytes[5];

    if ((version != SMID_PROTOCOL_VERSION) && (version != SMID_LEGACY_VERSION))
    {
        UARTReceiver_fail("ERR_VERSION");
        return;
    }
    if ((headerBytes[6] != 0U) || (headerBytes[7] != 0U) ||
        ((version == SMID_LEGACY_VERSION) && (command != SMID_COMMAND_MIDI)))
    {
        UARTReceiver_fail("ERR_HEADER");
        return;
    }
    if ((command != SMID_COMMAND_MIDI) && (command != SMID_COMMAND_STOP))
    {
        UARTReceiver_fail("ERR_COMMAND");
        return;
    }
    if (command == SMID_COMMAND_STOP)
    {
        if ((readLittleEndian32(&headerBytes[8]) != 0U) ||
            (readLittleEndian32(&headerBytes[12]) != 0U))
        {
            UARTReceiver_fail("ERR_HEADER");
            return;
        }
        playbackStop();
        UARTReceiver_reset();
        UART0_writeString("ACK_STOPPED\r\nIDLE\r\n");
        return;
    }

    expectedLength = readLittleEndian32(&headerBytes[8]);
    expectedCrc32 = readLittleEndian32(&headerBytes[12]);
    g_lastTransferLength = expectedLength;
    g_lastExpectedCrc32 = expectedCrc32;
    g_lastCalculatedCrc32 = 0U;

    if ((expectedLength == 0U) || (expectedLength > MAX_MIDI_FILE_SIZE))
    {
        UARTReceiver_fail("ERR_SIZE");
        return;
    }
    if (g_playbackActive)
    {
        UARTReceiver_fail("ERR_BUSY");
        return;
    }

    bytesReceived = 0U;
    receiverState = READ_PAYLOAD;
    UART0_writeTransferAck("ACK_READY", expectedLength, expectedCrc32);
    /* Start after queuing the acknowledgement, giving the host a full
     * idle interval to receive it and begin sending the payload.
     */
    payloadStartedMs = g_transferMilliseconds;
    lastRxByteMs = payloadStartedMs;
}

static void UARTReceiver_processByte(uint8_t byte)
{
    static const uint8_t magic[4] = {'S', 'M', 'I', 'D'};
    uint32_t now = g_transferMilliseconds;
    lastRxByteMs = now;

    if (receiverState == RECOVER)
    {
        return;
    }

    if (receiverState == WAIT_MAGIC)
    {
        if (byte == magic[magicBytesMatched])
        {
            if (magicBytesMatched == 0U)
            {
                headerStartedMs = now;
            }
            magicBytesMatched++;
            if (magicBytesMatched == 4U)
            {
                memcpy(headerBytes, magic, sizeof(magic));
                headerBytesReceived = 4U;
                magicBytesMatched = 0U;
                receiverState = READ_HEADER;
            }
        }
        else if (byte == magic[0])
        {
            /* Preserve a new 'S', e.g. the second S in "SSMID". */
            magicBytesMatched = 1U;
            headerStartedMs = now;
        }
        else
        {
            magicBytesMatched = 0U;
        }
        return;
    }

    if (receiverState == READ_HEADER)
    {
        if (headerBytesReceived >= SMID_HEADER_SIZE)
        {
            UARTReceiver_fail("ERR_HEADER");
            return;
        }
        headerBytes[headerBytesReceived++] = byte;
        if (headerBytesReceived == SMID_HEADER_SIZE)
        {
            UARTReceiver_acceptHeader();
        }
        return;
    }

    if (receiverState == READ_PAYLOAD)
    {
        if ((bytesReceived >= expectedLength) ||
            (bytesReceived >= MAX_MIDI_FILE_SIZE))
        {
            UARTReceiver_fail("ERR_SIZE");
            return;
        }
        /* All payload bytes are data: do not look for SMID or commands. */
        fileBuffer[bytesReceived++] = byte;
        if (bytesReceived == expectedLength)
        {
            receiverState = VALIDATE;
        }
    }
}

static void UARTReceiver_validate(void)
{
    uint32_t actualCrc;
    if (receiverState != VALIDATE)
    {
        return;
    }

    /* The host must wait for the result before sending anything else. */
    if (UARTCharsAvail(UART0_BASE))
    {
        UARTReceiver_fail("ERR_SIZE");
        return;
    }
    actualCrc = midiPayloadCrc32(fileBuffer, expectedLength);
    g_lastCalculatedCrc32 = actualCrc;
    if (actualCrc != expectedCrc32)
    {
        UARTReceiver_fail("ERR_CRC");
        return;
    }

    UART0_writeTransferAck("ACK_RECEIVED", expectedLength, actualCrc);
    if (!parseSimpleMidi(fileBuffer, expectedLength))
    {
        /* Parsing is reached only after accepting a header while idle. */
        playbackStop();
        UARTReceiver_fail(g_noteCapacityExceeded ?
                          "ERR_NOTE_CAPACITY" : "ERR_MIDI");
        return;
    }

    g_transferSuccessCount++;
    UART0_writeString("ACK_VALID\r\n");
    UARTReceiver_reset();
    playbackStartMelody();
    playbackCompletionPending = true;
    UART0_writeString("PLAYING\r\n");
}

static void UARTReceiver_service(void)
{
    uint32_t count;
    uint32_t uartErrors;
    int32_t rawByte;

    /* This call runs even when no bytes arrive. */
    UARTReceiver_checkTimeout();

    uartErrors = UARTRxErrorGet(UART0_BASE);
    if (uartErrors != 0U)
    {
        UARTRxErrorClear(UART0_BASE);
        UARTReceiver_fail("ERR_UART");
    }

    /* Bounded work keeps the clock and completion checks responsive. */
    for (count = 0U; count < RX_SERVICE_BYTE_BUDGET; count++)
    {
        if (!UARTCharsAvail(UART0_BASE))
        {
            break;
        }
        UARTReceiver_checkTimeout();
        rawByte = UARTCharGetNonBlocking(UART0_BASE);
        if (rawByte < 0)
        {
            break;
        }

        /* UART data-register bits 11:8 contain per-character error flags.
         * Inspect them before converting the word to an 8-bit payload.
         */
        uartErrors = UARTRxErrorGet(UART0_BASE);
        if ((((uint32_t)rawByte & 0x00000F00U) != 0U) ||
            (uartErrors != 0U))
        {
            UARTRxErrorClear(UART0_BASE);
            UARTReceiver_fail("ERR_UART");
            lastRxByteMs = g_transferMilliseconds;
            continue;
        }

        UARTReceiver_processByte((uint8_t)rawByte);
        if (receiverState == VALIDATE)
        {
            break;
        }
    }

    /* Also catch an overrun reported immediately after the last read. */
    if (UARTRxErrorGet(UART0_BASE) != 0U)
    {
        UARTRxErrorClear(UART0_BASE);
        UARTReceiver_fail("ERR_UART");
    }
    UARTReceiver_checkTimeout();
    UARTReceiver_validate();

    /* Never print from the audio or note-scheduler interrupt handlers. */
    if (playbackCompletionPending && !g_playbackActive)
    {
        playbackCompletionPending = false;
        UART0_writeString("DONE\r\n");
        if ((receiverState == WAIT_MAGIC) && (magicBytesMatched == 0U))
        {
            UART0_writeString("IDLE\r\n");
        }
    }
}

/* ============================================================
 * MAIN THREAD
 * ============================================================ */

void *mainThread(void *arg0)
{
    (void)arg0;
    GPIO_init();

    /* Preserve the proven audio hardware, PH2 control and timer setup. */
    Audio_init();
    AudioTimer_init();
    PlaybackTimer_init();

    /* Timer4 is reserved for the transfer clock; no SysConfig edit. */
    TransferClock_init();
    UART0_init();
    UARTRxErrorClear(UART0_BASE);
    UARTReceiver_reset();
    UART0_writeString("\r\nSMID v2 READY\r\nIDLE\r\n");

    while (1)
    {
        UARTReceiver_service();
    }
}
