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


#define SINE_TABLE_SIZE          64U
#define SINE_TABLE_BITS          6U


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
 * This is deliberately small for the first tests.
 *
 * Our simple Twinkle MIDI is far below 1024 bytes.
 */

#define MAX_MIDI_FILE_SIZE       1024U

static uint8_t fileBuffer[MAX_MIDI_FILE_SIZE];

static uint8_t lengthBytes[4];

static uint32_t lengthBytesReceived = 0;
static uint32_t expectedLength = 0;
static uint32_t bytesReceived = 0;


typedef enum
{
    READ_LENGTH = 0,
    READ_PAYLOAD

} ReceiverState;


static ReceiverState receiverState =
    READ_LENGTH;


/* ============================================================
 * SINE-WAVE TABLE
 * ============================================================
 */

static const uint16_t sineTable[SINE_TABLE_SIZE] =
{
    8192, 8437, 8680, 8918, 9149, 9370, 9581, 9778,
    9960, 10125, 10271, 10397, 10502, 10584, 10644, 10680,
    10692, 10680, 10644, 10584, 10502, 10397, 10271, 10125,
    9960, 9778, 9581, 9370, 9149, 8918, 8680, 8437,

    8192, 7947, 7704, 7466, 7235, 7014, 6803, 6606,
    6424, 6259, 6113, 5987, 5882, 5800, 5740, 5704,
    5692, 5704, 5740, 5800, 5882, 5987, 6113, 6259,
    6424, 6606, 6803, 7014, 7235, 7466, 7704, 7947
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


#define MAX_PARSED_NOTES         64U

static MelodyEvent g_melody[MAX_PARSED_NOTES];

volatile uint32_t g_melodyLength = 0;


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
         * Use upper 6 phase bits to select
         * one of 64 sine values.
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
        centeredSample /= 4;


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
    if (g_melodyIndex >=
        g_melodyLength)
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
    /*
     * Stop any previous playback.
     */
    TimerDisable(
        TIMER3_BASE,
        TIMER_A
    );


    TimerIntClear(
        TIMER3_BASE,
        TIMER_TIMA_TIMEOUT
    );


    audioStopMidiNote();


    /*
     * Need at least one parsed note.
     */
    if (g_melodyLength == 0U)
    {
        g_playbackState =
            PLAYBACK_IDLE;


        g_playbackActive =
            false;


        return;
    }


    g_melodyIndex = 0;


    g_playbackActive =
        true;


    g_playbackState =
        PLAYBACK_IDLE;


    /*
     * Start first parsed event.
     */
    playbackStartCurrentEvent();
}


/* ============================================================
 * SIMPLE MIDI TYPE-0 PARSER
 * ============================================================
 *
 * SUPPORTED:
 *
 * - Standard MIDI File Type 0
 * - one track
 * - ticks-per-quarter-note timing
 * - tempo meta event
 * - Note On
 * - Note Off
 * - Note On velocity 0 = Note Off
 * - End Of Track
 *
 * NOT SUPPORTED YET:
 *
 * - running status
 * - chords/polyphony
 * - control changes
 * - program changes
 * - pitch bend
 * - SysEx
 * - SMPTE timing
 *
 * This is intentional for the first prototype.
 */

static bool parseSimpleMidi(
    const uint8_t *data,
    uint32_t fileLength
)
{
    uint32_t headerLength;

    uint16_t format;

    uint16_t numberOfTracks;

    uint16_t division;


    uint32_t trackOffset;

    uint32_t trackLength;

    uint32_t index;

    uint32_t trackEnd;


    /*
     * Standard MIDI default tempo:
     *
     * 500,000 microseconds / quarter note
     *
     * = 120 BPM
     */
    uint32_t tempoUsPerQuarter =
        500000U;


    /*
     * Absolute playback time while parsing.
     */
    uint64_t currentTimeUs = 0;


    /*
     * When did the active note begin?
     */
    uint64_t noteStartTimeUs = 0;


    /*
     * Used to calculate rests.
     */
    uint64_t lastNoteOffTimeUs = 0;


    bool parserNoteActive = false;


    uint8_t activeNote =
        MIDI_NOTE_NONE;


    /*
     * Start with an empty parsed melody.
     */
    g_melodyLength = 0;


    /* ========================================================
     * MIDI HEADER
     * ========================================================
     */

    if (fileLength < 22U)
    {
        return false;
    }


    /*
     * Standard MIDI file must start:
     *
     * MThd
     */
    if (memcmp(
            data,
            "MThd",
            4
        ) != 0)
    {
        return false;
    }


    /*
     * MIDI header chunk size.
     */
    headerLength =
        readBigEndian32(
            &data[4]
        );


    /*
     * Standard header body is 6 bytes.
     */
    if (headerLength != 6U)
    {
        return false;
    }


    format =
        readBigEndian16(
            &data[8]
        );


    numberOfTracks =
        readBigEndian16(
            &data[10]
        );


    division =
        readBigEndian16(
            &data[12]
        );


    /*
     * First parser supports Type 0 only.
     */
    if (format != 0U)
    {
        return false;
    }


    /*
     * Type 0 should contain one track.
     */
    if (numberOfTracks != 1U)
    {
        return false;
    }


    /*
     * Reject:
     *
     * division = 0
     *
     * or SMPTE timing.
     */
    if ((division == 0U) ||
        ((division & 0x8000U) != 0U))
    {
        return false;
    }


    /* ========================================================
     * TRACK HEADER
     * ========================================================
     */

    trackOffset =
        8U + headerLength;


    /*
     * Need room for:
     *
     * MTrk + 4-byte length
     */
    if ((trackOffset + 8U) >
        fileLength)
    {
        return false;
    }


    if (memcmp(
            &data[trackOffset],
            "MTrk",
            4
        ) != 0)
    {
        return false;
    }


    trackLength =
        readBigEndian32(
            &data[trackOffset + 4U]
        );


    /*
     * First track-data byte.
     */
    index =
        trackOffset + 8U;


    /*
     * Avoid overflow / invalid track size.
     */
    if (trackLength >
        (fileLength - index))
    {
        return false;
    }


    trackEnd =
        index + trackLength;


    /* ========================================================
     * PARSE TRACK EVENTS
     * ========================================================
     */

    while (index < trackEnd)
    {
        uint32_t deltaTicks;

        uint64_t deltaUs;

        uint8_t status;


        /*
         * Every MIDI event begins with a
         * variable-length delta time.
         */
        if (!readMidiVLQ(
                data,
                trackEnd,
                &index,
                &deltaTicks))
        {
            return false;
        }


        /*
         * Convert MIDI ticks to real time:
         *
         * deltaUs =
         *
         * deltaTicks * tempo
         * ------------------
         *      division
         */
        deltaUs =
            (
                (uint64_t)deltaTicks *
                tempoUsPerQuarter
            )
            /
            division;


        /*
         * Keep an absolute real-time position.
         */
        currentTimeUs +=
            deltaUs;


        if (index >= trackEnd)
        {
            return false;
        }


        /*
         * Read event status.
         */
        status =
            data[index++];


        /* ====================================================
         * META EVENT
         * ====================================================
         */

        if (status == 0xFFU)
        {
            uint8_t metaType;

            uint32_t metaLength;


            if (index >= trackEnd)
            {
                return false;
            }


            metaType =
                data[index++];


            /*
             * Meta-event data length is itself a VLQ.
             */
            if (!readMidiVLQ(
                    data,
                    trackEnd,
                    &index,
                    &metaLength))
            {
                return false;
            }


            if (metaLength >
                (trackEnd - index))
            {
                return false;
            }


            /*
             * ----------------------------------------
             * TEMPO
             *
             * FF 51 03 TT TT TT
             * ----------------------------------------
             */
            if (metaType == 0x51U)
            {
                if (metaLength != 3U)
                {
                    return false;
                }


                tempoUsPerQuarter =
                    ((uint32_t)data[index] << 16) |
                    ((uint32_t)data[index + 1U] << 8) |
                    ((uint32_t)data[index + 2U]);


                /*
                 * Zero tempo makes no sense.
                 */
                if (tempoUsPerQuarter == 0U)
                {
                    return false;
                }
            }


            /*
             * ----------------------------------------
             * END OF TRACK
             *
             * FF 2F 00
             * ----------------------------------------
             */
            else if (metaType == 0x2FU)
            {
                /*
                 * End Of Track should contain
                 * no data bytes.
                 */
                if (metaLength != 0U)
                {
                    return false;
                }


                /*
                 * We must not finish while a
                 * MIDI note is still active.
                 */
                if (parserNoteActive)
                {
                    return false;
                }


                /*
                 * Successful file requires
                 * at least one parsed note.
                 */
                return
                    (g_melodyLength > 0U);
            }


            /*
             * Skip the meta-event data.
             *
             * This means harmless events such as
             * a track name can exist without the
             * parser needing to understand them.
             */
            index +=
                metaLength;


            continue;
        }


        /* ====================================================
         * RUNNING STATUS
         * ====================================================
         *
         * Data bytes have bit 7 = 0.
         *
         * If we encounter one where we expected
         * a status byte, this file is using MIDI
         * running status.
         *
         * We intentionally do not support that yet.
         */

        if (status < 0x80U)
        {
            return false;
        }


        /* ====================================================
         * NOTE ON
         *
         * Status high nibble = 0x9
         * ====================================================
         */

        if ((status & 0xF0U) ==
            0x90U)
        {
            uint8_t note;

            uint8_t velocity;


            if ((index + 2U) >
                trackEnd)
            {
                return false;
            }


            note =
                data[index++];


            velocity =
                data[index++];


            /*
             * MIDI convention:
             *
             * NOTE ON + velocity 0
             *
             * is equivalent to NOTE OFF.
             */
            if (velocity == 0U)
            {
                /*
                 * It must match our current note.
                 */
                if ((!parserNoteActive) ||
                    (note != activeNote))
                {
                    return false;
                }


                /*
                 * Make sure parsed melody array
                 * has enough room.
                 */
                if (g_melodyLength >=
                    MAX_PARSED_NOTES)
                {
                    return false;
                }


                /*
                 * Store completed note.
                 */
                g_melody[
                    g_melodyLength
                ].midiNote =
                    activeNote;


                g_melody[
                    g_melodyLength
                ].durationMs =
                    microsecondsToMilliseconds(
                        currentTimeUs -
                        noteStartTimeUs
                    );


                g_melody[
                    g_melodyLength
                ].restAfterMs =
                    0U;


                g_melodyLength++;


                parserNoteActive =
                    false;


                activeNote =
                    MIDI_NOTE_NONE;


                lastNoteOffTimeUs =
                    currentTimeUs;
            }

            else
            {
                /*
                 * Current project is monophonic.
                 *
                 * Therefore another note cannot
                 * start while one is still active.
                 */
                if (parserNoteActive)
                {
                    return false;
                }


                /*
                 * Stay inside the note range
                 * supported by our oscillator.
                 */
                if ((note < MIDI_NOTE_MIN) ||
                    (note > MIDI_NOTE_MAX))
                {
                    return false;
                }


                /*
                 * If at least one previous note exists,
                 * the gap between its Note Off and this
                 * Note On is a musical REST.
                 */
                if (g_melodyLength > 0U)
                {
                    g_melody[
                        g_melodyLength - 1U
                    ].restAfterMs =
                        microsecondsToMilliseconds(
                            currentTimeUs -
                            lastNoteOffTimeUs
                        );
                }


                /*
                 * Begin new note.
                 */
                activeNote =
                    note;


                noteStartTimeUs =
                    currentTimeUs;


                parserNoteActive =
                    true;
            }


            continue;
        }


        /* ====================================================
         * NOTE OFF
         *
         * Status high nibble = 0x8
         * ====================================================
         */

        if ((status & 0xF0U) ==
            0x80U)
        {
            uint8_t note;

            uint8_t velocity;


            if ((index + 2U) >
                trackEnd)
            {
                return false;
            }


            note =
                data[index++];


            velocity =
                data[index++];


            /*
             * We aren't using Note Off velocity
             * yet.
             */
            (void)velocity;


            /*
             * Note Off must match the note that
             * is currently active.
             */
            if ((!parserNoteActive) ||
                (note != activeNote))
            {
                return false;
            }


            if (g_melodyLength >=
                MAX_PARSED_NOTES)
            {
                return false;
            }


            /*
             * Store completed note.
             */
            g_melody[
                g_melodyLength
            ].midiNote =
                activeNote;


            /*
             * Note duration =
             *
             * Note Off time - Note On time.
             */
            g_melody[
                g_melodyLength
            ].durationMs =
                microsecondsToMilliseconds(
                    currentTimeUs -
                    noteStartTimeUs
                );


            /*
             * Rest is initially zero.
             *
             * If the next Note On happens later,
             * we will fill this value then.
             */
            g_melody[
                g_melodyLength
            ].restAfterMs =
                0U;


            g_melodyLength++;


            /*
             * Explicitly end parser-side note state.
             */
            parserNoteActive =
                false;


            activeNote =
                MIDI_NOTE_NONE;


            lastNoteOffTimeUs =
                currentTimeUs;


            continue;
        }


        /*
         * Any other MIDI event is intentionally
         * unsupported by this first parser.
         */
        return false;
    }


    /*
     * Reaching the physical end without an
     * End Of Track event is treated as invalid.
     */
    return false;
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
 * RESET UART FILE RECEIVER
 * ============================================================
 */

static void UARTReceiver_reset(void)
{
    lengthBytesReceived = 0;

    expectedLength = 0;

    bytesReceived = 0;

    receiverState =
        READ_LENGTH;
}


/* ============================================================
 * MAIN THREAD
 * ============================================================
 */

void *mainThread(void *arg0)
{
    (void)arg0;


    /*
     * TI GPIO setup.
     */
    GPIO_init();


    /*
     * Configure:
     *
     * SSI2
     * DAC8311
     * DAC SYNC
     * PH2 amplifier control
     */
    Audio_init();


    /*
     * Timer2:
     *
     * 20 kHz waveform/sample generation.
     */
    AudioTimer_init();


    /*
     * Timer3:
     *
     * MIDI note durations and rests.
     */
    PlaybackTimer_init();


    /*
     * UART:
     *
     * Receive the complete MIDI file.
     */
    UART0_init();


    /*
     * Make sure receiver starts clean.
     */
    UARTReceiver_reset();


    UART0_writeString(
        "\r\nMSP432 MIDI player ready\r\n"
    );


    while (1)
    {
        /*
         * Has one UART byte arrived?
         */
        if (UARTCharsAvail(
                UART0_BASE))
        {
            uint8_t receivedByte;


            receivedByte =
                (uint8_t)UARTCharGet(
                    UART0_BASE
                );


            /* =================================================
             * STATE 1:
             *
             * RECEIVE 4-BYTE FILE LENGTH
             * =================================================
             *
             * Python sends:
             *
             * [little-endian length][raw MIDI]
             */

            if (receiverState ==
                READ_LENGTH)
            {
                lengthBytes[
                    lengthBytesReceived
                ] =
                    receivedByte;


                lengthBytesReceived++;


                /*
                 * Have all four length bytes arrived?
                 */
                if (lengthBytesReceived ==
                    4U)
                {
                    /*
                     * Convert little-endian bytes
                     * into uint32_t.
                     */
                    expectedLength =
                        ((uint32_t)lengthBytes[0]) |
                        ((uint32_t)lengthBytes[1] << 8) |
                        ((uint32_t)lengthBytes[2] << 16) |
                        ((uint32_t)lengthBytes[3] << 24);


                    /*
                     * Make sure the complete MIDI file
                     * can fit into our RAM buffer.
                     */
                    if ((expectedLength == 0U) ||
                        (expectedLength >
                         MAX_MIDI_FILE_SIZE))
                    {
                        UART0_writeString(
                            "ERROR: invalid MIDI file size\r\n"
                        );


                        UARTReceiver_reset();
                    }
                    else
                    {
                        /*
                         * Start receiving raw MIDI bytes.
                         */
                        bytesReceived = 0;


                        receiverState =
                            READ_PAYLOAD;
                    }
                }
            }


            /* =================================================
             * STATE 2:
             *
             * RECEIVE RAW MIDI FILE
             * =================================================
             */

            else if (receiverState ==
                     READ_PAYLOAD)
            {
                fileBuffer[
                    bytesReceived
                ] =
                    receivedByte;


                bytesReceived++;


                /*
                 * Has the entire MIDI file arrived?
                 */
                if (bytesReceived ==
                    expectedLength)
                {
                    uint32_t completedLength;


                    /*
                     * Save the length BEFORE resetting
                     * the receiver state.
                     */
                    completedLength =
                        expectedLength;


                    UART0_writeString(
                        "Transfer complete\r\n"
                    );


                    /*
                     * Receiver is ready for another
                     * transfer later.
                     */
                    UARTReceiver_reset();


                    /*
                     * Parse raw MIDI bytes into our
                     * simple g_melody[] structure.
                     */
                    if (parseSimpleMidi(
                            fileBuffer,
                            completedLength))
                    {
                        UART0_writeString(
                            "MIDI parse: PASS\r\n"
                        );


                        UART0_writeString(
                            "Starting playback\r\n"
                        );


                        /*
                         * Timer3 now plays the notes
                         * created by the parser.
                         */
                        playbackStartMelody();
                    }
                    else
                    {
                        /*
                         * Failed parse must always leave
                         * audio safely OFF.
                         */
                        audioStopMidiNote();


                        TimerDisable(
                            TIMER3_BASE,
                            TIMER_A
                        );


                        g_playbackActive =
                            false;


                        g_playbackState =
                            PLAYBACK_IDLE;


                        UART0_writeString(
                            "MIDI parse: FAIL\r\n"
                        );
                    }
                }
            }
        }
    }
}