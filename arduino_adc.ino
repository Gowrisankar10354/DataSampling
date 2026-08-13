/*==============================================================================
  arduino_adc.ino  —  v3.0  Block / trigger AC power-line acquisition firmware
  Target : LattePanda 3 Delta on-board co-processor = Arduino Leonardo
           (ATmega32U4, F_CPU = 16 MHz)  ->  USB-CDC to the Intel x86 host

  WHAT CHANGED FROM v2.3 (continuous streaming) -> v3.0 (block / trigger)
  ----------------------------------------------------------------------
  The host now drives a strict REQUEST/RESPONSE handshake instead of an
  open-ended stream:

    1. SAMPLE RATE : 1600 Sa/s output, OSR = 4  ->  ADC 6400 raw conversions/s.
                     16e6 / (1600*4) = 2500 timer ticks EXACTLY -> Fs is exact
                     (no OCR1A rounding).  1024 output samples span
                     1024 / (1600/50) = exactly 32 mains cycles at 50 Hz, so the
                     host's 1024-point FFT is COHERENT (50 Hz and every harmonic
                     up to the 7th land on a bin centre -> no spectral leakage).
                     OSR = 4 sums 4 raw conversions per output sample.

    2. 2 KB BLOCK  : samples are collected into a single 1024-sample buffer
                     (1024 * int16 = 2048 bytes = 2 KB).  NOTHING is transmitted
                     until the buffer is COMPLETELY FULL.  Then the whole 2 KB is
                     shipped as one CRC-framed block.

    3. TRIGGER     : after the block is sent the ADC is stopped and the firmware
                     enters ACQ_WAIT.  It does NOT sample or transmit again until
                     the host sends the resume command ("TRIG").  This is the
                     handshake that keeps the Arduino and the host in lock-step.
                     A safety TIMEOUT auto-resumes so a lost trigger byte can
                     never deadlock the board (see RESUME_TIMEOUT_MS).

    4. NO POLLING / NO BLOCKING WAIT : the resume command is handled by the
                     normal cooperative loop() drain of Serial.available().  On a
                     32U4 USB-CDC link there is NO application-level UART RX
                     interrupt — the USB stack buffers received bytes and user
                     code reads them; checking Serial.available() each loop pass
                     IS the correct non-blocking idiom (it is not a busy-spin).

  MEAN-POINT / ZERO-CROSSING  (host side, by design)
  --------------------------------------------------
  The requirement "each transmission begins at a zero-crossing" is satisfied on
  the HOST: it finds the first rising mean-crossing inside each received 1024-
  sample block and rotates the block so the displayed waveform starts there.
  This was deliberately kept OFF the MCU: gating the ISR on an exact mean-
  crossing can arm forever on a quiet / railed / DC-drifted input (a hard
  deadlock), and coherent 32-cycle sampling already makes the FFT MAGNITUDE
  spectrum phase-invariant, so the start phase does not affect the harmonics at
  all — only the look of the time-domain trace, which the host aligns for free.

  NOISE / BASELINE CALIBRATION
  ----------------------------
  With no signal present, run CALZ: it measures the DC offset (noise baseline)
  over 4096 samples with two-pass Chauvenet outlier rejection and stores it in
  EEPROM.  In MODE CAL the ISR subtracts that baseline from every OSR-sum before
  it lands in the block buffer, removing the DC offset / noise floor at the
  source.  (The host additionally removes any residual DC per block.)  A static
  baseline cannot cancel *random* noise — only the fixed offset — so it is kept
  as a single offset value, never a per-sample vector.

  SRAM NOTE  (READ THIS)
  ----------------------
  The 2 KB block buffer is 2048 of the 32U4's 2560 SRAM bytes (~80 %).  That is
  intentionally close to the edge, so the firmware reports free SRAM at boot and
  in INFO (freeRam()).  Verify on YOUR build that free RAM stays comfortably
  above the worst-case call depth (the CALZ/CALG path uses int64/float locals and
  Serial.print(float)/dtostrf).  If free RAM is marginal, shrink cmdBuf or drop a
  feature — do NOT ignore it: AVR has no stack guard, so a stack-into-.bss
  collision is SILENT data corruption, not a crash.

  KEPT FROM v2.3
  --------------
    * Hardware-timed sampling: Timer1 CTC Compare-Match-B auto-triggers the ADC
      (ADTS=101) -> zero software jitter, crystal-locked instants.
    * OSR oversampling + summation on the MCU; adaptive ADC prescaler.
    * CRC-16/CCITT-FALSE over every block (stronger than a sum byte).
    * uint16 block sequence number (drop detection).
    * ADC rail-saturation (clip) detection, latched and reported (flag b3).
    * CALZ/CALG/GETCAL/SETCAL/SAVE/MODE, AVcc|EXT reference, EEPROM persistence.

  ---------------------------------------------------------------------------
  SERIAL PROTOCOL  (USB CDC; the baud-rate value is IGNORED by CDC).
  Host -> device: ASCII lines ending in '\n' (CR ignored).

    START            arm one block: fill 1024 samples, transmit the 2 KB block,
                     then wait for TRIG.  Reply "OK START <rate> <osr> <mode>".
    TRIG             resume: collect + transmit the NEXT block (silent — the
                     block itself is the reply).  Ignored unless waiting.
    STOP             stop acquisition, return to idle.  Reply "OK STOP".
    RATE <hz>        output rate (default 1600; rate*osr <= 38000)
    OSR <1|2|4|8|16> oversampling ratio (default 4)
    CH <0..7>        ADC channel (default 7 = pin A0)
    REF <AVCC|EXT>   ADC reference (default AVCC = safe; EXT = AREF pin)
    MODE <RAW|CAL>   RAW = summed counts; CAL = baseline-subtracted (needs CALZ)
    CALZ             offset/noise-baseline calibration, input quiescent
    CALG <mVrms>     gain calibration vs a known RMS at the pin
    GETCAL           "CAL <offset_x16> <uV_per_LSB> <valid>"
    SETCAL <o> <u>   set calibration manually
    SAVE             persist calibration + rate/osr/channel/ref to EEPROM
    INFO             human-readable status (incl. free SRAM)
    *IDN?            identification string

  BINARY DATA BLOCK (device -> host, one per START/TRIG; little-endian)
    off  size  field
    0    2     0xA5 0x5A     sync
    2    1     flags         b0=CAL mode  b3=ADC saturation/clip since last block
    3    2     seq           uint16, wraps at 65536 (drop detection)
    5    2     nsamp         uint16 samples in payload (= 1024)
    7    2*n   samples       int16[n] = sum of OSR raw conversions (RAW) or that
                             sum minus the calibrated baseline (CAL)
    ...  2     crc16         CRC-16/CCITT-FALSE over bytes [2 .. 7+2n)
    total = 9 + 2n bytes;  n = 1024 -> 2057 bytes

    Volts_per_sample = value / OSR * uV_per_LSB * 1e-6   (uV_per_LSB from GETCAL)

  ANALOG FRONT-END (unchanged; must exist before the pin)
    source -> clamp -> AC-couple into mid-rail bias -> Sallen-Key LPF -> A0.
    At 1600 Sa/s the output Nyquist is 800 Hz, which covers the fundamental and
    up to the 7th harmonic (350 Hz) — the band of interest for this build.
==============================================================================*/

#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <math.h>

/* ------------------------------------------------------------------ config */
#define FW_IDN            "LattePanda-32U4 PowerLineDAQ v3.0 block/trigger"
#define F_CPU_HZ          16000000UL

/* 2 KB block: 1024 int16 samples.  DO NOT transmit until this is full. */
#define BLOCK_SAMPLES     1024U         /* 1024 * 2 = 2048 bytes = 2 KB       */
#define BLOCK_BYTES       (2U * BLOCK_SAMPLES)
#define BLK_SYNC0         0xA5
#define BLK_SYNC1         0x5A
#define BLK_HDR_LEN       7             /* 2 sync + 1 flags + 2 seq + 2 nsamp  */

/* Default 1600 Sa/s output, OSR=4 -> ADC 6400 raw conv/s (Nyquist 800 Hz).
   16e6/(1600*4)=2500 timer ticks EXACTLY, so Fs is exact.  1024 samples span
   exactly 32 mains cycles at 50 Hz -> coherent 1024-pt FFT on the host. */
#define DEF_RATE          1600UL        /* output samples per second          */
#define DEF_OSR           4
#define DEF_CHANNEL       7             /* ADC7 = pin "A0" on LattePanda      */
#define MAX_ADC_SPS       38000UL       /* ADC conversion-rate ceiling        */
#define MIN_RATE          800UL         /* Nyquist floor for the 7th harmonic */
#define MAX_RATE          10000UL

/* Trigger handshake: how long to wait for the host's TRIG before auto-resuming.
   In normal operation the host sends TRIG within ~1 s (fill = 0.64 s + a fast
   FFT/redraw), so this only ever fires to break a genuine deadlock (a lost TRIG
   byte or a stalled host) — it can never make the board hang. */
#define RESUME_TIMEOUT_MS 5000UL

#define CAL_SAMPLES       4096UL        /* total per calibration pass         */
#define CHAUVENET_Z       3.85f         /* erfc(z/sqrt2)=1/(2*4096) -> z=3.85 */
#define EE_MAGIC          0xB3AA        /* bumped for v3.0                    */

/* -------------------------------------------------------------- EEPROM map */
struct EeCal {
  uint16_t magic;
  int32_t  offset_x16;   /* DC/noise baseline in raw-ADC counts * 16 (Q4)     */
  float    uV_per_LSB;   /* microvolts per single raw ADC count               */
  uint8_t  valid;        /* bit0 offset valid, bit1 gain valid                */
  uint16_t rate_div;     /* persisted output rate / 10                        */
  uint8_t  osr;
  uint8_t  channel;
  uint8_t  useExtRef;
  uint8_t  crc;          /* sum-complement over previous bytes                */
};
EeCal EEMEM eeCal;

/* ------------------------------------------------------------ shared state */
/* The 2 KB acquisition buffer.  Filled by the ADC ISR (producer) while the
   main loop is idle in ACQ_FILL; read by the main loop (consumer) only after
   the ISR has set blockFull and the ADC has been stopped, so producer and
   consumer never touch it at the same time (no ring / wraparound needed). */
static int16_t           block[BLOCK_SAMPLES];
static volatile uint16_t blockIdx  = 0;         /* next write slot (ISR)      */
static volatile uint8_t  blockFull = 0;         /* set when idx reaches N     */
static volatile uint8_t  clipFlag  = 0;         /* ADC rail-saturation latch  */

static volatile uint16_t osAcc    = 0;          /* oversampling accumulator   */
static volatile uint8_t  osCnt    = 0;
static volatile uint8_t  osrIsr   = DEF_OSR;    /* copy used inside the ISR   */
static volatile uint8_t  calModeIsr = 0;
static volatile int32_t  offSumIsr  = 0;        /* baseline*OSR, pre-computed */

/* configuration (main-loop domain) */
static uint32_t outRate   = DEF_RATE;
static uint8_t  osr       = DEF_OSR;
static uint8_t  channel   = DEF_CHANNEL;
static uint8_t  useExtRef = 0;                  /* 0 = AVcc, 1 = AREF pin     */
static uint8_t  calMode   = 0;                  /* 0 = RAW, 1 = CAL           */

/* calibration (main-loop domain) */
static int32_t  calOffsetX16 = 0;               /* noise baseline (Q4 counts) */
static float    calUvPerLsb  = 4882.8f;         /* 5.0 V / 1024 by default    */
static uint8_t  calValid     = 0;

/* acquisition state machine */
enum AcqState { ACQ_IDLE, ACQ_FILL, ACQ_WAIT };
static AcqState acqState = ACQ_IDLE;
static bool     running  = false;               /* ADC actively converting    */
static uint16_t blkSeq   = 0;                   /* block sequence number      */
static uint32_t waitStart = 0;                  /* millis() when ACQ_WAIT began*/

/* command line buffer (kept small to preserve stack headroom next to the 2 KB
   block; longest command is "SETCAL <offset> <uV>" ~ 27 chars) */
static char     cmdBuf[30];
static uint8_t  cmdLen = 0;
static uint8_t  cmdOverflow = 0;   /* line too long: discard through newline    */

/* --------------------------------------------------------- free-SRAM probe */
/* Reports the gap between the top of the heap and the current stack pointer —
   i.e. the SRAM headroom that the 2 KB block buffer leaves for the stack.
   Printed at boot and in INFO so the tight 2 KB budget is verifiable on the
   real board (AVR has no stack-overflow trap). */
extern unsigned int __heap_start;
extern void *__brkval;
static int freeRam(void)
{
  int top;
  return (int)((int)&top - (__brkval == 0 ? (int)&__heap_start : (int)__brkval));
}

/* ------------------------------------------------------------- CRC-16 ---- */
/* CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no xorout.
   Split into an "update" form so the block CRC can span the header and the
   2 KB payload without copying the payload into a temporary buffer. */
static uint16_t crc16_upd(uint16_t crc, const uint8_t *d, uint16_t n)
{
  while (n--) {
    crc ^= (uint16_t)(*d++) << 8;
    for (uint8_t i = 0; i < 8; i++)
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}
static inline uint16_t crc16_ccitt(const uint8_t *d, uint16_t n)
{
  return crc16_upd(0xFFFF, d, n);
}

/* =========================================================================
 *  ADC INTERRUPT — runs once per raw conversion (hardware triggered).
 *  Producer: sums OSR raw conversions, then stores one output sample into the
 *  block buffer until it is full.  Never blocks.
 * ========================================================================= */
ISR(ADC_vect)
{
  TIFR1 = _BV(OCF1B);                 /* re-arm the Timer1 auto-trigger      */

  uint16_t raw = ADC;                 /* ADCL then ADCH, compiler handles    */
  if (raw >= 1020 || raw <= 3)        /* front-end pinned at a rail: clipping */
    clipFlag = 1;                     /* latched; reported in block flag b3  */
  osAcc += raw;

  if (++osCnt >= osrIsr) {
    int32_t v = osAcc;
    osAcc = 0;
    osCnt = 0;

    if (calModeIsr)
      v -= offSumIsr;                 /* subtract the noise/DC baseline      */

    if (v >  32767) v =  32767;       /* clip to int16 just in case          */
    if (v < -32768) v = -32768;

    uint16_t i = blockIdx;
    if (i < BLOCK_SAMPLES) {          /* still filling the 2 KB block         */
      block[i] = (int16_t)v;
      blockIdx = i + 1;
      if (blockIdx >= BLOCK_SAMPLES)
        blockFull = 1;                /* full -> main loop ships + waits      */
    }
    /* else: block already full; extra ISR fires are dropped until the main
       loop stops the ADC and re-arms.  In block/trigger mode this window is
       only a couple of conversions wide, so no sample of a NEW block is lost. */
  }
}

/* =========================================================================
 *  Low-level ADC / Timer setup
 * ========================================================================= */
static uint8_t adcPrescalerBits(uint32_t adcSps, bool &hsm)
{
  hsm = false;
  if (adcSps <=  9000) return 0x07;              /* /128 -> 125 kHz  */
  if (adcSps <= 18000) return 0x06;              /* /64  -> 250 kHz  */
  hsm = true;
  return 0x05;                                   /* /32  -> 500 kHz  */
}

static void adcConfigure(void)
{
  ADCSRA = 0;                                    /* stop ADC                 */

  uint8_t ref = useExtRef ? 0x00 : 0x40;         /* REFS1:0 = 00 ext, 01 AVcc */
  ADMUX  = ref | (channel & 0x07);

  bool hsm;
  uint8_t ps = adcPrescalerBits(outRate * osr, hsm);
  ADCSRB = (hsm ? _BV(ADHSM) : 0) | 0x05;        /* ADTS2:0=101: T1 comp B    */

  ADCSRA = _BV(ADEN) | _BV(ADATE) | _BV(ADIE) | ps;

  ADCSRA |= _BV(ADSC);                           /* one throw-away conversion */
  while (ADCSRA & _BV(ADSC)) { }
  (void)ADC;
  ADCSRA |= _BV(ADIF);
}

static void timerConfigure(void)
{
  uint32_t adcSps = outRate * osr;
  uint32_t top = (F_CPU_HZ / adcSps) - 1;        /* prescaler 1               */

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A  = (uint16_t)top;                        /* CTC TOP                   */
  OCR1B  = (uint16_t)top;                        /* trigger at TOP            */
  TIFR1  = 0xFF;
}

/* Start converting into a fresh block.  osrIsr/calModeIsr/offSumIsr are latched
   from the main-loop config so the ISR is self-contained. */
static void blockArm(void)
{
  noInterrupts();
  blockIdx = 0;
  blockFull = 0;
  osAcc = 0;
  osCnt = 0;
  interrupts();

  osrIsr     = osr;
  calModeIsr = calMode && (calValid & 0x01);
  offSumIsr  = ((calOffsetX16 * (int32_t)osr) + 8) >> 4;   /* baseline*OSR    */
  clipFlag   = 0;

  adcConfigure();
  timerConfigure();
  TCCR1B = _BV(WGM12) | _BV(CS10);               /* CTC, clk/1 -> GO          */
  running = true;
}

static void acqStop(void)
{
  TCCR1B = 0;                                    /* stop trigger timer        */
  ADCSRA &= (uint8_t)~(_BV(ADEN) | _BV(ADIE) | _BV(ADATE));
  running = false;
}

/* =========================================================================
 *  Ship the full 2 KB block as one CRC-framed message.
 *  Called only when blockFull is set and the ADC has been stopped, so the
 *  block buffer is stable while we stream it straight from SRAM (no 2 KB stack
 *  copy — that would blow the tight SRAM budget).
 * ========================================================================= */
static void sendBlock(void)
{
  uint8_t flags = 0;
  noInterrupts();
  if (clipFlag) { flags |= 0x08; clipFlag = 0; }
  interrupts();
  if (calModeIsr) flags |= 0x01;

  uint8_t hdr[BLK_HDR_LEN];
  hdr[0] = BLK_SYNC0;
  hdr[1] = BLK_SYNC1;
  hdr[2] = flags;
  hdr[3] = (uint8_t)blkSeq;
  hdr[4] = (uint8_t)(blkSeq >> 8);
  hdr[5] = (uint8_t)BLOCK_SAMPLES;
  hdr[6] = (uint8_t)(BLOCK_SAMPLES >> 8);

  /* CRC over flags+seq+nsamp, then over the 2 KB payload (streamed in place) */
  uint16_t crc = crc16_upd(0xFFFF, &hdr[2], BLK_HDR_LEN - 2);
  crc = crc16_upd(crc, (const uint8_t *)block, BLOCK_BYTES);

  Serial.write(hdr, BLK_HDR_LEN);
  Serial.write((const uint8_t *)block, BLOCK_BYTES);   /* AVR is little-endian */
  uint8_t tail[2] = { (uint8_t)crc, (uint8_t)(crc >> 8) };
  Serial.write(tail, 2);

  blkSeq++;
}

/* =========================================================================
 *  Block-acquisition service — called every loop() pass.  Non-blocking:
 *  advances the ACQ_FILL -> (ship) -> ACQ_WAIT -> (TRIG/timeout) -> ACQ_FILL
 *  state machine.  The resume TRIG itself is handled in handleCommand().
 * ========================================================================= */
static void serviceAcq(void)
{
  if (acqState == ACQ_FILL) {
    if (blockFull) {
      acqStop();                 /* freeze the buffer while we ship + wait    */
      sendBlock();
      acqState  = ACQ_WAIT;
      waitStart = millis();
    }
  } else if (acqState == ACQ_WAIT) {
    /* Deadlock guard: if the host's TRIG never arrives, auto-resume so the
       board can never hang waiting forever. */
    if ((uint32_t)(millis() - waitStart) >= RESUME_TIMEOUT_MS) {
      blockArm();
      acqState = ACQ_FILL;
    }
  }
}

/* =========================================================================
 *  Calibration capture — blocking (a setup command, not the stream path).
 *  Fills the SAME 2 KB block buffer repeatedly and accumulates statistics,
 *  so no extra RAM is needed.  Runs in RAW (no baseline subtract) internally.
 *  zLimit == 0 : plain capture (pass 1, gathers mean/sigma).
 *  zLimit  > 0 : CHAUVENET pass — samples with |x-refMean| > zLimit*refSd are
 *                excluded from the statistics (pass 2).
 * ========================================================================= */
static bool fillBlockBlocking(uint32_t timeoutMs)
{
  noInterrupts();
  blockIdx = 0; blockFull = 0; osAcc = 0; osCnt = 0;
  interrupts();
  clipFlag = 0;

  adcConfigure();
  timerConfigure();
  TCCR1B = _BV(WGM12) | _BV(CS10);

  uint32_t t0 = millis();
  while (!blockFull) {
    if ((uint32_t)(millis() - t0) > timeoutMs) { acqStop(); return false; }
  }
  acqStop();
  return true;
}

static bool captureStats(uint32_t n, float refMean, float refSd, float zLimit,
                         float &mean, float &var, uint32_t &kept)
{
  osrIsr     = osr;
  calModeIsr = 0;                                /* always RAW internally     */

  int64_t  sum   = 0;
  uint64_t sumSq = 0;
  uint32_t consumed = 0;
  kept = 0;
  float lim = zLimit * refSd;

  while (consumed < n) {
    if (!fillBlockBlocking(1000)) return false;  /* watchdog: no samples      */
    for (uint16_t i = 0; i < BLOCK_SAMPLES && consumed < n; i++) {
      int32_t v = block[i];
      consumed++;
      if (zLimit > 0.0f && fabsf((float)v - refMean) > lim)
        continue;                                /* Chauvenet: reject outlier */
      sum   += v;
      sumSq += (uint64_t)((int64_t)v * v);
      kept++;
    }
  }

  if (kept < n / 2) return false;                /* too many rejects = bogus  */
  float fk = (float)kept;
  mean = (float)sum / fk;
  var  = (float)sumSq / fk - mean * mean;
  if (var < 0.0f) var = 0.0f;
  return true;
}

static void doCalZero(void)
{
  float m1, v1, m2, v2; uint32_t kept;
  Serial.println(F("# CALZ pass 1/2: measuring baseline, keep input quiescent..."));
  if (!captureStats(CAL_SAMPLES, 0, 0, 0, m1, v1, kept)) {
    Serial.println(F("ERR CALZ timeout")); return;
  }
  float sd = sqrtf(v1);
  Serial.println(F("# CALZ pass 2/2: Chauvenet-filtered..."));
  if (sd > 0.0f) {
    if (!captureStats(CAL_SAMPLES, m1, sd, CHAUVENET_Z, m2, v2, kept)) {
      Serial.println(F("ERR CALZ timeout/unstable")); return;
    }
  } else {
    m2 = m1; kept = CAL_SAMPLES;                 /* perfectly quiet input     */
  }
  /* mean of OSR-summed samples -> per-raw-count baseline offset in Q4 */
  calOffsetX16 = (int32_t)((m2 / (float)osr) * 16.0f + (m2 >= 0 ? 0.5f : -0.5f));
  calValid |= 0x01;
  Serial.print(F("OK CALZ offset_x16=")); Serial.print(calOffsetX16);
  Serial.print(F(" rejected="));          Serial.println(CAL_SAMPLES - kept);
}

static void doCalGain(float mVrms)
{
  if (!(calValid & 0x01)) { Serial.println(F("ERR run CALZ first")); return; }
  if (mVrms <= 0.0f)      { Serial.println(F("ERR bad reference value")); return; }

  float m1, v1, m2, v2; uint32_t kept;
  Serial.println(F("# CALG pass 1/2: apply reference sine now..."));
  if (!captureStats(CAL_SAMPLES, 0, 0, 0, m1, v1, kept)) {
    Serial.println(F("ERR CALG timeout")); return;
  }
  float sd = sqrtf(v1);
  if (sd < 1.0f) { Serial.println(F("ERR signal too small")); return; }
  Serial.println(F("# CALG pass 2/2: Chauvenet-filtered..."));
  if (!captureStats(CAL_SAMPLES, m1, sd, CHAUVENET_Z, m2, v2, kept)) {
    Serial.println(F("ERR CALG timeout/unstable")); return;
  }
  if (v2 < 1.0f) { Serial.println(F("ERR signal too small")); return; }

  float rmsSum = sqrtf(v2);                      /* AC RMS in counts*OSR      */
  float rmsRaw = rmsSum / (float)osr;            /* raw-count RMS             */

  calUvPerLsb = (mVrms * 1000.0f) / rmsRaw;      /* uV per raw LSB            */
  calValid |= 0x02;
  Serial.print(F("OK CALG uV_per_LSB=")); Serial.print(calUvPerLsb, 3);
  Serial.print(F(" rejected="));          Serial.println(CAL_SAMPLES - kept);
}

/* =========================================================================
 *  EEPROM
 * ========================================================================= */
static uint8_t eeCrc(const EeCal &c)
{
  const uint8_t *p = (const uint8_t *)&c;
  uint8_t s = 0;
  for (uint8_t i = 0; i < sizeof(EeCal) - 1; i++) s += p[i];
  return (uint8_t)(0 - s);
}

static void eeSave(void)
{
  EeCal c;
  c.magic      = EE_MAGIC;
  c.offset_x16 = calOffsetX16;
  c.uV_per_LSB = calUvPerLsb;
  c.valid      = calValid;
  c.rate_div   = (uint16_t)(outRate / 10);
  c.osr        = osr;
  c.channel    = channel;
  c.useExtRef  = useExtRef;
  c.crc        = eeCrc(c);
  eeprom_update_block(&c, &eeCal, sizeof(c));
  Serial.println(F("OK SAVE"));
}

static void eeLoad(void)
{
  EeCal c;
  eeprom_read_block(&c, &eeCal, sizeof(c));
  if (c.magic != EE_MAGIC || c.crc != eeCrc(c)) return;   /* keep defaults    */
  calOffsetX16 = c.offset_x16;
  calUvPerLsb  = c.uV_per_LSB;
  calValid     = c.valid;
  outRate      = (uint32_t)c.rate_div * 10;
  osr          = c.osr;
  channel      = c.channel;
  useExtRef    = (c.useExtRef == 1) ? 1 : 0;
  if (outRate < MIN_RATE || outRate > MAX_RATE) outRate = DEF_RATE;
  if (osr != 1 && osr != 2 && osr != 4 && osr != 8 && osr != 16) osr = DEF_OSR;
  if (channel > 7) channel = DEF_CHANNEL;
  if (!(calUvPerLsb > 0.0f)) { calUvPerLsb = 4882.8f; calValid &= ~0x02; }
}

/* =========================================================================
 *  Command parser
 * ========================================================================= */
static void printInfo(void)
{
  Serial.print(F("INFO rate=")); Serial.print(outRate);
  Serial.print(F(" osr="));      Serial.print(osr);
  Serial.print(F(" adc_sps="));  Serial.print(outRate * osr);
  Serial.print(F(" nyquist="));  Serial.print(outRate / 2);
  Serial.print(F(" block="));    Serial.print(BLOCK_SAMPLES);
  Serial.print(F(" ch=ADC"));    Serial.print(channel);
  Serial.print(F(" ref="));      Serial.print(useExtRef ? F("EXT") : F("AVCC"));
  Serial.print(F(" mode="));     Serial.print(calMode ? F("CAL") : F("RAW"));
  Serial.print(F(" offset_x16=")); Serial.print(calOffsetX16);
  Serial.print(F(" uV_per_LSB=")); Serial.print(calUvPerLsb, 3);
  Serial.print(F(" calvalid=")); Serial.print(calValid);
  Serial.print(F(" freeram=")); Serial.print(freeRam());
  Serial.print(F(" running="));  Serial.println(running ? 1 : 0);
}

static void handleCommand(char *cmd)
{
  for (char *q = cmd; *q; q++) if (*q >= 'a' && *q <= 'z') *q -= 32;

  char *arg = strchr(cmd, ' ');
  if (arg) { *arg++ = 0; while (*arg == ' ') arg++; }

  if (!strcmp(cmd, "START")) {
    /* Arm the first block.  (The optional sample-count arg from v2.3 is gone —
       one START = one block, then TRIG-gated blocks thereafter.) */
    if (acqState != ACQ_IDLE) { Serial.println(F("ERR already running")); return; }
    blkSeq = 0;
    Serial.print(F("OK START ")); Serial.print(outRate);
    Serial.print(' ');            Serial.print(osr);
    Serial.print(' ');            Serial.println(calMode ? F("CAL") : F("RAW"));
    Serial.flush();
    blockArm();
    acqState = ACQ_FILL;
  }
  else if (!strcmp(cmd, "TRIG")) {
    /* Resume: collect + ship the NEXT block.  Silent by design — the block is
       the reply, and printing text here would interleave with the binary the
       host is parsing.  Ignored unless we are actually waiting. */
    if (acqState == ACQ_WAIT) {
      blockArm();
      acqState = ACQ_FILL;
    }
  }
  else if (!strcmp(cmd, "STOP")) {
    if (acqState == ACQ_IDLE) { Serial.println(F("ERR not running")); return; }
    acqStop();
    acqState = ACQ_IDLE;
    Serial.println(F("OK STOP"));
  }
  else if (!strcmp(cmd, "RATE")) {
    if (acqState != ACQ_IDLE) { Serial.println(F("ERR stop first")); return; }
    uint32_t r = arg ? strtoul(arg, NULL, 10) : 0;
    if (r < MIN_RATE || r > MAX_RATE || r * osr > MAX_ADC_SPS) {
      Serial.println(F("ERR range")); return;
    }
    outRate = r; Serial.println(F("OK RATE"));
  }
  else if (!strcmp(cmd, "OSR")) {
    if (acqState != ACQ_IDLE) { Serial.println(F("ERR stop first")); return; }
    uint8_t o = arg ? (uint8_t)strtoul(arg, NULL, 10) : 0;
    if ((o != 1 && o != 2 && o != 4 && o != 8 && o != 16) ||
        outRate * o > MAX_ADC_SPS) { Serial.println(F("ERR range")); return; }
    osr = o; Serial.println(F("OK OSR"));
  }
  else if (!strcmp(cmd, "CH")) {
    if (acqState != ACQ_IDLE) { Serial.println(F("ERR stop first")); return; }
    uint8_t c = arg ? (uint8_t)strtoul(arg, NULL, 10) : 255;
    if (c > 7) { Serial.println(F("ERR range")); return; }
    channel = c; Serial.println(F("OK CH"));
  }
  else if (!strcmp(cmd, "REF")) {
    if (acqState != ACQ_IDLE) { Serial.println(F("ERR stop first")); return; }
    if (arg && !strcmp(arg, "EXT"))  { useExtRef = 1; Serial.println(F("OK REF EXT")); }
    else if (arg && !strcmp(arg, "AVCC")) {
      if (useExtRef) { Serial.println(F("ERR reset required to leave EXT")); return; }
      useExtRef = 0; Serial.println(F("OK REF AVCC"));
    }
    else Serial.println(F("ERR use REF AVCC|EXT"));
  }
  else if (!strcmp(cmd, "MODE")) {
    if (arg && !strcmp(arg, "CAL")) {
      if (!(calValid & 0x01)) { Serial.println(F("ERR no offset cal")); return; }
      calMode = 1; Serial.println(F("OK MODE CAL"));
    } else if (arg && !strcmp(arg, "RAW")) {
      calMode = 0; Serial.println(F("OK MODE RAW"));
    } else Serial.println(F("ERR use MODE RAW|CAL"));
  }
  else if (!strcmp(cmd, "CALZ")) {
    if (acqState != ACQ_IDLE) { Serial.println(F("ERR stop first")); return; }
    doCalZero();
  }
  else if (!strcmp(cmd, "CALG")) {
    if (acqState != ACQ_IDLE) { Serial.println(F("ERR stop first")); return; }
    doCalGain(arg ? (float)atof(arg) : 0.0f);
  }
  else if (!strcmp(cmd, "GETCAL")) {
    Serial.print(F("CAL ")); Serial.print(calOffsetX16);
    Serial.print(' ');       Serial.print(calUvPerLsb, 3);
    Serial.print(' ');       Serial.println(calValid);
  }
  else if (!strcmp(cmd, "SETCAL")) {
    char *a2 = arg ? strchr(arg, ' ') : NULL;
    if (!arg || !a2) { Serial.println(F("ERR SETCAL <offset_x16> <uV_per_LSB>")); return; }
    *a2++ = 0;
    calOffsetX16 = strtol(arg, NULL, 10);
    calUvPerLsb  = (float)atof(a2);
    calValid     = 0x03;
    Serial.println(F("OK SETCAL"));
  }
  else if (!strcmp(cmd, "SAVE"))   { eeSave(); }
  else if (!strcmp(cmd, "INFO"))   { printInfo(); }
  else if (!strcmp(cmd, "*IDN?"))  { Serial.println(F(FW_IDN)); }
  else if (cmd[0])                 { Serial.println(F("ERR unknown cmd")); }
}

/* =========================================================================
 *  Arduino entry points
 * ========================================================================= */
void setup()
{
  DIDR0 = 0xF3;                       /* disable ADC pin digital buffers      */
  DIDR2 = 0x3F;

  eeLoad();
  Serial.begin(2000000);             /* request max nominal baud (IGNORED by USB-CDC) */

  /* Silent binary until commanded; boot banner is plain text so the Serial
     Monitor stays readable and free-SRAM headroom is visible up front. */
  Serial.print(F("# ")); Serial.print(F(FW_IDN));
  Serial.print(F("  freeram=")); Serial.println(freeRam());
}

void loop()
{
  /* host vanished mid-capture? stop cleanly instead of spinning */
  if (acqState != ACQ_IDLE && !Serial) {
    acqStop(); acqState = ACQ_IDLE;
    cmdLen = 0; cmdOverflow = 0;        /* drop any half-typed line too       */
  }

  /* 1. advance the block/trigger state machine (ship-when-full, wait-for-TRIG) */
  serviceAcq();

  /* 2. handle incoming commands — non-blocking drain of the CDC RX buffer.
        This is the "no polling / no blocking wait" resume path: TRIG is just
        another line processed here whenever the host has sent one. */
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      /* Only dispatch a line that did NOT overflow.  Previously an over-long
         line reset cmdLen mid-stream, so its TAIL was then accumulated and
         parsed as a bogus fresh command (could coincidentally spell STOP/TRIG).
         Now an overflowed line is discarded whole. */
      if (!cmdOverflow) {
        cmdBuf[cmdLen] = 0;
        handleCommand(cmdBuf);
      }
      cmdLen = 0;
      cmdOverflow = 0;
    } else if (cmdOverflow) {
      /* already too long: keep dropping until newline */
    } else if (cmdLen < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdLen++] = c;
    } else {
      cmdOverflow = 1;                  /* mark line invalid; discard to '\n'  */
    }
  }
}
