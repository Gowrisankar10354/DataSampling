#!/usr/bin/env python3
# =============================================================================
#  fft_new.py  —  Intel-host side of the LattePanda power-line acquisition system
#                 (matches lattapanda_adc_new.ino v3.0 — block / trigger firmware)
#
#  GOAL
#  ----
#  Acquire a mains sine (<= 3.3 Vpp, biased to mid-rail) from the Arduino ONE
#  2 KB BLOCK AT A TIME and display, in real time:
#     LEFT  — the live time-domain voltage waveform (aligned to a zero-crossing),
#     RIGHT — the amplitude-vs-frequency spectrum with the harmonic peaks
#             labelled, plus a harmonics table for harmonics 1..7 (50..350 Hz).
#  START / STOP control acquisition; BASELINE captures the no-signal noise floor.
#
#  SIGNAL FLOW (this build)
#  ------------------------
#     ADC -> oversample&sum (OSR=4) -> 2 KB ring (1024 samples) -> USB block
#         -> Python -> DC removal -> zero-crossing align -> FFT
#         -> /4 magnitude correction -> display -> TRIG back to Arduino
#
#  DESIGN NOTES
#  ------------
#  * BLOCK / REQUEST-RESPONSE.  The firmware fills a 1024-sample (2 KB) buffer,
#    ships the WHOLE block as one CRC-framed message, then WAITS.  After this
#    host processes and displays a block it sends "TRIG" to release the next one.
#    That handshake keeps the two ends in lock-step (the firmware also has a
#    safety timeout so a lost TRIG can never deadlock it).
#
#  * COHERENT SAMPLING.  fs = 1600 Sa/s, N_FFT = 1024.  1024 / (1600/50) = 32.0
#    mains cycles EXACTLY, so 50 Hz and every harmonic up to the 7th land on an
#    FFT bin centre -> no spectral leakage.  Nyquist = 800 Hz covers the 7th
#    harmonic (350 Hz) with margin.
#
#  * OSR = 4 SUMMATION & THE /4 CORRECTION.  The firmware SUMS 4 raw conversions
#    per output sample, so every received value is 4x the single-conversion code
#    (~11 effective bits).  The FFT is run on that summed data and the SPECTRUM
#    is divided by 4 (== / OSR) — a SINGLE magnitude correction, applied in ONE
#    place (see DSP.analyze).  Do NOT also divide the samples by OSR before the
#    FFT: because the FFT is linear that would divide twice (a 16x error).
#
#  * DC REMOVAL.  Authoritative per-block mean subtraction before the FFT
#    (harmonics never touch bin 0, so this is exact and tracks slow drift).  A
#    separately-captured no-signal BASELINE (DC + noise floor) is reported and
#    subtracted as well; on the firmware side, MODE CAL subtracts the CALZ
#    baseline at the source.
#
#  * ZERO-CROSSING ALIGNMENT (host side).  Each received block is rotated so its
#    first rising mean-crossing becomes sample 0 — "each transmission begins at a
#    zero-crossing", done on the host.  Because the block is an integer number of
#    cycles this is a circular shift, so it leaves the FFT MAGNITUDE spectrum
#    unchanged and only steadies the displayed trace.  (Kept off the MCU on
#    purpose: gating the ISR on a mean-crossing can arm forever on a quiet/railed
#    input — a deadlock — and buys nothing for the coherent spectrum.)
#
#  Usage:  python fft_new.py                       (AVcc reference, no wiring)
#          python fft_new.py -p COM5 --ref ext     (only if 3.3 V wired to AREF)
#          python fft_new.py --calz --calg 810     (one-time gain calibration)
#          python fft_new.py --selftest            (offline DSP self-test, no board)
#  Python 3.8+.
# =============================================================================

# ---- dependency bootstrap (auto-install if missing) ------------------------
import importlib, subprocess, sys
for _pkg, _imp in [("numpy", "numpy"), ("scipy", "scipy"),
                   ("pyserial", "serial"), ("matplotlib", "matplotlib")]:
    try:
        importlib.import_module(_imp)
    except ImportError:
        subprocess.check_call([sys.executable, "-m", "pip", "install", _pkg])

import struct, threading, time, argparse, queue
import numpy as np
from scipy.signal.windows import flattop
from scipy.special import erfc

# ============================ CONFIG ========================================
LINE_HZ     = 50.0                   # nominal mains fundamental
N_HARM      = 7                      # analyse harmonics 1..7 (50..350 Hz)
N_FFT       = 1024                   # FFT block = ONE Arduino block.  2 KB =
                                     # 2048 BYTES = 1024 int16 SAMPLES, so the
                                     # FFT length (in SAMPLES) is 1024, NOT 2048.
                                     # Must equal the firmware BLOCK_SAMPLES; a
                                     # mismatch makes flattop(N_FFT) broadcast
                                     # against the 1024-pt block and crash.
DEF_RATE    = 1600                   # device output rate to request (Sa/s)
DEF_OSR     = 4                      # device oversampling ratio to request
DEF_VREF    = 3.3                    # nominal reference volts (self-test scaling)
F_CPU       = 16_000_000             # 32U4 clock, for exact-fs computation
RX_BUF      = 1 << 20                # 1 MB OS serial RX buffer
SYNC        = b"\xA5\x5A"
HDR_FMT     = "<BHH"                 # flags, seq(u16), nsamp(u16)  (after sync)
HDR_LEN     = 2 + struct.calcsize(HDR_FMT)      # 7 bytes incl. sync
MAX_NSAMP   = 4096                   # sanity clamp when scanning frames

# firmware block flag bits: b0=CAL mode, b3=clip since last block.
FLAG_CAL    = 0x01
FLAG_CLIP   = 0x08

# ============================ CRC-16 ========================================
def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE — identical to the Arduino implementation."""
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc & 0xFFFF

# ========================= DEVICE HANDSHAKE =================================
def read_line(ser, timeout=3.0):
    old = ser.timeout
    ser.timeout = timeout
    line = ser.readline().decode(errors="replace").strip()
    ser.timeout = old
    return line

def cmd(ser, c, timeout=3.0):
    ser.write((c + "\n").encode())
    r = read_line(ser, timeout)
    print(f"  > {c:<18} < {r}")
    return r

def expect_ok(ser, c):
    """Send a config command and FAIL LOUDLY if the device rejects it."""
    r = cmd(ser, c)
    if r.startswith("ERR"):
        sys.exit(f"device rejected '{c}': {r}\n"
                 "  (check rate*osr <= 38000 and rate in 800..10000)")
    return r

def wait_ok(ser, what, timeout=15.0):
    """Read progress lines ('# ...') until OK/ERR — used for CALZ/CALG."""
    t0 = time.time()
    while time.time() - t0 < timeout:
        r = read_line(ser, timeout=5.0)
        if r:
            print("  <", r)
        if r.startswith("OK"):
            return True
        if r.startswith("ERR"):
            return False
    print(f"!! {what}: no OK/ERR within {timeout}s")
    return False

def parse_info(line):
    d = {}
    for tok in line.split():
        if "=" in tok:
            k, v = tok.split("=", 1)
            d[k] = v
    return d

def fs_actual(rate, osr):
    """Exact output rate from the firmware's Timer1 integer arithmetic:
    period = floor(F_CPU / (rate*osr)); output = F_CPU/period/osr.
    For 1600 Sa/s @ OSR 4: 16e6/6400 = 2500 exactly -> fs = 1600.000 Sa/s."""
    period = F_CPU // (rate * osr)
    return F_CPU / period / osr

def setup_device(ser, args):
    """Configure the device and leave it STOPPED (the reader drives START).
    Streams RAW; all DC removal / scaling happens on the host.
    Returns (fs, osr, volts_per_count)."""
    ser.reset_input_buffer()
    idn = cmd(ser, "*IDN?")
    if "PowerLineDAQ" not in idn:
        ser.write(b"STOP\n"); time.sleep(0.5); ser.reset_input_buffer()
        idn = cmd(ser, "*IDN?")
        if "PowerLineDAQ" not in idn:
            sys.exit(f"Unexpected/absent *IDN? reply: {idn!r} — is the v3.0 "
                     "firmware flashed?")

    ser.write(b"STOP\n"); time.sleep(0.2); ser.reset_input_buffer()

    # Order matters: OSR=1 first so RATE can never transiently exceed the ADC
    # ceiling, then the target rate, then the target OSR.  Each step checked.
    expect_ok(ser, "OSR 1")
    expect_ok(ser, f"RATE {args.rate}")
    expect_ok(ser, f"OSR {args.osr}")

    if args.ref.upper() == "EXT":
        r = cmd(ser, "REF EXT")
        if ("EXT" not in r) and ("reset" not in r.lower()):
            print("   !! could not select EXT reference:", r)
    else:
        r = cmd(ser, "REF AVCC")
        if "reset" in r.lower():
            print("   !! board is latched in EXT reference from a previous run — "
                  "UNPLUG/REPLUG the board to return to AVcc, then rerun.")

    if args.calz:
        print("  # offset/noise-baseline calibration: keep the input quiescent")
        ser.write(b"CALZ\n")
        if not wait_ok(ser, "CALZ"):
            sys.exit("CALZ failed")
    if args.calg:
        print(f"  # gain calibration: apply the known {args.calg} mVrms sine now")
        ser.write(f"CALG {args.calg}\n".encode())
        if not wait_ok(ser, "CALG"):
            sys.exit("CALG failed")
    if args.calz or args.calg:
        cmd(ser, "SAVE")

    # Stream RAW: the host owns DC removal + the /4 magnitude correction.
    expect_ok(ser, "MODE RAW")

    info = parse_info(cmd(ser, "INFO"))
    rate = int(info.get("rate", args.rate))
    osr = int(info.get("osr", args.osr))
    ref = info.get("ref", "?")
    uv = float(info.get("uV_per_LSB", 4882.8))
    cv = int(info.get("calvalid", 0))
    freeram = info.get("freeram", "?")

    if cv & 0x02:
        vpc = uv * 1e-6
        src = f"gain-cal ({uv:.1f} uV/LSB)"
    else:
        vpc = args.vref / 1024.0
        src = f"Vref/1024 ({args.vref} V)"
    print(f"  ADC reference = {ref}   volts/count via {src}")
    print(f"  device free SRAM = {freeram} bytes  (2 KB block leaves this for "
          "the stack — watch it stays comfortably > 0)")
    if args.ref.upper() == "EXT" and ref != "EXT":
        print(f"  !! WARNING: device reference is {ref}, not EXT — the {args.vref} V "
              "scaling may be wrong.  Reset the board and retry.")
    # scale disclosure for the GUI: what volts/count assumption is in use, and
    # whether it is a calibrated (gain-cal) or a nominal (Vref/1024) figure.
    calibrated = bool(cv & 0x02)
    scale_note = (f"CALG {uv:.0f} uV/LSB" if calibrated
                  else f"{ref} nominal {args.vref:.2f} V (Vref/1024)")
    return fs_actual(rate, osr), osr, vpc, scale_note, calibrated

# ================== BLOCK READER (request / response) ======================
class BlockReader(threading.Thread):
    """Owns the port.  Drives the block/trigger handshake:
       START -> firmware ships one 2 KB block -> stored here -> GUI displays ->
       GUI calls request_next() -> "TRIG" -> firmware ships the next block ...
    Commands are delivered through a thread-safe queue so ALL serial writes
    happen on this one thread (no cross-thread write races)."""
    def __init__(self, ser, fs, osr, vpc):
        super().__init__(daemon=True)
        self.ser = ser
        self.fs = fs
        self.osr = osr
        self.vpc = vpc
        self.running = True
        self.acquiring = False
        self.cmd_q = queue.Queue()
        self._buf = bytearray()
        self._last_seq = None

        # latest complete block + diagnostics (guarded by lock)
        self.lock = threading.Lock()
        self.latest = None            # np.float64 codes (OSR sums)
        self.latest_flags = 0
        self.latest_seq = None
        self.block_count = 0          # increments on every new block
        self.clip = 0
        self.bad_crc = 0
        self.lost = 0

        # host-side no-signal baseline (real volts) captured by the BASELINE btn
        self.baseline = 0.0
        self.noise_rms = 0.0

    # --- public, called from the GUI thread ---
    def start_acq(self):  self.cmd_q.put("start")
    def stop_acq(self):   self.cmd_q.put("stop")
    def request_next(self):
        """Send the resume TRIG for the next block — called AFTER display."""
        self.cmd_q.put("trig")
    def stop(self):       self.running = False

    def snapshot(self):
        with self.lock:
            if self.latest is None:
                return None
            return dict(count=self.block_count, codes=self.latest.copy(),
                        flags=self.latest_flags, seq=self.latest_seq,
                        clip=self.clip, bad_crc=self.bad_crc, lost=self.lost)

    # --- internals ---
    def _begin(self):
        with self.lock:
            self.latest = None
            self.block_count = 0
            self.clip = self.bad_crc = self.lost = 0
        self._buf.clear()
        self._last_seq = None
        try:
            self.ser.reset_input_buffer()
            self.ser.write(b"START\n")            # arm the first block
        except Exception as e:
            print("!! START failed:", e)
            self.acquiring = False
            return
        self.acquiring = True

    def _end(self):
        try:
            self.ser.write(b"STOP\n")
            time.sleep(0.1)
            self.ser.reset_input_buffer()
        except Exception:
            pass
        self.acquiring = False
        self._buf.clear()                 # drop stale bytes so the next START
        self._last_seq = None             # parses only fresh blocks

    def _trig(self):
        if not self.acquiring:
            return
        try:
            self.ser.write(b"TRIG\n")             # release the next block
        except Exception as e:
            print("!! TRIG failed:", e)

    def _store(self, codes, flags, seq):
        with self.lock:
            self.latest = codes
            self.latest_flags = flags
            self.latest_seq = seq
            self.block_count += 1
            if flags & FLAG_CLIP:
                self.clip += 1
            if self._last_seq is not None and seq != ((self._last_seq + 1) & 0xFFFF):
                self.lost += 1
            self._last_seq = seq

    def _parse(self):
        """Drain self._buf of complete 2 KB blocks.  ASCII noise (boot banner,
        'OK ...' replies) never matches the 0xA5 0x5A sync + CRC, so it is
        skipped harmlessly."""
        buf = self._buf
        while True:
            i = buf.find(SYNC)
            if i < 0:
                del buf[:-1]; break                # keep last byte (maybe 0xA5)
            if i:
                del buf[:i]
            if len(buf) < HDR_LEN:
                break
            flags, seq, nsamp = struct.unpack_from(HDR_FMT, buf, 2)
            if not (1 <= nsamp <= MAX_NSAMP):
                del buf[:1]; continue
            need = HDR_LEN + 2 * nsamp + 2
            if len(buf) < need:
                break
            pkt = bytes(buf[:need])
            crc_rx, = struct.unpack_from("<H", pkt, HDR_LEN + 2 * nsamp)
            if crc16_ccitt(pkt[2:HDR_LEN + 2 * nsamp]) != crc_rx:
                with self.lock:
                    self.bad_crc += 1
                del buf[:1]; continue
            del buf[:need]
            codes = np.frombuffer(pkt, dtype="<i2", count=nsamp,
                                  offset=HDR_LEN).astype(np.float64)
            self._store(codes, flags, seq)

    # Hard cap on the parse buffer: a full block frame is 9 + 2*1024 = 2057 B, so
    # a few frames is plenty of slack.  A false 0xA5 0x5A sync with a large nsamp,
    # or the host falling behind, could otherwise let _buf grow unbounded; if it
    # exceeds the cap we drop the OLDEST bytes (never the freshest, which may hold
    # a partial in-progress frame).
    _BUF_CAP = 16384

    def _pump(self):
        try:
            chunk = self.ser.read(self.ser.in_waiting or 1)
        except Exception:
            return
        if chunk:
            self._buf.extend(chunk)
            if len(self._buf) > self._BUF_CAP:
                del self._buf[:len(self._buf) - self._BUF_CAP]
            self._parse()

    def run(self):
        while self.running:
            try:
                c = self.cmd_q.get_nowait()
            except queue.Empty:
                c = None
            if   c == "start": self._begin()
            elif c == "stop":  self._end()
            elif c == "trig":  self._trig()

            if self.acquiring:
                self._pump()
            else:
                time.sleep(0.02)

# ============================ DSP PIPELINE ==================================
def chauvenet_spike_guard(x, clipping):
    """NARROW spike guard (not a filter): replace isolated samples whose
    deviation is so large that fewer than 0.5 are expected in a block of this
    size, by linear interpolation.  Skipped entirely when the ADC is clipping.
    Returns (cleaned copy, n_replaced)."""
    if clipping:
        return x, 0
    mu, sd = x.mean(), x.std()
    if sd <= 0.0:
        return x, 0
    bad = erfc(np.abs(x - mu) / sd / np.sqrt(2.0)) * x.size < 0.5
    nbad = int(bad.sum())
    if nbad == 0 or nbad > x.size // 100:
        return x, 0
    idx = np.arange(x.size)
    x = x.copy()
    x[bad] = np.interp(idx[bad], idx[~bad], x[~bad])
    return x, nbad

class DSP:
    """Coherent-sampling harmonic analyser for ONE 1024-sample block.  All
    amplitudes are peak-volts read from a single spectrum array, so plot,
    markers and table always agree."""
    def __init__(self, fs, osr, volts_per_count, n_harm):
        self.fs = fs
        self.osr = osr
        self.vpc = volts_per_count
        self.nh = n_harm
        self.win = flattop(N_FFT)          # flat passband => accurate peak amplitude
        self.cg = self.win.sum()           # coherent gain
        self.freqs = np.fft.rfftfreq(N_FFT, d=1.0 / fs)
        self.binhz = fs / N_FFT
        # scale disclosure (set by main from setup_device); defaults keep the
        # self-test and any headless use working without the device handshake.
        self.scale_note = f"nominal {volts_per_count*1024:.2f} V (Vref/1024)"
        self.calibrated = False
        self.vfs = volts_per_count * 1024.0

    def real_volts(self, codes):
        """Per-sample REAL volts = (OSR sum / OSR) * volts_per_count."""
        return (codes / self.osr) * self.vpc

    @staticmethod
    def _align_zero_crossing(v):
        """Rotate a DC-removed block so its first RISING mean-crossing is sample
        0.  Coherent (integer-cycle) block => circular shift => FFT magnitude
        unchanged; this only steadies the displayed waveform.  A small hysteresis
        band rejects noise crossings.  Returns (rolled, crossing_index or -1)."""
        peak = float(np.max(np.abs(v))) if v.size else 0.0
        if peak <= 0.0:
            return v, -1
        eps = 0.02 * peak
        rising = np.where((v[:-1] < -eps) & (v[1:] > eps))[0]
        if rising.size == 0:
            return v, -1
        k = int(rising[0]) + 1
        return np.roll(v, -k), k

    def analyze(self, codes, clipping=False, baseline=0.0):
        # --- RAW input diagnostics (real volts, BEFORE any removal) ----------
        # NOTE: over-range is NOT detected here.  A host-side "railed" test on
        # codes/osr uses the OSR *average*, which both false-trips on a healthy
        # near-full-scale signal AND can average away a genuine per-conversion
        # rail.  The firmware's per-conversion clipFlag (packet FLAG_CLIP) is the
        # authoritative over-range signal; the GUI uses that (r["clipping"]).
        v_real = self.real_volts(codes)
        raw_min, raw_mean, raw_max = float(v_real.min()), float(v_real.mean()), float(v_real.max())
        vpp_raw = raw_max - raw_min

        # --- FFT path works on the OSR-SUMMED data (as received) so the single
        #     /4 (== /OSR) magnitude correction lands on the SPECTRUM only. -----
        vs = codes * self.vpc                            # OSR-sum volts (4x real)
        vs = vs - baseline * self.osr                    # subtract no-signal baseline
        vs, replaced = chauvenet_spike_guard(vs, clipping)
        vs = vs - vs.mean()                              # authoritative DC removal
        vs, zc = self._align_zero_crossing(vs)           # host-side zero-cross align

        w = vs / self.osr                                # real-volt waveform for display
        rms_time = float(np.sqrt(np.mean(w ** 2)))

        X = np.fft.rfft(vs * self.win)
        # peak-volts per bin: |X|*(2/cg) is the OSR-summed amplitude; dividing by
        # OSR (=4) is THE magnitude correction for the 4x oversample summation —
        # applied here ONCE and nowhere else.
        spec = np.abs(X) * (2.0 / self.cg) / self.osr

        # fundamental: strongest bin in a +/-30% band around 50 Hz, refined by a
        # magnitude-weighted centroid over its main lobe.
        band = (self.freqs > LINE_HZ * 0.7) & (self.freqs < LINE_HZ * 1.3)
        k1 = int(np.argmax(np.where(band, spec, 0.0)))
        lo1, hi1 = max(1, k1 - 3), min(len(spec), k1 + 4)
        wsum = spec[lo1:hi1].sum()
        f1 = float((self.freqs[lo1:hi1] * spec[lo1:hi1]).sum() / wsum) if wsum > 0 else LINE_HZ
        if not (LINE_HZ * 0.7 < f1 < LINE_HZ * 1.3):
            f1 = LINE_HZ

        harm = []
        for h in range(1, self.nh + 1):
            kc = int(round(h * f1 / self.binhz))
            if kc >= len(spec) - 1:                     # above Nyquist: skip
                break
            lo, hi = max(1, kc - 2), min(len(spec), kc + 3)
            k = lo + int(np.argmax(spec[lo:hi]))
            harm.append((h, float(self.freqs[k]), float(spec[k]), k))

        a1 = harm[0][2] if harm and harm[0][2] > 1e-12 else 1e-12
        thd = float(np.sqrt(sum(a ** 2 for _, _, a, _ in harm[1:])) / a1)

        return dict(v=w, f1=f1, a1=a1, thd=thd, rms_time=rms_time,
                    rms_fund=a1 / np.sqrt(2), harm=harm, spec=spec,
                    replaced=replaced, clipping=clipping, zc=zc,
                    raw_min=raw_min, raw_mean=raw_mean, raw_max=raw_max,
                    vpp_raw=vpp_raw)

# ============================ LIVE DISPLAY ==================================
def run_gui(reader, dsp):
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Button
    from matplotlib.animation import FuncAnimation

    plt.style.use("dark_background")
    fig = plt.figure(figsize=(13.5, 8.0))
    gs = fig.add_gridspec(3, 2, width_ratios=[1.15, 1.0],
                          height_ratios=[1.0, 0.55, 1.05],
                          left=0.07, right=0.985, top=0.90, bottom=0.12,
                          hspace=0.45, wspace=0.18)
    ax_t = fig.add_subplot(gs[:, 0])
    ax_f = fig.add_subplot(gs[0, 1])
    ax_s = fig.add_subplot(gs[1, 1]); ax_s.axis("off")
    ax_tab = fig.add_subplot(gs[2, 1]); ax_tab.axis("off")

    fig.suptitle("LattePanda 50 Hz Power-Line Harmonic Monitor   "
                 f"(fs={dsp.fs:.1f} Sa/s, block=1024, coherent 32-cycle FFT, "
                 f"Nyq {dsp.fs/2:.0f} Hz)", fontsize=12)

    # --- left: live time-domain voltage waveform (one full block) ---
    show = N_FFT
    tms = np.arange(show) / dsp.fs * 1000.0
    (l_wave,) = ax_t.plot(tms, np.zeros(show), lw=1.0, color="tab:cyan")
    ax_t.axhline(0.0, color="gray", lw=0.6, ls="--")
    ax_t.set_title("Live voltage waveform (DC-removed, zero-crossing aligned)", fontsize=10)
    ax_t.set_xlabel("time (ms)"); ax_t.set_ylabel("Voltage (V)")
    ax_t.grid(alpha=0.2); ax_t.set_xlim(0, tms[-1])

    # --- right-top: amplitude spectrum with labelled harmonic peaks ---
    fmax = min(1.15 * N_HARM * LINE_HZ, dsp.fs / 2)
    fmask = dsp.freqs <= fmax
    (l_spec,) = ax_f.plot(dsp.freqs[fmask], np.zeros(fmask.sum()), lw=1.0, color="tab:orange")
    hmarks = ax_f.scatter([], [], c="red", s=28, zorder=5)
    hann = [ax_f.annotate("", (0, 0), textcoords="offset points", xytext=(0, 6),
                          ha="center", fontsize=7.5, color="red", visible=False)
            for _ in range(N_HARM)]
    ax_f.set_title("Amplitude vs frequency (harmonics 1..7 marked)", fontsize=10)
    ax_f.set_xlabel("Frequency (Hz)"); ax_f.set_ylabel("Amplitude (V)")
    ax_f.set_xlim(0, fmax); ax_f.grid(alpha=0.2)

    # --- right-mid: status readout ---
    status = ax_s.text(0.0, 1.0, "", va="top", ha="left", family="monospace",
                       fontsize=8.5, transform=ax_s.transAxes)

    # --- right-bottom: persistent harmonics table ---
    # %F0  = amplitude as a percent OF THE FUNDAMENTAL (h1 is 100% by definition —
    #        it is the reference, not a bug).
    # %tot = amplitude as a percent of the TOTAL harmonic RMS (sqrt(sum a^2)); this
    #        IS a share of the whole, so h1 reads < 100% whenever other harmonics
    #        are present.  Both come from the same peak-volts spectrum array.
    col_labels = ["h", "Freq (Hz)", "Amp (V)", "%F0", "%tot"]
    tbl = ax_tab.table(cellText=[["", "", "", "", ""] for _ in range(N_HARM)],
                       colLabels=col_labels, cellLoc="right", colLoc="center",
                       bbox=[0.0, 0.0, 1.0, 1.0])
    tbl.auto_set_font_size(False); tbl.set_fontsize(8.5)
    for (rr, _cc), cell in tbl.get_celld().items():
        cell.set_edgecolor("#444444")
        if rr == 0:
            cell.set_facecolor("#26364d"); cell.set_text_props(weight="bold", color="white")
        else:
            cell.set_facecolor("#111417")
    for i in range(N_HARM):
        tbl[(i + 1, 0)].get_text().set_text(str(i + 1))

    # --- buttons: START / STOP / BASELINE ---
    bax_start = fig.add_axes([0.07, 0.02, 0.11, 0.05])
    bax_stop  = fig.add_axes([0.19, 0.02, 0.11, 0.05])
    bax_base  = fig.add_axes([0.31, 0.02, 0.15, 0.05])
    b_start = Button(bax_start, "START", color="#264d26", hovercolor="#3a7a3a")
    b_stop  = Button(bax_stop,  "STOP",  color="#4d2626", hovercolor="#7a3a3a")
    b_base  = Button(bax_base,  "BASELINE", color="#26364d", hovercolor="#3a5a7a")
    b_start.on_clicked(lambda _e: reader.start_acq())
    b_stop.on_clicked(lambda _e: reader.stop_acq())

    def capture_baseline(_e):
        """Store the current block's mean (DC) + std (noise floor) as the
        no-signal baseline.  Press with NO signal applied."""
        snap = reader.snapshot()
        if snap is None:
            return
        v = dsp.real_volts(snap["codes"])
        reader.baseline = float(v.mean())
        reader.noise_rms = float(v.std())
        print(f"# baseline captured: DC={reader.baseline:+.4f} V  "
              f"noise_rms={reader.noise_rms:.4f} V")
    b_base.on_clicked(capture_baseline)

    state = {"count": -1}

    def update_table(harm, a1):
        # total harmonic RMS basis for %tot: rms_h = a/sqrt(2); total = sqrt(sum a^2)/sqrt(2)
        # so %tot = 100*a/sqrt(sum a^2) — h1 < 100% once any harmonic is nonzero.
        rss = np.sqrt(sum(a ** 2 for _, _, a, _ in harm)) if harm else 0.0
        for i in range(N_HARM):
            row = i + 1
            if i < len(harm):
                _h, fh, a, _ = harm[i]
                tbl[(row, 1)].get_text().set_text(f"{fh:7.1f}")
                tbl[(row, 2)].get_text().set_text(f"{a:8.5f}")
                tbl[(row, 3)].get_text().set_text(f"{100*a/a1:6.1f}")
                tbl[(row, 4)].get_text().set_text(f"{(100*a/rss) if rss > 0 else 0:6.1f}")
            else:
                for c in (1, 2, 3, 4):
                    tbl[(row, c)].get_text().set_text("")

    def update(_):
        if not reader.acquiring:
            status.set_text(" [STOPPED]\n press START to acquire blocks\n"
                            " (BASELINE captures the no-signal noise floor)")
            return

        snap = reader.snapshot()
        if snap is None or snap["count"] == state["count"]:
            return                                       # no NEW block yet
        state["count"] = snap["count"]

        clipping = bool(snap["flags"] & FLAG_CLIP)
        r = dsp.analyze(snap["codes"], clipping=clipping, baseline=reader.baseline)

        # left: DC-removed, zero-crossing-aligned waveform.  No red screen / no
        # "NO AC SIGNAL" overlay: over-range is reported as a quiet text note
        # (below), driven by the firmware's authoritative per-conversion clip
        # flag — a healthy near-full-scale trace is no longer flagged as broken.
        w = r["v"]
        l_wave.set_ydata(w)
        m = max(1e-3, np.max(np.abs(w)) * 1.25)
        ax_t.set_ylim(-m, m)

        # right-top: spectrum + labelled peaks
        sp = r["spec"][fmask]
        l_spec.set_ydata(sp)
        ax_f.set_ylim(0, max(1e-3, sp.max() * 1.25))
        hx = [fh for _, fh, _, _ in r["harm"]]
        hy = [a for _, _, a, _ in r["harm"]]
        hmarks.set_offsets(np.column_stack([hx, hy]) if hx else np.empty((0, 2)))
        for i, ann in enumerate(hann):
            if i < len(r["harm"]):
                h, fh, a, _ = r["harm"][i]
                if a > r["a1"] * 0.02 or h == 1:
                    ann.set_text(f"h{h}"); ann.xy = (fh, a); ann.set_visible(True)
                else:
                    ann.set_visible(False)
            else:
                ann.set_visible(False)

        # right-mid/bottom: status + table
        update_table(r["harm"], r["a1"])
        zc_tag = f"{r['zc']}" if r["zc"] >= 0 else "none"
        # honest, non-alarming over-range note (firmware clip flag), not a red screen
        clip_note = ("  clip: input near/over full-scale — reduce amplitude"
                     if r["clipping"] else "")
        cal_hint = "" if dsp.calibrated else "  (run --calg for accurate V)"
        status.set_text(
            " state : RUN (block/trigger)\n" +
            " block #%d  seq=%s  zero-cross@%s\n" % (snap["count"], snap["seq"], zc_tag) +
            " scale : %s%s\n" % (dsp.scale_note, cal_hint) +
            " raw in : mean=%+.3f V  range %+.3f..%+.3f V  Vpp=%.3f V (FS 0..%.2f V)%s\n"
            % (r["raw_mean"], r["raw_min"], r["raw_max"], r["vpp_raw"], dsp.vfs, clip_note) +
            " f0=%7.3f Hz   THD(2..%d)=%6.2f %%\n" % (r["f1"], dsp.nh, r["thd"] * 100) +
            " A1=%7.4f Vpk (%7.4f Vrms)   Vpp=%7.4f V\n" % (r["a1"], r["rms_fund"], 2 * r["a1"]) +
            " RMS=%7.4f V   baseline DC=%+7.4f V  noise=%.4f V\n"
            % (r["rms_time"], reader.baseline, reader.noise_rms) +
            " clip=%d  bad-crc=%d  lost=%d  spike-rej=%d"
            % (snap["clip"], snap["bad_crc"], snap["lost"], r["replaced"]))

        # HANDSHAKE: block processed + displayed -> release the next one.
        reader.request_next()

    # 60 ms tick: fast enough to pick up each ~640 ms block promptly and issue
    # the TRIG, but light on the GIL so the reader thread keeps draining USB.
    ani = FuncAnimation(fig, update, interval=60, blit=False, cache_frame_data=False)
    plt.show()
    return ani

# ============================== PORT PICK ===================================
def find_port():
    import serial.tools.list_ports
    for p in serial.tools.list_ports.comports():
        if (p.vid == 0x2341) or ("leonardo" in (p.description or "").lower()) \
           or ("arduino" in (p.description or "").lower()):
            return p.device
    return None

def list_ports_str():
    import serial.tools.list_ports
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        return "   (none — the board is not enumerating; check USB + upload)"
    return "\n".join(f"   {p.device:8} {p.description}" for p in ports)

def open_serial(port, baud=2_000_000, attempts=3):
    """Open the CDC port, retrying briefly.  (CDC ignores the baud value; the
    high number just mirrors the firmware's 'max baud' request.)"""
    import serial
    last = None
    for i in range(attempts):
        try:
            ser = serial.Serial(port, baud, timeout=2)
            try:
                ser.set_buffer_size(rx_size=RX_BUF, tx_size=4096)   # Windows
            except Exception:
                pass
            time.sleep(1.5); ser.reset_input_buffer()
            return ser
        except serial.SerialException as e:
            last = e; msg = str(e).lower()
            if "access is denied" in msg or "permission" in msg:
                print(f"[{i+1}/{attempts}] {port} busy (Access denied) — retrying...")
                time.sleep(1.5); continue
            if "filenotfound" in msg or "no such" in msg or "cannot find" in msg:
                break
            raise
    denied = last and ("access is denied" in str(last).lower()
                       or "permission" in str(last).lower())
    print("\n" + "=" * 70)
    print(f"ERROR: could not open {port}\n  {last}\n" + "-" * 70)
    if denied:
        print("'Access is denied' means the port EXISTS but is already open.")
        print("  1. Close the Arduino IDE Serial Monitor / Serial Plotter.")
        print("  2. Close any other copy of this script (Task Manager -> python).")
        print("  3. Unplug & replug the board, wait 5 s, run again.")
    else:
        print(f"{port} is not present. Check the cable/upload, or pass -p COMx")
    print("\nSerial ports visible right now:\n" + list_ports_str())
    print("=" * 70)
    sys.exit(1)

# ============================== SELF-TEST ===================================
def selftest():
    """Offline validation of the DSP against a synthesised, coherently-sampled
    50 Hz signal with known harmonics — no board required.  Confirms the SINGLE
    /4 magnitude correction is exact (amplitudes come out right, not 4x/16x off)."""
    fs = fs_actual(DEF_RATE, DEF_OSR)
    vpc = DEF_VREF / 1024.0
    dsp = DSP(fs, DEF_OSR, vpc, N_HARM)
    print(f"self-test: fs={fs:.3f} Sa/s  bin={dsp.binhz:.4f} Hz  "
          f"N={N_FFT} ({N_FFT/(fs/LINE_HZ):.3f} cycles)")

    truth = {1: 1.000, 3: 0.150, 5: 0.080, 7: 0.030}    # harmonics 1..7 only
    dc = 1.60
    t = np.arange(N_FFT) / fs
    sig = np.full(N_FFT, dc)
    for h, a in truth.items():
        sig += a * np.cos(2 * np.pi * h * LINE_HZ * t + 0.3 * h)
    codes = np.round(sig / vpc * DEF_OSR)               # volts -> OSR-summed codes
    r = dsp.analyze(codes)

    ok = True
    if abs(r["f1"] - LINE_HZ) > 0.05:
        print(f"  FAIL f0: {r['f1']:.3f} != {LINE_HZ}"); ok = False
    else:
        print(f"  PASS f0 = {r['f1']:.4f} Hz")
    amp = {h: a for h, _, a, _ in r["harm"]}
    for h, a_true in truth.items():
        a_meas = amp.get(h, 0.0)
        err = abs(a_meas - a_true) / a_true * 100
        tag = "PASS" if err < 1.0 else "FAIL"
        if err >= 1.0:
            ok = False
        print(f"  {tag} h{h:<2} amp meas={a_meas:.5f} V true={a_true:.5f} V "
              f"err={err:.3f}%   %F0 meas={100*a_meas/r['a1']:.2f}")
    leak = max((a for h, _, a, _ in r["harm"] if h not in truth), default=0.0)
    print(f"  max spurious harmonic amp = {leak:.6f} V "
          f"({'PASS' if leak < 1e-3 else 'FAIL'} < 1e-3)")
    if leak >= 1e-3:
        ok = False
    thd_true = np.sqrt(sum(a**2 for h, a in truth.items() if h != 1)) / truth[1]
    print(f"  THD meas={r['thd']*100:.3f}%  true={thd_true*100:.3f}%  "
          f"({'PASS' if abs(r['thd']-thd_true) < 5e-4 else 'FAIL'})")
    print("SELF-TEST:", "PASS" if ok else "FAIL")
    return 0 if ok else 1

# ================================ MAIN ======================================
def main():
    ap = argparse.ArgumentParser(description="Live 50 Hz power-line harmonic "
                                 "monitor (block/trigger, coherent 1024-pt FFT)")
    ap.add_argument("-p", "--port", help="serial port (auto-detect if omitted)")
    ap.add_argument("--rate", type=int, default=DEF_RATE, help="output Sa/s (800..10000)")
    ap.add_argument("--osr", type=int, default=DEF_OSR, choices=[1, 2, 4, 8, 16],
                    help="oversampling ratio (rate*osr <= 38000)")
    ap.add_argument("--ref", choices=["ext", "avcc", "EXT", "AVCC"], default="avcc",
                    help="ADC reference. avcc = internal ~5 V rail (DEFAULT, no wiring). "
                         "ext = AREF pin (wire 3.3 V there first).")
    ap.add_argument("--vref", type=float, default=None,
                    help="reference volts for code->volt scaling (default 5.0 for "
                         "avcc, 3.3 for ext); ignored when a gain calibration exists")
    ap.add_argument("--calz", action="store_true", help="offset/baseline calibration first")
    ap.add_argument("--calg", type=float, metavar="MVRMS", help="gain calibration mVrms")
    ap.add_argument("--selftest", action="store_true",
                    help="run the offline DSP self-test and exit (no board)")
    args = ap.parse_args()

    if args.vref is None:
        args.vref = 5.0 if args.ref.lower() == "avcc" else 3.3

    if args.selftest:
        sys.exit(selftest())

    if args.rate * args.osr > 38000:
        sys.exit(f"config rejected: rate*osr = {args.rate*args.osr} > 38000 "
                 f"(the ADC ceiling). Try --rate 1600 --osr 4, or lower osr.")

    port = args.port or find_port()
    if not port:
        sys.exit("No Arduino serial port found. Pass one with -p (e.g. -p COM5).")

    ser = open_serial(port)
    try:
        fs, osr, vpc, scale_note, calibrated = setup_device(ser, args)
    except Exception:
        ser.close(); raise

    print(f"Port {port} | fs = {fs:.3f} Sa/s (Nyquist {fs/2:.0f} Hz) | osr = {osr} | "
          f"RAW+host-DC | block = {N_FFT} samples (2 KB) | "
          f"bin {fs/N_FFT:.4f} Hz, {N_FFT/(fs/LINE_HZ):.3f} cycles | "
          f"harmonics 1..{N_HARM} ({N_HARM*int(LINE_HZ)} Hz)")

    ser.timeout = 0.02
    reader = BlockReader(ser, fs, osr, vpc)
    reader.start()
    reader.start_acq()                    # arm the first block immediately
    dsp = DSP(fs, osr, vpc, N_HARM)
    dsp.scale_note = scale_note           # what volts/count assumption is shown
    dsp.calibrated = calibrated           # gain-cal present? (else show --calg hint)
    dsp.vfs = vpc * 1024.0                # nominal full-scale volts (0..vfs)
    try:
        _ani = run_gui(reader, dsp)
    finally:
        reader.stop(); reader.join(timeout=1.5)
        try:
            ser.write(b"STOP\n"); time.sleep(0.3); ser.reset_input_buffer()
        except Exception:
            pass
        ser.close()

if __name__ == "__main__":
    main()
