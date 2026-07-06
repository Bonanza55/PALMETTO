/* fsk_wfmd.c  --  broadcast (wideband) FM chunked receive & playback daemon
 * Async Capture Edition (v2).
 *
 * Derived from fsk_rxd.c (Hardened Edition).  This is NOT a digital-mode tool:
 * it tunes a broadcast FM station, demodulates WFM to 48 kHz mono audio
 * in-process, and writes fixed-length WAV chunks into a circular set of
 * NUM_CHUNKS slot files (default: a 2 x 120 s ping-pong):
 *
 *      chunk1.wav -> chunk2.wav -> chunk1.wav -> ...
 *
 * A player thread plays finished chunks in order with /usr/bin/afplay.
 *
 * v2 change (the "pixelated audio" fix): v1 used rtlsdr_read_sync() in a
 * loop.  Sync reads have no background buffering, so every gap between
 * consecutive reads (demod compute, and especially the 2.9 MB WAV write at
 * each chunk boundary) dropped samples -> phase discontinuity at the FM
 * discriminator -> a click every ~27 ms block = gritty "bitcrushed" audio.
 * v2 uses rtlsdr_read_async(): libusb keeps a transfer queue continuously
 * filled, the USB callback only memcpys into a 16 MB IQ ring (~7 s of
 * headroom at 1.2 MSps), and a separate demod thread drains it.  Disk
 * stalls no longer touch the RF path.  Same producer-consumer pattern as
 * the async disk subsystem in fsk_rxd.c, pointed the other direction.
 *
 * Threads:
 *   main   : device setup, runs rtlsdr_read_async() event loop, reinit ladder
 *   demod  : IQ ring -> WFM demod -> chunk WAVs -> play queue
 *   player : play queue -> afplay (one chunk at a time, in order)
 *
 * Pacing model:
 *   producer: exactly chunk_sec of audio per chunk_sec of wall time (SDR clock)
 *   consumer: chunk_sec of audio per chunk_sec + afplay spawn (~0.1 s)
 *   => lag grows ~0.1-0.2 s per chunk.  The ring absorbs it; when the queue
 *   fills, the OLDEST pending chunk is dropped (skip forward, stay near-live).
 *
 * Overwrite safety: pending queue capped at NUM_CHUNKS-1 (pending slot
 * distance is never 0 by construction), and the demod thread WAITS briefly
 * (slot_wait_free, stall absorbed by the IQ ring) if the target slot is the
 * one afplay is reading -- essential in a 2-slot ping-pong, where afplay's
 * accumulated spawn drift periodically overlaps the wrap by a fraction of a
 * second.  Only a wedged player (> SLOT_WAIT_SEC) costs a dropped chunk.
 *
 * All output goes to stderr (logmsg, usage, and child stdout is duped).
 *
 * DSP chain (streaming state across blocks):
 *   u8 IQ -> float -> FIR decimate to 240 kHz IF -> polar discriminator
 *   -> 75 us de-emphasis -> FIR decimate to 48 kHz (14 kHz LPF kills the
 *   19 kHz stereo pilot) -> DC block -> s16 mono.
 *
 * Build (Mac mini / Apple Silicon, Homebrew librtlsdr):
 *   clang -arch arm64 -O3 -std=c11 -Wall -Wextra \
 *         -I/opt/homebrew/include -L/opt/homebrew/lib \
 *         -o fsk_wfmd fsk_wfmd.c -lrtlsdr -lm -lpthread
 *
 * Run:
 *   ./fsk_wfmd -f 89300000            # 89.3 MHz, 30 s chunks, play as you go
 *   ./fsk_wfmd -f 96900000 -c 20 -N   # 20 s chunks, record only (no afplay)
 */

#define _POSIX_C_SOURCE 200809L
#define _DEFAULT_SOURCE

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <time.h>
#include <errno.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include <getopt.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <rtl-sdr.h>

/* ---- compile-time defaults (override on the command line) ---- */
#define DEFAULT_FREQ_HZ        89300000u    /* broadcast band, not 2 m       */
#define DEFAULT_SAMP_RATE      1200000u     /* must be a multiple of IF_RATE */
#define DEFAULT_GAIN_TENTHS    402
#define DEFAULT_PPM            0
#define DEFAULT_DEV_INDEX      0
#define DEFAULT_DEEMPH_US      75.0         /* US broadcast de-emphasis      */
#define DEFAULT_CHUNK_SEC      120
#define DEFAULT_OUTDIR         "."
#define DEFAULT_AUDIO_GAIN     1.0

#define IF_RATE                240000u      /* post-stage-1 rate             */
#define AUDIO_RATE             48000u       /* WAV output rate               */
#define NUM_CHUNKS             2            /* chunk1.wav, chunk2.wav        */
#define MAX_CHUNK_SEC          180
#define MIN_CHUNK_SEC          2

#define ASYNC_BUF_NUM          12           /* libusb transfer queue depth   */
#define ASYNC_BUF_LEN          (1u << 18)   /* 256 KiB per transfer          */
#define FLUSH_BLOCKS           4
#define PLL_TOL_HZ             1000
#define PLL_RETRIES            5

#define REINIT_ATTEMPTS        5
#define REINIT_BACKOFF_SEC     3

#define SQ_DC_BIAS             127.4f

/* audio scaling: +/-75 kHz deviation at 240 kHz IF -> ~1.96 rad disc peak */
#define BASE_AUDIO_SCALE       15000.0f

/* === HARDENED STATIC STORAGE CAPACITY CEILINGS === */
#define MAX_PATH_LEN           512
#define MAX_TAPS               192
#define DEMOD_SLICE_BYTES      (1u << 16)              /* 64 KiB per pass    */
#define MAX_BLOCK_SAMPS        (DEMOD_SLICE_BYTES / 2) /* 32768              */
#define MAX_IF_SAMPS           (MAX_BLOCK_SAMPS / 2 + 8)
#define MAX_AUDIO_SAMPS        (MAX_IF_SAMPS / 5 + 8)
#define IQ_RING_BYTES          (32u * 1024u * 1024u)   /* ~14 s @ 1.2 MSps   */
#define SLOT_WAIT_SEC          6            /* max wait for player slot      */

_Static_assert(IF_RATE % AUDIO_RATE == 0, "IF must decimate integrally to audio");
_Static_assert(NUM_CHUNKS >= 2, "ping-pong needs at least 2 slots");
_Static_assert(SLOT_WAIT_SEC * 2u * 2400000u < IQ_RING_BYTES,
               "slot wait must be absorbable by the IQ ring at max rate");
_Static_assert(ASYNC_BUF_LEN % 512 == 0, "libusb bulk transfers need 512-B multiples");
_Static_assert(IQ_RING_BYTES > 4u * ASYNC_BUF_LEN, "ring must dwarf one transfer");

static volatile sig_atomic_t g_stop = 0;
static void on_signal(int s) { (void)s; g_stop = 1; }

static void logmsg(const char *fmt, ...) {
    char ts[32], line[512];
    time_t now = time(NULL);
    struct tm tmv;
    gmtime_r(&now, &tmv);
    strftime(ts, sizeof ts, "%Y-%m-%dT%H:%M:%SZ", &tmv);

    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof line, fmt, ap);
    va_end(ap);

    fprintf(stderr, "%s %s\n", ts, line);
    fflush(stderr);
}

/* =============================================================================
 * IQ RING (USB producer -> demod consumer)
 * The USB callback must never block on disk or DSP; it only memcpys here.
 * ===========================================================================*/
static uint8_t          s_iq_ring[IQ_RING_BYTES];
static size_t           iq_head = 0, iq_tail = 0;
static unsigned long    iq_overruns = 0;      /* bytes dropped (ring full)   */
static pthread_mutex_t  iq_mx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t   iq_cv = PTHREAD_COND_INITIALIZER;
static int              demod_quit = 0;

static void usb_callback(unsigned char *buf, uint32_t len, void *ctx) {
    rtlsdr_dev_t *dev = (rtlsdr_dev_t *)ctx;
    if (g_stop) { rtlsdr_cancel_async(dev); return; }

    pthread_mutex_lock(&iq_mx);
    const size_t space = (iq_tail > iq_head)
                       ? (iq_tail - iq_head - 1)
                       : (IQ_RING_BYTES - iq_head + iq_tail - 1);
    if (space < len) {
        iq_overruns += len;                   /* demod fell > ring behind    */
    } else {
        size_t first = IQ_RING_BYTES - iq_head;
        if (first > len) first = len;
        memcpy(s_iq_ring + iq_head, buf, first);
        if (len > first) memcpy(s_iq_ring, buf + first, len - first);
        iq_head = (iq_head + len) % IQ_RING_BYTES;
        pthread_cond_signal(&iq_cv);
    }
    pthread_mutex_unlock(&iq_mx);
}

/* pull up to cap bytes (even count) into dst; blocks until data or quit */
static size_t iq_ring_pull(uint8_t *dst, size_t cap) {
    pthread_mutex_lock(&iq_mx);
    while (iq_head == iq_tail && !demod_quit)
        pthread_cond_wait(&iq_cv, &iq_mx);
    if (demod_quit && iq_head == iq_tail) {
        pthread_mutex_unlock(&iq_mx);
        return 0;
    }
    size_t avail = (iq_head >= iq_tail)
                 ? (iq_head - iq_tail)
                 : (IQ_RING_BYTES - iq_tail + iq_head);
    size_t take = (avail < cap) ? avail : cap;
    take &= ~(size_t)1;                       /* keep I/Q pairs intact       */
    if (take == 0) { pthread_mutex_unlock(&iq_mx); return 0; }

    size_t first = IQ_RING_BYTES - iq_tail;
    if (first > take) first = take;
    memcpy(dst, s_iq_ring + iq_tail, first);
    if (take > first) memcpy(dst + first, s_iq_ring, take - first);
    iq_tail = (iq_tail + take) % IQ_RING_BYTES;
    pthread_mutex_unlock(&iq_mx);
    return take;
}

/* =============================================================================
 * STREAMING FIR DECIMATOR (windowed-sinc, Hamming), state carried across blocks
 * ===========================================================================*/
typedef struct {
    int   ntaps, dec, hlen, phase;
    float taps[MAX_TAPS];
    float hist[MAX_TAPS];
    float buf[MAX_TAPS + MAX_BLOCK_SAMPS];
} decim_t;

static void decim_design(decim_t *d, int dec, int ntaps, double fc_norm) {
    if (ntaps > MAX_TAPS) ntaps = MAX_TAPS;
    if (!(ntaps & 1)) ntaps--;                    /* keep it odd / symmetric */
    d->ntaps = ntaps;
    d->dec   = dec;
    d->hlen  = ntaps - 1;
    d->phase = 0;

    const int M = ntaps - 1;
    double sum = 0.0;
    for (int n = 0; n < ntaps; n++) {
        double x = n - M / 2.0;
        double s = (x == 0.0) ? 2.0 * fc_norm
                              : sin(2.0 * M_PI * fc_norm * x) / (M_PI * x);
        double w = 0.54 - 0.46 * cos(2.0 * M_PI * n / M);
        d->taps[n] = (float)(s * w);
        sum += s * w;
    }
    for (int n = 0; n < ntaps; n++) d->taps[n] /= (float)sum;  /* unity DC */
    memset(d->hist, 0, sizeof d->hist);
}

/* feed n input samples, emit decimated output; returns output count */
static int decim_run(decim_t *d, const float *in, int n, float *out) {
    memcpy(d->buf, d->hist, (size_t)d->hlen * sizeof(float));
    memcpy(d->buf + d->hlen, in, (size_t)n * sizeof(float));

    const int L = d->hlen + n;
    int p = d->phase, m = 0;
    while (p + d->ntaps <= L) {
        const float *b = d->buf + p;
        float acc = 0.0f;
        for (int k = 0; k < d->ntaps; k++) acc += b[k] * d->taps[k];
        out[m++] = acc;
        p += d->dec;
    }
    const int keep_from = L - d->hlen;
    memcpy(d->hist, d->buf + keep_from, (size_t)d->hlen * sizeof(float));
    d->phase = p - keep_from;                    /* provably >= 0            */
    return m;
}

/* =============================================================================
 * WFM DEMODULATOR (streaming)
 * ===========================================================================*/
static decim_t s_dec1_i, s_dec1_q;               /* rate    -> IF_RATE       */
static decim_t s_dec2;                           /* IF_RATE -> AUDIO_RATE    */

static float  s_prev_i = 0.0f, s_prev_q = 0.0f;  /* discriminator memory     */
static float  s_deemph_y = 0.0f, s_deemph_a = 0.0f;
static float  s_dcb_x1 = 0.0f, s_dcb_y1 = 0.0f;  /* audio DC blocker         */
static float  s_audio_scale = BASE_AUDIO_SCALE;

/* per-slice scratch */
static float  s_fi[MAX_BLOCK_SAMPS],  s_fq[MAX_BLOCK_SAMPS];
static float  s_ifi[MAX_IF_SAMPS],    s_ifq[MAX_IF_SAMPS];
static float  s_disc[MAX_IF_SAMPS];
static float  s_aud[MAX_AUDIO_SAMPS];

static void wfm_init(uint32_t rate, double deemph_us, double audio_gain) {
    const int dec1 = (int)(rate / IF_RATE);
    const int dec2 = (int)(IF_RATE / AUDIO_RATE);              /* == 5       */

    /* stage 1: pass the ~200 kHz broadcast channel, kill everything else   */
    decim_design(&s_dec1_i, dec1, 16 * dec1 + 1, 100000.0 / (double)rate);
    decim_design(&s_dec1_q, dec1, 16 * dec1 + 1, 100000.0 / (double)rate);

    /* stage 2: mono audio LPF; 14 kHz cutoff, 161 taps -> 19 kHz pilot is
     * ~5 kHz into the stopband (Hamming: > 50 dB down)                     */
    decim_design(&s_dec2, dec2, 161, 14000.0 / (double)IF_RATE);

    s_deemph_a    = 1.0f - (float)exp(-1.0 / (deemph_us * 1e-6 * (double)IF_RATE));
    s_audio_scale = (float)(BASE_AUDIO_SCALE * audio_gain);
}

/* raw u8 IQ slice -> int16 mono audio; returns audio sample count */
static int wfm_block(const uint8_t *iq, int nbytes, int16_t *out) {
    const int n = nbytes / 2;
    if (n <= 0 || n > (int)MAX_BLOCK_SAMPS) return 0;

    for (int k = 0; k < n; k++) {
        s_fi[k] = ((float)iq[2*k]     - SQ_DC_BIAS) * (1.0f / 128.0f);
        s_fq[k] = ((float)iq[2*k + 1] - SQ_DC_BIAS) * (1.0f / 128.0f);
    }

    const int n1 = decim_run(&s_dec1_i, s_fi, n, s_ifi);
    (void)      decim_run(&s_dec1_q, s_fq, n, s_ifq);  /* same count as n1 */

    /* polar discriminator: arg( x[n] * conj(x[n-1]) ) */
    float pi_ = s_prev_i, pq_ = s_prev_q;
    for (int k = 0; k < n1; k++) {
        const float ci = s_ifi[k], cq = s_ifq[k];
        const float re = ci * pi_ + cq * pq_;
        const float im = cq * pi_ - ci * pq_;
        s_disc[k] = atan2f(im, re);
        pi_ = ci; pq_ = cq;
    }
    s_prev_i = pi_; s_prev_q = pq_;

    /* 75 us de-emphasis (one-pole IIR at IF rate) */
    float y = s_deemph_y;
    const float a = s_deemph_a;
    for (int k = 0; k < n1; k++) {
        y += a * (s_disc[k] - y);
        s_disc[k] = y;
    }
    s_deemph_y = y;

    const int n2 = decim_run(&s_dec2, s_disc, n1, s_aud);

    /* DC block (~4 Hz HPF) + scale + clamp */
    float x1 = s_dcb_x1, y1 = s_dcb_y1;
    for (int k = 0; k < n2; k++) {
        const float x  = s_aud[k];
        const float hp = x - x1 + 0.9995f * y1;
        x1 = x; y1 = hp;
        float v = hp * s_audio_scale;
        if (v >  32767.0f) v =  32767.0f;
        if (v < -32768.0f) v = -32768.0f;
        out[k] = (int16_t)lrintf(v);
    }
    s_dcb_x1 = x1; s_dcb_y1 = y1;
    return n2;
}

/* =============================================================================
 * WAV WRITER (44-byte PCM header, mono s16)
 * ===========================================================================*/
static int write_wav(const char *path, const int16_t *pcm, uint32_t nsamp,
                     uint32_t rate) {
    FILE *f = fopen(path, "wb");
    if (!f) return -1;

    const uint32_t data_bytes = nsamp * 2u;
    const uint32_t riff_bytes = 36u + data_bytes;
    const uint16_t ch = 1, bits = 16;
    const uint32_t byte_rate = rate * 2u;
    const uint16_t block_align = 2;
    uint8_t h[44];

    memcpy(h, "RIFF", 4);
    memcpy(h + 4, &riff_bytes, 4);
    memcpy(h + 8, "WAVEfmt ", 8);
    const uint32_t fmt_len = 16; const uint16_t pcm_tag = 1;
    memcpy(h + 16, &fmt_len, 4);
    memcpy(h + 20, &pcm_tag, 2);
    memcpy(h + 22, &ch, 2);
    memcpy(h + 24, &rate, 4);
    memcpy(h + 28, &byte_rate, 4);
    memcpy(h + 32, &block_align, 2);
    memcpy(h + 34, &bits, 2);
    memcpy(h + 36, "data", 4);
    memcpy(h + 40, &data_bytes, 4);

    int ok = (fwrite(h, 1, 44, f) == 44) &&
             (fwrite(pcm, 2, nsamp, f) == nsamp);
    if (fclose(f) != 0) ok = 0;
    return ok ? 0 : -1;
}

/* =============================================================================
 * PLAY QUEUE + PLAYER THREAD (afplay)
 *
 * Capacity NUM_CHUNKS-1: pending chunks are at slot distance 1..NUM_CHUNKS-1
 * from the capture side's write slot, never 0, so pending files can't be
 * overwritten.  The PLAYING slot is protected by slot_wait_free(): the demod
 * thread waits (IQ ring absorbs the stall) instead of scribbling on the file
 * afplay has open.  With NUM_CHUNKS=2 this is a strict ping-pong.
 * ===========================================================================*/
#define PQ_CAP (NUM_CHUNKS - 1)

static pthread_mutex_t pq_mx      = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  pq_cv      = PTHREAD_COND_INITIALIZER;
static pthread_cond_t  pq_play_cv = PTHREAD_COND_INITIALIZER;  /* slot freed */
static long            pq_q[PQ_CAP];
static int             pq_h = 0, pq_t = 0, pq_n = 0;
static int             pq_quit = 0;
static volatile pid_t  g_play_pid = 0;
static long            g_playing_seq = -1;   /* guarded by pq_mx            */
static char            g_outdir[MAX_PATH_LEN] = DEFAULT_OUTDIR;

static void pq_push(long seq) {
    pthread_mutex_lock(&pq_mx);
    if (pq_n == PQ_CAP) {                        /* player lagging: skip fwd */
        logmsg("WARN player lagging; dropping oldest pending chunk seq=%ld",
               pq_q[pq_t]);
        pq_t = (pq_t + 1) % PQ_CAP;
        pq_n--;
    }
    pq_q[pq_h] = seq;
    pq_h = (pq_h + 1) % PQ_CAP;
    pq_n++;
    pthread_cond_signal(&pq_cv);
    pthread_mutex_unlock(&pq_mx);
}

static void *player_thread(void *arg) {
    (void)arg;
    for (;;) {
        pthread_mutex_lock(&pq_mx);
        while (pq_n == 0 && !pq_quit)
            pthread_cond_wait(&pq_cv, &pq_mx);
        if (pq_quit) { pthread_mutex_unlock(&pq_mx); break; }
        const long seq = pq_q[pq_t];
        pq_t = (pq_t + 1) % PQ_CAP;
        pq_n--;
        pthread_mutex_unlock(&pq_mx);

        char path[MAX_PATH_LEN * 2];
        snprintf(path, sizeof path, "%s/chunk%d.wav",
                 g_outdir, (int)(seq % NUM_CHUNKS) + 1);

        pthread_mutex_lock(&pq_mx);
        g_playing_seq = seq;
        pthread_mutex_unlock(&pq_mx);

        const pid_t pid = fork();
        if (pid < 0) {
            pthread_mutex_lock(&pq_mx);
            g_playing_seq = -1;
            pthread_cond_broadcast(&pq_play_cv);
            pthread_mutex_unlock(&pq_mx);
            logmsg("ERR fork(afplay): %s", strerror(errno));
            continue;
        }
        if (pid == 0) {
            /* everything a child prints lands on stderr too */
            dup2(STDERR_FILENO, STDOUT_FILENO);
            execlp("afplay", "afplay", path, (char *)NULL);
            fprintf(stderr, "exec afplay failed: %s\n", strerror(errno));
            _exit(127);
        }
        g_play_pid = pid;
        int status = 0;
        while (waitpid(pid, &status, 0) < 0) {
            if (errno != EINTR) { logmsg("ERR waitpid(afplay): %s", strerror(errno)); break; }
        }
        g_play_pid = 0;
        pthread_mutex_lock(&pq_mx);
        g_playing_seq = -1;
        pthread_cond_broadcast(&pq_play_cv);
        pthread_mutex_unlock(&pq_mx);
        if (WIFEXITED(status) && WEXITSTATUS(status) != 0)
            logmsg("WARN afplay rc=%d for %s", WEXITSTATUS(status), path);
        else
            logmsg("PLAY  done seq=%ld (%s)", seq, path);
    }
    return NULL;
}

static void player_shutdown(pthread_t tid) {
    pthread_mutex_lock(&pq_mx);
    pq_quit = 1;
    pthread_cond_broadcast(&pq_cv);
    pthread_cond_broadcast(&pq_play_cv);
    pthread_mutex_unlock(&pq_mx);
    if (g_play_pid > 0) kill(g_play_pid, SIGTERM);
    pthread_join(tid, NULL);
}

/* =============================================================================
 * RTL-SDR SETUP (carried over from fsk_rxd.c)
 * ===========================================================================*/
static uint8_t s_flush_buf[DEMOD_SLICE_BYTES];

static int snap_gain(rtlsdr_dev_t *dev, int want_tenths) {
    int n = rtlsdr_get_tuner_gains(dev, NULL);
    if (n <= 0) return want_tenths;

    int *gains = malloc((size_t)n * sizeof(int));
    if (!gains) return want_tenths;

    rtlsdr_get_tuner_gains(dev, gains);
    int best = gains[0], bestd = abs(gains[0] - want_tenths);
    for (int i = 1; i < n; i++) {
        int d = abs(gains[i] - want_tenths);
        if (d < bestd) { bestd = d; best = gains[i]; }
    }
    free(gains);
    return best;
}

static int configure_dongle(rtlsdr_dev_t *dev, uint32_t freq, uint32_t rate,
                            int gain_tenths, int ppm) {
    if (rtlsdr_set_sample_rate(dev, rate) < 0) {
        logmsg("ERR set_sample_rate(%u) failed", rate); return -1;
    }
    if (rtlsdr_set_tuner_gain_mode(dev, 1) < 0) {
        logmsg("ERR set_tuner_gain_mode(manual) failed"); return -1;
    }
    int g = snap_gain(dev, gain_tenths);
    if (rtlsdr_set_tuner_gain(dev, g) < 0) {
        logmsg("ERR set_tuner_gain(%d) failed", g); return -1;
    }
    rtlsdr_set_agc_mode(dev, 0);
    if (ppm) rtlsdr_set_freq_correction(dev, ppm);
    if (gain_tenths != g)
        logmsg("gain snapped %.1f -> %.1f dB", gain_tenths / 10.0, g / 10.0);

    int locked = 0;
    for (int i = 0; i < PLL_RETRIES; i++) {
        rtlsdr_set_center_freq(dev, freq);
        usleep(50000);
        uint32_t actual = rtlsdr_get_center_freq(dev);
        long err = (long)actual - (long)freq;
        if (labs(err) <= PLL_TOL_HZ) { locked = 1; break; }
        logmsg("PLL retry %d: want %u got %u (err %ld Hz)", i + 1, freq, actual, err);
    }
    if (!locked)
        logmsg("WARN PLL not confirmed within %d Hz; continuing", PLL_TOL_HZ);

    rtlsdr_reset_buffer(dev);
    for (int i = 0; i < FLUSH_BLOCKS; i++) {
        int nr = 0;
        rtlsdr_read_sync(dev, s_flush_buf, DEMOD_SLICE_BYTES, &nr);
    }
    return 0;
}

static int reinit_device(rtlsdr_dev_t **pdev, int dev_index, uint32_t freq,
                         uint32_t rate, int gain_tenths, int ppm) {
    if (*pdev) { rtlsdr_close(*pdev); *pdev = NULL; }

    for (int a = 1; a <= REINIT_ATTEMPTS && !g_stop; a++) {
        logmsg("device re-init attempt %d/%d (backoff %d s)",
               a, REINIT_ATTEMPTS, REINIT_BACKOFF_SEC);
        sleep(REINIT_BACKOFF_SEC);
        if (rtlsdr_open(pdev, (uint32_t)dev_index) < 0) {
            *pdev = NULL;
            continue;
        }
        if (configure_dongle(*pdev, freq, rate, gain_tenths, ppm) == 0) {
            logmsg("device re-init OK");
            return 0;
        }
        rtlsdr_close(*pdev); *pdev = NULL;
    }
    return -1;
}

/* =============================================================================
 * DEMOD THREAD: IQ ring -> WFM -> chunk WAVs -> play queue
 * ===========================================================================*/

/* Wait until the target slot is not the one afplay is reading, up to
 * SLOT_WAIT_SEC.  Normal waits are the spawn-drift remainder (well under a
 * second); the IQ ring carries the demod stall.  Returns 0 = free, -1 = the
 * player is wedged and the caller should drop the chunk. */
static int slot_wait_free(long seq) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += SLOT_WAIT_SEC;

    int waited = 0;
    pthread_mutex_lock(&pq_mx);
    while (!g_stop && g_playing_seq >= 0 &&
           (g_playing_seq % NUM_CHUNKS) == (seq % NUM_CHUNKS)) {
        if (!waited) {
            logmsg("slot %d busy (player at seq=%ld); waiting up to %d s",
                   (int)(seq % NUM_CHUNKS) + 1, g_playing_seq, SLOT_WAIT_SEC);
            waited = 1;
        }
        if (pthread_cond_timedwait(&pq_play_cv, &pq_mx, &ts) == ETIMEDOUT) {
            pthread_mutex_unlock(&pq_mx);
            return -1;
        }
    }
    pthread_mutex_unlock(&pq_mx);
    return 0;
}

static int16_t s_chunk[MAX_CHUNK_SEC * AUDIO_RATE];   /* one chunk, ~17 MB   */
static int16_t s_ablk[MAX_AUDIO_SAMPS];               /* per-slice audio     */
static uint8_t s_slice[DEMOD_SLICE_BYTES];            /* IQ pulled from ring */

/* parameters handed from main to the demod thread */
static long g_chunk_samps = 0;
static int  g_chunk_sec = DEFAULT_CHUNK_SEC;
static int  g_noplay = 0;

static void *demod_thread(void *arg) {
    (void)arg;
    long fill = 0, seq = 0;
    int  peak = 0;
    double rms_acc = 0.0;
    unsigned long overruns_seen = 0;

    for (;;) {
        size_t got = iq_ring_pull(s_slice, DEMOD_SLICE_BYTES);
        if (got == 0) break;                      /* quit and drained        */

        int na = wfm_block(s_slice, (int)got, s_ablk);
        int src = 0;
        while (na > 0) {
            long room = g_chunk_samps - fill;
            int take = (na < room) ? na : (int)room;

            memcpy(s_chunk + fill, s_ablk + src, (size_t)take * sizeof(int16_t));
            for (int k = 0; k < take; k++) {
                int v = s_ablk[src + k];
                int av = v < 0 ? -v : v;
                if (av > peak) peak = av;
                rms_acc += (double)v * (double)v;
            }
            fill += take; src += take; na -= take;

            if (fill == g_chunk_samps) {
                const int slot = (int)(seq % NUM_CHUNKS) + 1;
                char path[MAX_PATH_LEN * 2];
                snprintf(path, sizeof path, "%s/chunk%d.wav", g_outdir, slot);

                if (!g_noplay && slot_wait_free(seq) != 0) {
                    /* player wedged for > SLOT_WAIT_SEC; never scribble on
                     * the file afplay has open -- drop the new chunk       */
                    logmsg("WARN slot %d still busy after %d s (player "
                           "wedged); dropping chunk seq=%ld",
                           slot, SLOT_WAIT_SEC, seq);
                } else if (write_wav(path, s_chunk, (uint32_t)g_chunk_samps,
                                     AUDIO_RATE) != 0) {
                    logmsg("ERR writing %s: %s -- chunk lost",
                           path, strerror(errno));
                } else {
                    double rms = sqrt(rms_acc / (double)g_chunk_samps);
                    pthread_mutex_lock(&iq_mx);
                    unsigned long ovr = iq_overruns;
                    size_t depth = (iq_head >= iq_tail)
                                 ? (iq_head - iq_tail)
                                 : (IQ_RING_BYTES - iq_tail + iq_head);
                    pthread_mutex_unlock(&iq_mx);

                    logmsg("CHUNK seq=%ld slot=%d  %ds  peak=%.1f dBFS  "
                           "rms=%.1f dBFS  ring=%.0f%%  -> %s",
                           seq, slot, g_chunk_sec,
                           20.0 * log10((peak + 1) / 32768.0),
                           20.0 * log10((rms + 1e-6) / 32768.0),
                           100.0 * (double)depth / IQ_RING_BYTES, path);
                    if (ovr != overruns_seen) {
                        logmsg("WARN IQ ring overran: %lu bytes dropped since "
                               "last chunk", ovr - overruns_seen);
                        overruns_seen = ovr;
                    }
                    if (peak < 300)
                        logmsg("WARN near-silent chunk: wrong frequency, "
                               "dead air, or gain too low");
                    if (!g_noplay) pq_push(seq);
                }
                seq++;
                fill = 0; peak = 0; rms_acc = 0.0;
            }
        }
    }
    logmsg("demod thread done (%ld chunks captured)", seq);
    return NULL;
}

/* =============================================================================
 * MAIN: device lifecycle + async USB event loop
 * ===========================================================================*/
static void usage(const char *p) {
    fprintf(stderr,
      "usage: %s [-f freq_hz] [-p ppm] [-s samp_rate] [-g gain_tenths] [-d dev_index]\n"
      "          [-c chunk_sec] [-e deemph_us] [-a audio_gain] [-o outdir] [-N] [-v]\n"
      "  -c  chunk length in seconds (%d..%d, default %d)\n"
      "  -a  audio gain multiplier (default %.1f)\n"
      "  -N  record chunks only; do not launch afplay\n"
      "  defaults: -f %u -p %d -s %u -g %d -d %d -c %d -e %.0f -o %s\n"
      "  note: samp_rate must be a multiple of %u\n",
      p, MIN_CHUNK_SEC, MAX_CHUNK_SEC, DEFAULT_CHUNK_SEC, DEFAULT_AUDIO_GAIN,
      DEFAULT_FREQ_HZ, DEFAULT_PPM, DEFAULT_SAMP_RATE, DEFAULT_GAIN_TENTHS,
      DEFAULT_DEV_INDEX, DEFAULT_CHUNK_SEC, DEFAULT_DEEMPH_US, DEFAULT_OUTDIR,
      IF_RATE);
}

int main(int argc, char **argv) {
    uint32_t freq = DEFAULT_FREQ_HZ, rate = DEFAULT_SAMP_RATE;
    int gain = DEFAULT_GAIN_TENTHS, ppm = DEFAULT_PPM, dev_index = DEFAULT_DEV_INDEX;
    int chunk_sec = DEFAULT_CHUNK_SEC, noplay = 0, verbose = 0;
    double deemph_us = DEFAULT_DEEMPH_US, audio_gain = DEFAULT_AUDIO_GAIN;
    const char *outdir = DEFAULT_OUTDIR;

    int c;
    while ((c = getopt(argc, argv, "f:p:s:g:d:c:e:a:o:Nvh")) != -1) {
        switch (c) {
            case 'f': freq = (uint32_t)strtoul(optarg, NULL, 10); break;
            case 'p': ppm  = atoi(optarg); break;
            case 's': rate = (uint32_t)strtoul(optarg, NULL, 10); break;
            case 'g': gain = atoi(optarg); break;
            case 'd': dev_index = atoi(optarg); break;
            case 'c': chunk_sec = atoi(optarg); break;
            case 'e': deemph_us = atof(optarg); break;
            case 'a': audio_gain = atof(optarg); break;
            case 'o': outdir = optarg; break;
            case 'N': noplay = 1; break;
            case 'v': verbose = 1; break;
            case 'h': default: usage(argv[0]); return (c == 'h') ? 0 : 2;
        }
    }
    (void)verbose;

    if (strlen(outdir) >= MAX_PATH_LEN) {
        fprintf(stderr, "Error: outdir path exceeds %d bytes.\n", MAX_PATH_LEN);
        return 1;
    }
    if (rate % IF_RATE != 0 || rate / IF_RATE < 2) {
        fprintf(stderr, "Error: samp_rate %u must be a multiple of %u (e.g. 1200000, 2400000).\n",
                rate, IF_RATE);
        return 1;
    }
    if (chunk_sec < MIN_CHUNK_SEC || chunk_sec > MAX_CHUNK_SEC) {
        fprintf(stderr, "Error: chunk_sec must be %d..%d.\n", MIN_CHUNK_SEC, MAX_CHUNK_SEC);
        return 1;
    }
    if (audio_gain <= 0.0 || audio_gain > 16.0) {
        fprintf(stderr, "Error: audio_gain must be in (0, 16].\n");
        return 1;
    }
    snprintf(g_outdir, sizeof g_outdir, "%s", outdir);
    g_chunk_sec   = chunk_sec;
    g_chunk_samps = (long)chunk_sec * (long)AUDIO_RATE;
    g_noplay      = noplay;

    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);

    wfm_init(rate, deemph_us, audio_gain);

    pthread_t player_tid, demod_tid;
    if (!noplay) {
        if (pthread_create(&player_tid, NULL, player_thread, NULL) != 0) {
            fprintf(stderr, "Error: cannot start player thread.\n");
            return 1;
        }
    }
    if (pthread_create(&demod_tid, NULL, demod_thread, NULL) != 0) {
        fprintf(stderr, "Error: cannot start demod thread.\n");
        return 1;
    }

    rtlsdr_dev_t *dev = NULL;
    if (rtlsdr_open(&dev, (uint32_t)dev_index) < 0) {
        logmsg("ERR cannot open RTL-SDR device %d", dev_index);
        return 1;
    }
    if (configure_dongle(dev, freq, rate, gain, ppm) < 0) {
        rtlsdr_close(dev); return 1;
    }

    logmsg("fsk_wfmd up (async): f=%.4f MHz  s=%u sps  g=%.1f dB  ppm=%d  dev=%d",
           freq / 1e6, rate, snap_gain(dev, gain) / 10.0, ppm, dev_index);
    logmsg("chunks: %d s x %d slots (%s/chunk1..%d.wav)  play=%s  "
           "usb queue=%dx%u KiB  iq ring=%.1f s",
           chunk_sec, NUM_CHUNKS, g_outdir, NUM_CHUNKS,
           noplay ? "OFF" : "afplay",
           ASYNC_BUF_NUM, ASYNC_BUF_LEN / 1024u,
           (double)IQ_RING_BYTES / 2.0 / (double)rate);

    int fatal = 0;
    while (!g_stop) {
        rtlsdr_reset_buffer(dev);
        int r = rtlsdr_read_async(dev, usb_callback, dev,
                                  ASYNC_BUF_NUM, ASYNC_BUF_LEN);
        if (g_stop) break;

        logmsg("WARN read_async returned r=%d; device fault", r);
        logmsg("NOTE audio discontinuity in current chunk (device re-init)");
        if (reinit_device(&dev, dev_index, freq, rate, gain, ppm) < 0) {
            logmsg("FATAL device unrecoverable after %d attempts; exiting "
                   "for supervisor restart", REINIT_ATTEMPTS);
            fatal = 1;
            break;
        }
    }

    /* shutdown: retire demod thread, then the player */
    pthread_mutex_lock(&iq_mx);
    demod_quit = 1;
    pthread_cond_broadcast(&iq_cv);
    pthread_mutex_unlock(&iq_mx);
    pthread_mutex_lock(&pq_mx);
    pthread_cond_broadcast(&pq_play_cv);   /* wake a demod stuck in slot wait */
    pthread_mutex_unlock(&pq_mx);
    pthread_join(demod_tid, NULL);

    if (!noplay) player_shutdown(player_tid);
    if (dev) rtlsdr_close(dev);

    logmsg("fsk_wfmd shutting down %s", fatal ? "(FATAL)" : "cleanly");
    return fatal ? 1 : 0;
}
