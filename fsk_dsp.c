#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#define DEFAULT_SDR_RATE 1200000   // 1.2 MSps input (matches fsk_rx default). Override
                                   // with -r if you changed fsk_rx -s. Must be an
                                   // integer multiple of INTER_RATE.
#define INTER_RATE      240000    // intermediate rate after stage 1
#define AUDIO_RATE      48000     // output rate after stage 2 (decim 5)
#define DECIM2          (INTER_RATE / AUDIO_RATE)  // 5
// DECIM1 is computed at runtime as sdr_rate / INTER_RATE:
//   1.2 MSps -> 5,  2.4 MSps -> 10.

// Channel filter (on the IQ, stage 1): pass the full NBFM channel, reject the rest
// of the captured spectrum before the discriminator. 7800 Hz one-sided: an
// uncorrected/temperature-drifting R820T can sit several kHz off center at 147.55
// MHz, and the occupied NBFM bandwidth already nearly fills a 6 kHz half-channel,
// so the margin keeps the upper-tone sideband inside the passband. 301 taps gives
// ample transition bandwidth at either supported input rate.
#define CH_CUTOFF       7800.0    // Hz, one-sided
#define CH_TAPS         301

// Audio filter (post-discriminator, stage 2): anti-alias for the 48 kHz output
// and pass both tones (1 kHz / 3 kHz) flat.
#define AUDIO_CUTOFF    2600.0     // Hz
#define AUDIO_TAPS      121

// De-emphasis time constant (microseconds). 75 us (US FM) undoes the radio's
// transmit pre-emphasis, which otherwise leaves the 3 kHz space tone ~4 dB hotter
// than the 1 kHz mark tone and eats all the margin on binary-1 symbols.
#define DEFAULT_DEEMPH_US   75.0

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Ascending comparator for percentile normalization.
static int cmp_float(const void *a, const void *b) {
    float fa = *(const float *)a, fb = *(const float *)b;
    return (fa > fb) - (fa < fb);
}

struct WavHeader {
    char riff_id[4]; uint32_t chunk_size; char wave_id[4];
    char fmt_id[4];  uint32_t fmt_size;   uint16_t audio_format;
    uint16_t num_channels; uint32_t sample_rate; uint32_t byte_rate;
    uint16_t block_align;  uint16_t bits_per_sample;
    char data_id[4]; uint32_t data_size;
};

// ---------------------------------------------------------------------------
// Decimating FIR with persistent ring-buffer state (so it streams across
// arbitrarily-sized input chunks). Handles real or complex input.
// ---------------------------------------------------------------------------
typedef struct {
    double *taps;
    int     ntaps;
    int     decim;
    int     is_complex;
    double *ri, *rq;   // ring buffers
    int     widx;      // next write position
    int     phase;     // input counter toward next decimated output
} FIRDecim;

static void design_lpf(double *taps, int N, double fs, double fc) {
    int M = N - 1;
    double sum = 0.0;
    for (int n = 0; n < N; n++) {
        double k = (double)n - M / 2.0;
        double sinc = (k == 0.0) ? (2.0 * fc / fs)
                                 : sin(2.0 * M_PI * fc / fs * k) / (M_PI * k);
        double w = 0.54 - 0.46 * cos(2.0 * M_PI * n / M);  // Hamming
        taps[n] = sinc * w;
        sum += taps[n];
    }
    for (int n = 0; n < N; n++) taps[n] /= sum;            // unity DC gain
}

static int fir_init(FIRDecim *f, int ntaps, int decim, int is_complex,
                    double fs, double fc) {
    f->ntaps = ntaps; f->decim = decim; f->is_complex = is_complex;
    f->widx = 0; f->phase = 0;
    f->taps = malloc(ntaps * sizeof(double));
    f->ri   = calloc(ntaps, sizeof(double));
    f->rq   = is_complex ? calloc(ntaps, sizeof(double)) : NULL;
    if (!f->taps || !f->ri || (is_complex && !f->rq)) return 0;
    design_lpf(f->taps, ntaps, fs, fc);
    return 1;
}

static void fir_free(FIRDecim *f) { free(f->taps); free(f->ri); free(f->rq); }

// Push one input sample. Returns 1 and fills *outi/*outq when a decimated
// output sample is produced, else 0.
static int fir_push(FIRDecim *f, double ini, double inq, double *outi, double *outq) {
    f->ri[f->widx] = ini;
    if (f->is_complex) f->rq[f->widx] = inq;
    f->widx = (f->widx + 1 == f->ntaps) ? 0 : f->widx + 1;

    if (++f->phase < f->decim) return 0;
    f->phase = 0;

    double acc_i = 0.0, acc_q = 0.0;
    int idx = f->widx - 1; if (idx < 0) idx += f->ntaps;
    for (int k = 0; k < f->ntaps; k++) {
        acc_i += f->taps[k] * f->ri[idx];
        if (f->is_complex) acc_q += f->taps[k] * f->rq[idx];
        if (--idx < 0) idx += f->ntaps;
    }
    *outi = acc_i;
    if (outq) *outq = acc_q;
    return 1;
}

int main(int argc, char **argv) {
    const char *in_path = NULL, *out_path = NULL;
    double deemph_us = DEFAULT_DEEMPH_US;
    int verbose = 0;
    long sdr_rate = DEFAULT_SDR_RATE;

    // Args: <in> <out> [-r <input_rate>] [-e <us>|0=off] [-v]
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-e") == 0 && i + 1 < argc) {
            deemph_us = atof(argv[++i]);
        } else if (strcmp(argv[i], "-r") == 0 && i + 1 < argc) {
            sdr_rate = atol(argv[++i]);
        } else if (strcmp(argv[i], "-v") == 0) {
            verbose = 1;
        } else if (!in_path) {
            in_path = argv[i];
        } else if (!out_path) {
            out_path = argv[i];
        }
    }
    if (!in_path || !out_path) {
        fprintf(stderr, "Usage: %s <input_iq.bin> <output.wav> [-r <input_rate>] [-e <deemph_us>|0] [-v]\n",
                argv[0]);
        return 1;
    }
    if (sdr_rate <= 0 || sdr_rate % INTER_RATE != 0) {
        fprintf(stderr, "Error: input rate %ld must be a positive multiple of %d "
                "(e.g. 1200000 or 2400000). It MUST match fsk_rx -s.\n",
                sdr_rate, INTER_RATE);
        return 1;
    }
    int decim1 = (int)(sdr_rate / INTER_RATE);

    FILE *fin = fopen(in_path, "rb");
    if (!fin) { fprintf(stderr, "Error: cannot open %s\n", in_path); return 1; }
    FILE *fout = fopen(out_path, "wb");
    if (!fout) { fprintf(stderr, "Error: cannot open %s\n", out_path); fclose(fin); return 1; }

    // Size the output buffer from the input file length.
    fseek(fin, 0, SEEK_END);
    long file_bytes = ftell(fin);
    fseek(fin, 0, SEEK_SET);
    long iq_samples = file_bytes / 2;
    long est_audio = iq_samples / ((long)decim1 * DECIM2) + 16;
    float *audio = malloc((est_audio > 0 ? est_audio : 1) * sizeof(float));
    if (!audio) { fprintf(stderr, "Error: OOM\n"); fclose(fin); fclose(fout); return 1; }

    // Filters
    FIRDecim ch, au;
    if (!fir_init(&ch, CH_TAPS, decim1, 1, (double)sdr_rate, CH_CUTOFF) ||
        !fir_init(&au, AUDIO_TAPS, DECIM2, 0, (double)INTER_RATE, AUDIO_CUTOFF)) {
        fprintf(stderr, "Error: filter init failed\n");
        free(audio); fclose(fin); fclose(fout); return 1;
    }

    // Discriminator state (at INTER_RATE)
    double prev_i = 0.0, prev_q = 0.0;
    int have_prev = 0;

    // DC blocker (one-pole high-pass, ~8 Hz corner at 48 kHz) to strip the
    // constant phase-ramp term from any residual carrier offset.
    const double dc_R = 0.95;
    double dc_x1 = 0.0, dc_y1 = 0.0;

    // De-emphasis (one-pole low-pass). pole = exp(-1/(tau*fs)); off if us <= 0.
    int    deemph_on = (deemph_us > 0.0);
    double de_p = deemph_on ? exp(-1.0 / (deemph_us * 1e-6 * AUDIO_RATE)) : 0.0;
    double de_y1 = 0.0;

    printf("[*] Demodulating %s -> %s%s...\n", in_path, out_path,
           deemph_on ? "" : " (de-emphasis off)");
    if (verbose) {
        printf("[*] Input rate: %ld Hz\n", sdr_rate);
        printf("[*] Stage1 IQ LPF: %d taps, fc=%.0f Hz, decim %d -> %d Hz\n",
               CH_TAPS, CH_CUTOFF, decim1, INTER_RATE);
        printf("[*] Stage2 audio LPF: %d taps, fc=%.0f Hz, decim %d -> %d Hz\n",
               AUDIO_TAPS, AUDIO_CUTOFF, DECIM2, AUDIO_RATE);
        if (deemph_on) printf("[*] De-emphasis: %.0f us (pole %.4f)\n", deemph_us, de_p);
    }

    const size_t CHUNK = 1 << 16;             // IQ samples per read
    uint8_t *buf = malloc(CHUNK * 2);
    if (!buf) { fprintf(stderr, "Error: OOM\n"); free(audio); fir_free(&ch); fir_free(&au); fclose(fin); fclose(fout); return 1; }

    long n_audio = 0;
    size_t got;
    while ((got = fread(buf, 2, CHUNK, fin)) > 0) {
        for (size_t s = 0; s < got; s++) {
            double si = ((double)buf[2*s]     - 127.5) / 127.5;
            double sq = ((double)buf[2*s + 1] - 127.5) / 127.5;

            double ci, cq;
            if (!fir_push(&ch, si, sq, &ci, &cq)) continue;   // stage 1 (IQ)

            // Polar discriminator: arg(cur * conj(prev)) — instantaneous freq.
            double freq;
            if (!have_prev) { freq = 0.0; have_prev = 1; }
            else {
                double re = ci * prev_i + cq * prev_q;
                double im = cq * prev_i - ci * prev_q;
                freq = atan2(im, re);
            }
            prev_i = ci; prev_q = cq;

            double af;
            if (!fir_push(&au, freq, 0.0, &af, NULL)) continue; // stage 2 (audio)

            // DC block
            double dc_y = af - dc_x1 + dc_R * dc_y1;
            dc_x1 = af; dc_y1 = dc_y;

            // De-emphasis
            double out;
            if (deemph_on) { de_y1 = (1.0 - de_p) * dc_y + de_p * de_y1; out = de_y1; }
            else           { out = dc_y; }

            if (n_audio < est_audio) audio[n_audio++] = (float)out;
        }
    }

    // Normalize to ~0.9 full scale using a high-percentile reference rather than the
    // global peak. The peak is a single sample: one transient anywhere in the capture
    // (a neighbor keying up, a static crash) would otherwise scale the whole file down
    // and starve the FSK below the demod squelch. FM-discriminated FSK is near
    // constant-envelope, so the 99.5th percentile tracks the real signal level while
    // ignoring outliers. Samples above the reference are hard-limited on write.
    double ref = 0.0;
    if (n_audio > 0) {
        float *mags = malloc(n_audio * sizeof(float));
        if (mags) {
            for (long i = 0; i < n_audio; i++) mags[i] = fabsf(audio[i]);
            qsort(mags, n_audio, sizeof(float), cmp_float);
            long pidx = (long)(0.995 * (double)(n_audio - 1));
            if (pidx < 0) pidx = 0;
            ref = mags[pidx];
            free(mags);
        } else {
            for (long i = 0; i < n_audio; i++) { double a = fabs(audio[i]); if (a > ref) ref = a; }
        }
    }
    double scale = (ref > 1e-9) ? (0.9 * 32767.0 / ref) : 0.0;

    struct WavHeader header;
    memset(&header, 0, sizeof(header));
    fwrite(&header, sizeof(header), 1, fout);   // placeholder

    for (long i = 0; i < n_audio; i++) {
        double v = audio[i] * scale;
        if (v > 32767.0) v = 32767.0; else if (v < -32768.0) v = -32768.0;
        int16_t pcm = (int16_t)lround(v);
        fwrite(&pcm, sizeof(int16_t), 1, fout);
    }

    uint32_t data_bytes = (uint32_t)(n_audio * sizeof(int16_t));
    memcpy(header.riff_id, "RIFF", 4);
    header.chunk_size = 36 + data_bytes;
    memcpy(header.wave_id, "WAVE", 4);
    memcpy(header.fmt_id, "fmt ", 4);
    header.fmt_size = 16;
    header.audio_format = 1;
    header.num_channels = 1;
    header.sample_rate = AUDIO_RATE;
    header.byte_rate = AUDIO_RATE * sizeof(int16_t);
    header.block_align = sizeof(int16_t);
    header.bits_per_sample = 16;
    memcpy(header.data_id, "data", 4);
    header.data_size = data_bytes;
    fseek(fout, 0, SEEK_SET);
    fwrite(&header, sizeof(header), 1, fout);

    printf("[+] Demodulation complete. Processed %ld audio samples.\n", n_audio);

    free(buf); free(audio);
    fir_free(&ch); fir_free(&au);
    fclose(fin); fclose(fout);
    return 0;
}
