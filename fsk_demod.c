#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

// === STANDARD PACKET RADIO PARAMETERS ===
#define SAMPLE_RATE      48000
#define BAUD_RATE        100                       // matches the modulator
#define SAMPLES_PER_SYM  (SAMPLE_RATE / BAUD_RATE) // 160 samples/symbol

#define TONE_MARK        1200.0f                   // binary 1 (Bell 202 Mark)
#define TONE_SPACE       2400.0f                   // binary 0 (Bell 202 Space)

#define PREAMBLE_BITS    128
#define SYNC_BITS        16
#define SYNC_WORD        0xD3D3
#define MAX_PAYLOAD      2048

// === CONVOLUTIONAL FEC (replaces Hamming(7,4) + depth-32 interleaver) ===
// K=7, rate 1/2, NASA-standard generators (171, 133 octal), free distance 10.
// The transmitter's encoder is a 6-bit shift register seeded to zero and
// flushed with CONV_FLUSH zero bits, so the trellis terminates in state 0 and
// the Viterbi traceback below anchors there. MUST match fsk_mod.c exactly.
#define CONV_G1     0x79    // 171 octal = 1111001b
#define CONV_G2     0x5B    // 133 octal = 1011011b
#define CONV_FLUSH  6       // K-1 termination bits
#define CONV_STATES 64      // 2^(K-1)

// === FRAME HEADER ===
// [len_hi, len_lo, CRC-8(len)] -> 24 info bits + 6 flush -> 60 coded bits,
// convolutionally coded but NOT interleaved, sitting right after the sync
// word. The receiver decodes this first: the full-frame interleaver geometry
// depends on the length, so the length must be recoverable BEFORE any
// de-interleaving. The CRC-8 rejects a corrupted-but-plausible length before
// it can scramble the geometry. MUST match fsk_mod.c.
#define HDR_INFO_BITS   24
#define HDR_STEPS       (HDR_INFO_BITS + CONV_FLUSH)   // 30 trellis steps
#define HDR_CODED_BITS  (HDR_STEPS * 2)                // 60 on-air bits

// === CHASE COMBINING (time diversity, modulator '-r N') ===
// The modulator's -r flag repeats the ENTIRE frame (preamble+sync+header+body)
// back-to-back. Each copy is independently acquirable: it carries its own
// preamble and sync word, so each copy's sync position anchors its own body
// region on the continuously-tracked symbol grid.
//
// The decode ladder, cheapest rung first:
//   1. SELECTION  - decode each copy alone, in arrival order; first CRC pass
//                   wins. (This alone fixes a real gap: the old code decoded
//                   only the FIRST sync it found, so a body-burst on copy 1
//                   failed the whole capture with a clean copy 2 unread.)
//   2. COMBINING  - if every copy fails alone, sum the copies' bipolar soft
//                   streams position-by-position in the natural (coded) bit
//                   order and Viterbi-decode the sum once. The soft values
//                   are sign*reliability, so the sum is maximal-ratio
//                   combining: a strong bit in one copy outvotes a faded bit
//                   in another, erasures (0.0) contribute nothing, and two
//                   confident disagreements cancel to an erasure -- which is
//                   the honest answer. In AWGN two combined copies are worth
//                   ~3 dB over either alone; against bursts each coded bit
//                   effectively takes the best evidence any copy has.
// The same ladder applies to the 60-bit length header (per-copy CRC-8 first,
// combined-header rescue second). No modulator or on-air format change.
#define MAX_COPIES  8       // sync positions tracked per capture

// === COMPILE-TIME WORST-CASE SIZING (mirrors fsk_mod.c) ===
#define PACKED_MAX      ((MAX_PAYLOAD * 7 + 7) / 8)
#define BODY_BYTES_MAX  (PACKED_MAX + 2)
#define BODY_STEPS_MAX  ((size_t)BODY_BYTES_MAX * 8 + CONV_FLUSH)
#define BODY_CODED_MAX  (BODY_STEPS_MAX * 2)

// --- Feature 2: soft-decision / erasure decoding ---
// Per-bit reliability below this is treated as a full erasure (free to flip).
// The Viterbi branch metrics already down-weight low-reliability bits
// smoothly; this just floors the worst symbols (deep fades / pure ambiguity)
// to exactly zero weight so they contribute nothing to any path.
#define ERASURE_FLOOR    0.15f

// --- Refined correlator / timing-recovery tuning ---
// INTEG_LEN and GATE are now DERIVED from the symbol length instead of being
// hand constants, so a baud change is a one-line edit on both ends. At 100
// baud these reproduce the proven values exactly (432 and 120); at 300 baud
// they become 144 and 40. INTEG_LEN integrates 90% of the symbol, leaving 10%
// slack so a slightly off-center grid still reads clean tone; GATE splits the
// early/late probes a quarter symbol out (the old hand value 60 overlapped
// the on-time window ~72% and made the timing-error detector nearly inert).
#define INTEG_LEN        (SAMPLES_PER_SYM * 9 / 10)
#define GATE             (SAMPLES_PER_SYM / 4)
#define SLEW_HYST        1.01    // Snaps early/late transitions tightly
#define ACQ_SYMS         16      // preamble symbols used for fine phase acquisition

// PASS 1 coarse-acquisition tuning. With only 4 phase hypotheses the worst-case
// symbol-center error was ~60 samples, contaminating ~12% of every INTEG_LEN
// correlation and randomly tripping the preamble gate depending on packet
// arrival phase. 16 steps -> 30-sample worst case (~6% contamination).
#define PHASE_STEPS      16
#define PREAMBLE_GATE    0.75    // fraction of preamble bits that must match. The
                                 // 16-bit sync at <=1 mismatch is already a strong
                                 // gate (~1/3800 false-positive/position); 0.90 here
                                 // mostly produced false negatives.

// Timing invariants enforced at compile time: the symbol grid must divide the
// sample rate evenly, the correlator must fit inside one symbol with slack,
// and PASS 1's fractional phase scan must have at least 1-sample granularity.
_Static_assert(SAMPLE_RATE % BAUD_RATE == 0,
               "SAMPLES_PER_SYM must be an integer: SAMPLE_RATE % BAUD_RATE != 0");
_Static_assert(INTEG_LEN < SAMPLES_PER_SYM,
               "correlator window must leave timing slack inside the symbol");
_Static_assert(SAMPLES_PER_SYM / PHASE_STEPS >= 1,
               "PHASE_STEPS too fine for this symbol length");

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Precomputed correlation tables (filled by init_tables)
static double cos_mark[INTEG_LEN], sin_mark[INTEG_LEN];
static double cos_space[INTEG_LEN], sin_space[INTEG_LEN];

// === STATIC VITERBI / COMBINING WORKING STORAGE (BSS, no heap) ===
// Sized for the worst-case body at MAX_PAYLOAD: soft bits ~115 KB, the chase
// accumulator another ~115 KB, traceback decisions ~115 KB, info bits ~14 KB.
// Fixed at link time, guarded at runtime.
static float    vb_soft[BODY_CODED_MAX];     // current copy's natural-order bipolar soft bits
static float    comb_soft[BODY_CODED_MAX];   // chase accumulator: sum of all failed copies
static uint64_t vb_dec[BODY_STEPS_MAX];      // traceback decisions
static float    vb_pm[2][CONV_STATES];       // path metrics, ping-pong
static uint8_t  vb_info[BODY_STEPS_MAX];     // decoded info bits incl. flush
static uint8_t  body_decoded[BODY_BYTES_MAX];// decoded body bytes
static uint8_t  reenc_bits[BODY_CODED_MAX];  // re-encoded stream for -v stats

// Frame geometry, derived identically on both ends from the header's text_len.
typedef struct {
    size_t text_len;    // characters in the payload
    size_t packed_len;  // ceil(text_len*7/8) packed bytes
    size_t body_len;    // packed + CRC16
    size_t steps;       // trellis steps: body_len*8 + flush
    size_t n_coded;     // coded bits: steps*2
    size_t R, C;        // interleaver grid
    size_t air_bits;    // R*C on-air body bits
} frame_geom;

static uint16_t crc16(const uint8_t *data, size_t len);
static uint8_t crc8(const uint8_t *data, size_t len);
static int read_wav(const char *path, int16_t **buffer, uint32_t *num_samples);
static int find_sync_from(const uint8_t *bits, size_t num_bits, size_t start);
static size_t unpack7(const uint8_t *packed, size_t n_chars, uint8_t *out);
static float bit_reliability(float conf, double energy, double eref);
static double viterbi_decode(const float *soft, size_t steps, uint8_t *info_out);
static int derive_geometry(size_t text_len, frame_geom *g);
static int header_from_soft(const float *hsoft, size_t *text_len_out, double *metric_out);
static int decode_body_soft(const float *soft, const frame_geom *g,
                            uint8_t *payload, size_t *payload_len,
                            int verbose, const char *tag);
static int decode_with_diversity(const uint8_t *bits, const float *confidences,
                                 const double *energies, size_t num_symbols,
                                 const int *syncs, const double *erefs, int n_copies,
                                 uint8_t *payload, size_t *payload_len, int verbose);

static void init_tables(void) {
    for (int s = 0; s < INTEG_LEN; s++) {
        double rad_mark  = 2.0 * M_PI * TONE_MARK  * ((double)s / SAMPLE_RATE);
        double rad_space = 2.0 * M_PI * TONE_SPACE * ((double)s / SAMPLE_RATE);
        cos_mark[s]  = cos(rad_mark);   sin_mark[s]  = sin(rad_mark);
        cos_space[s] = cos(rad_space);  sin_space[s] = sin(rad_space);
    }
}

// Coherent tone energies over INTEG_LEN samples starting at `offset`.
static void tone_energies(const int16_t *x, size_t n, long offset,
                          double *mark_e, double *space_e) {
    *mark_e = 0.0;
    *space_e = 0.0;
    if (offset < 0 || (size_t)(offset + INTEG_LEN) > n) return;

    double Im = 0, Qm = 0, Is = 0, Qs = 0;
    for (int s = 0; s < INTEG_LEN; s++) {
        double v = (double)x[offset + s] / 32767.0;
        Im += v * cos_mark[s];   Qm += v * sin_mark[s];
        Is += v * cos_space[s];  Qs += v * sin_space[s];
    }
    *mark_e  = Im * Im + Qm * Qm;
    *space_e = Is * Is + Qs * Qs;
}

static inline double dominant(double me, double se) { return me > se ? me : se; }

static uint16_t crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else crc <<= 1;
        }
    }
    return crc;
}

// CRC-8 (poly 0x07, init 0x00) guarding the 2-byte length header. MUST match
// fsk_mod.c.
static uint8_t crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80) crc = (uint8_t)((crc << 1) ^ 0x07);
            else crc = (uint8_t)(crc << 1);
        }
    }
    return crc;
}

// Inverse of the modulator's pack7: read n_chars 7-bit characters MSB-first from a
// dense bitstream into out[]. Returns n_chars. The caller knows n_chars from the
// frame's coded length header, so there is no terminator to scan for.
static size_t unpack7(const uint8_t *packed, size_t n_chars, uint8_t *out) {
    size_t bit = 0;
    for (size_t i = 0; i < n_chars; i++) {
        uint8_t c = 0;
        for (int b = 6; b >= 0; b--) {
            int v = (packed[bit >> 3] >> (7 - (bit & 7))) & 1;
            c |= (uint8_t)(v << b);
            bit++;
        }
        out[i] = c;
    }
    return n_chars;
}

static int read_wav(const char *path, int16_t **buffer, uint32_t *num_samples) {
    FILE *f = fopen(path, "rb");
    if (!f) return 0;

    char chunk_id[4]; uint32_t chunk_size; char format[4];
    if (fread(chunk_id, 1, 4, f) != 4 || memcmp(chunk_id, "RIFF", 4) != 0) { fclose(f); return 0; }
    if (fread(&chunk_size, 4, 1, f) != 1) { fclose(f); return 0; }
    if (fread(format, 1, 4, f) != 4 || memcmp(format, "WAVE", 4) != 0) { fclose(f); return 0; }

    char subchunk_id[4]; uint32_t subchunk_size;
    uint16_t audio_format = 0, num_channels = 0, bits_per_sample = 0; uint32_t sample_rate = 0;

    while (fread(subchunk_id, 1, 4, f) == 4) {
        if (fread(&subchunk_size, 4, 1, f) != 1) { fclose(f); return 0; }
        if (memcmp(subchunk_id, "fmt ", 4) == 0) {
            if (fread(&audio_format, 2, 1, f) != 1) { fclose(f); return 0; }
            if (fread(&num_channels, 2, 1, f) != 1) { fclose(f); return 0; }
            if (fread(&sample_rate, 4, 1, f) != 1) { fclose(f); return 0; }
            if (fseek(f, 6, SEEK_CUR) != 0) { fclose(f); return 0; }
            if (fread(&bits_per_sample, 2, 1, f) != 1) { fclose(f); return 0; }
            if (audio_format != 1 || bits_per_sample != 16 || num_channels != 1) { fclose(f); return 0; }
            if (subchunk_size > 16) fseek(f, subchunk_size - 16, SEEK_CUR);
        } else if (memcmp(subchunk_id, "data", 4) == 0) {
            *num_samples = subchunk_size / sizeof(int16_t);
            if (*num_samples == 0) { fclose(f); return 0; }
            *buffer = (int16_t*)malloc(*num_samples * sizeof(int16_t));
            if (!*buffer) { fclose(f); return 0; }
            if (fread(*buffer, sizeof(int16_t), *num_samples, f) != *num_samples) { free(*buffer); fclose(f); return 0; }
            fclose(f); return 1;
        } else { fseek(f, subchunk_size, SEEK_CUR); }
    }
    fclose(f); return 0;
}

// Scan for a sync word at or after bit position `start`, returning the FIRST
// position that clears both gates (<=1 sync mismatch AND the preamble gate),
// or -1. Semantics change from the old global-best find_sync, deliberate: the
// multi-copy scan wants syncs in arrival order, and a rare pseudo-sync that
// slips both gates is now self-correcting -- its header fails CRC-8, the scan
// continues past it, and the real sync becomes the next copy. The preamble
// gate is what keeps false positives out of the body: a candidate needs >=96
// of the 128 preceding bits alternating, and coded body data can't sustain
// that.
static int find_sync_from(const uint8_t *bits, size_t num_bits, size_t start) {
    if (num_bits < (size_t)(PREAMBLE_BITS + SYNC_BITS)) return -1;
    size_t first = (start > (size_t)PREAMBLE_BITS) ? start : (size_t)PREAMBLE_BITS;

    for (size_t i = first; i + SYNC_BITS <= num_bits; i++) {
        int mismatches = 0;
        for (int j = 0; j < SYNC_BITS; j++) {
            int expected = (SYNC_WORD >> (SYNC_BITS - 1 - j)) & 1;
            if (bits[i + j] != expected) mismatches++;
        }
        if (mismatches > 1) continue;

        int preamble_matches = 0;
        for (int p = 0; p < PREAMBLE_BITS; p++) {
            int expected = (p % 2) ? 1 : 0;
            if (bits[i - PREAMBLE_BITS + p] == expected) preamble_matches++;
        }
        if (preamble_matches < (int)(PREAMBLE_BITS * PREAMBLE_GATE)) continue;

        return (int)i;
    }
    return -1;
}

// Per-bit reliability in [0,1]. Combines RELATIVE tone confidence (how cleanly one
// tone beat the other) with ABSOLUTE energy versus the in-packet nominal: a deep
// fade has little energy at all, and its |mark-space| split is then just noise that
// can look falsely confident -- so a faded symbol must be down-weighted even if its
// raw confidence is high. Uniform attenuation (all symbols a bit quieter than the
// reference) scales every cost equally and changes nothing; only RELATIVE dips --
// the fades -- pull a bit toward erasure.
static float bit_reliability(float conf, double energy, double eref) {
    double efrac = (eref > 1e-12) ? (energy / eref) : 1.0;
    if (efrac > 1.0) efrac = 1.0;
    double rel = (double)conf * efrac;
    if (rel < ERASURE_FLOOR) rel = 0.0;   // hard-erase the worst symbols
    return (float)rel;
}

// One trellis step of the K=7 rate-1/2 encoder (transmit-side model). The
// 6-bit state holds the six previous input bits, newest at bit 5; the tap
// window is the current input above the state. MUST match fsk_mod.c.
static inline void conv_step(int state, int u, int *c1, int *c2) {
    int w = (u << 6) | state;
    *c1 = __builtin_parity(w & CONV_G1);
    *c2 = __builtin_parity(w & CONV_G2);
}

// =============================================================================
// SOFT-DECISION VITERBI DECODER
//
// soft[2t], soft[2t+1] are the two coded bits of trellis step t as BIPOLAR
// soft values: sign = the hard tone decision (+ for mark/1, - for space/0),
// magnitude = bit_reliability() in [0,1]. A branch expecting coded bits
// (c1, c2) scores (c1 ? +1 : -1)*soft[2t] + (c2 ? +1 : -1)*soft[2t+1] -- a
// correlation metric. A confident matching bit pulls its path up, a confident
// mismatch pulls it down, and an erased bit (magnitude 0) is free to be
// either: errors-and-erasures decoding falls out of the arithmetic with no
// special cases.
//
// The correlation metric is LINEAR in the soft values, which is why chase
// combining needs no decoder changes at all: Viterbi(soft_1 + soft_2) is the
// joint maximum-likelihood decode of both observations, magnitudes > 1.0 in
// the summed stream simply carry proportionally more weight.
//
// The trellis has 64 states (the encoder's 6 register bits). Each step keeps
// only the best path into each state -- the Viterbi insight: any path that
// arrives at a state with a worse metric can never later beat one that
// arrived better, because their futures are identical. Decisions are stored
// as one bit per state per step (a uint64_t per step) and walked backward
// from state 0, where the transmitter's flush bits guarantee the true path
// ends.
//
// Returns the winning path metric (diagnostic); decoded info bits (including
// the CONV_FLUSH tail) land in info_out, one per step.
// =============================================================================
static double viterbi_decode(const float *soft, size_t steps, uint8_t *info_out) {
    const float NEG = -1e30f;

    float *pm_prev = vb_pm[0];
    float *pm_next = vb_pm[1];
    for (int s = 0; s < CONV_STATES; s++) pm_prev[s] = NEG;
    pm_prev[0] = 0.0f;   // encoder starts in state 0

    for (size_t t = 0; t < steps; t++) {
        float s0 = soft[2 * t];
        float s1 = soft[2 * t + 1];
        uint64_t dec = 0;

        for (int ns = 0; ns < CONV_STATES; ns++) {
            int u = ns >> 5;                       // input bit that leads into ns
            int pa = ((ns & 0x1F) << 1);           // predecessor with LSB 0
            int pb = pa | 1;                       // predecessor with LSB 1

            int c1, c2;
            conv_step(pa, u, &c1, &c2);
            float bma = (c1 ? s0 : -s0) + (c2 ? s1 : -s1);
            conv_step(pb, u, &c1, &c2);
            float bmb = (c1 ? s0 : -s0) + (c2 ? s1 : -s1);

            float ma = pm_prev[pa] + bma;
            float mb = pm_prev[pb] + bmb;
            if (mb > ma) {
                pm_next[ns] = mb;
                dec |= (1ULL << ns);
            } else {
                pm_next[ns] = ma;
            }
        }

        vb_dec[t] = dec;
        float *tmp = pm_prev; pm_prev = pm_next; pm_next = tmp;
    }

    // Traceback from state 0: the flush bits force the true path to terminate
    // there. Each state's top bit IS the input bit that created it.
    int state = 0;
    double final_metric = (double)pm_prev[0];
    for (size_t t = steps; t-- > 0; ) {
        info_out[t] = (uint8_t)(state >> 5);
        int b = (int)((vb_dec[t] >> state) & 1);
        state = ((state & 0x1F) << 1) | b;
    }
    return final_metric;
}

// Integer floor(sqrt(n)) for the interleaver geometry. MUST match fsk_mod.c.
static size_t isqrt_floor(size_t n) {
    size_t r = 1;
    while ((r + 1) * (r + 1) <= n) r++;
    return r;
}

// Bipolar soft value for on-air symbol index si, or 0 (erasure) if the symbol
// is beyond the demodulated region -- a fade that ate the tail of the frame
// degrades into erasures instead of garbage.
static inline float soft_of_symbol(const uint8_t *bits, const float *confidences,
                                   const double *energies, double eref,
                                   size_t num_symbols, size_t si) {
    if (si >= num_symbols) return 0.0f;
    float rel = bit_reliability(confidences[si], energies[si], eref);
    return bits[si] ? rel : -rel;
}

// Both ends derive the identical geometry from text_len. Returns 1 if it fits
// the compile-time decoder envelope, 0 if the (necessarily corrupt) length
// would overrun it.
static int derive_geometry(size_t text_len, frame_geom *g) {
    g->text_len   = text_len;
    g->packed_len = (text_len * 7 + 7) / 8;
    g->body_len   = g->packed_len + 2;                 // packed + CRC16
    g->steps      = g->body_len * 8 + CONV_FLUSH;
    g->n_coded    = g->steps * 2;
    if (g->body_len > BODY_BYTES_MAX || g->steps > BODY_STEPS_MAX
        || g->n_coded > BODY_CODED_MAX) return 0;
    g->R = isqrt_floor(g->n_coded);
    g->C = (g->n_coded + g->R - 1) / g->R;
    g->air_bits = g->R * g->C;
    return 1;
}

// Gather one copy's 60 header soft bits, anchored at that copy's sync.
static void gather_header_soft(const uint8_t *bits, const float *confidences,
                               const double *energies, double eref,
                               size_t num_symbols, int sync_pos, float *hsoft) {
    size_t hdr_start = (size_t)sync_pos + SYNC_BITS;
    for (int i = 0; i < HDR_CODED_BITS; i++) {
        hsoft[i] = soft_of_symbol(bits, confidences, energies, eref,
                                  num_symbols, hdr_start + (size_t)i);
    }
}

// Viterbi-decode a 60-bit header soft stream (per-copy or chase-combined) and
// validate its CRC-8. On pass, writes text_len_out and returns 1.
static int header_from_soft(const float *hsoft, size_t *text_len_out, double *metric_out) {
    uint8_t hdr_info[HDR_STEPS];
    double m = viterbi_decode(hsoft, HDR_STEPS, hdr_info);
    if (metric_out) *metric_out = m;

    uint8_t hdr[3] = {0, 0, 0};
    for (int i = 0; i < HDR_INFO_BITS; i++) {
        if (hdr_info[i]) hdr[i >> 3] |= (uint8_t)(0x80 >> (i & 7));
    }

    size_t text_len = ((size_t)hdr[0] << 8) | hdr[1];
    if (crc8(hdr, 2) != hdr[2] || text_len == 0 || text_len > MAX_PAYLOAD) return 0;
    *text_len_out = text_len;
    return 1;
}

// De-interleave one copy's body into natural-order soft bits, anchored at that
// copy's sync. Coded bit n was transmitted at air offset row*C + col where
// row = n % R, col = n / R. Each bit's reliability rides through the SAME
// permutation as the bit itself, so a faded symbol's low weight stays glued to
// the bit it produced -- and a contiguous on-air burst lands as isolated soft
// errors spaced R apart in the coded stream, exactly what Viterbi corrects
// best. Symbols past the demodulated region come back as erasures.
static void gather_body_soft(const uint8_t *bits, const float *confidences,
                             const double *energies, double eref,
                             size_t num_symbols, int sync_pos,
                             const frame_geom *g, float *soft_out) {
    size_t body_start = (size_t)sync_pos + SYNC_BITS + HDR_CODED_BITS;
    for (size_t n = 0; n < g->n_coded; n++) {
        size_t row = n % g->R;
        size_t col = n / g->R;
        size_t si = body_start + row * g->C + col;
        soft_out[n] = soft_of_symbol(bits, confidences, energies, eref,
                                     num_symbols, si);
    }
}

// Viterbi-decode a natural-order body soft stream (per-copy or chase-combined)
// and verify the CRC-16. Always deposits the decoded payload (best effort even
// on a CRC fail, matching the old behavior); returns 1 only on a CRC pass.
static int decode_body_soft(const float *soft, const frame_geom *g,
                            uint8_t *payload, size_t *payload_len,
                            int verbose, const char *tag) {
    double body_metric = viterbi_decode(soft, g->steps, vb_info);

    memset(body_decoded, 0, g->body_len);
    for (size_t i = 0; i < g->body_len * 8; i++) {
        if (vb_info[i]) body_decoded[i >> 3] |= (uint8_t)(0x80 >> (i & 7));
    }

    if (verbose) {
        // Corrected-channel-bits estimate: re-encode the decoded stream and
        // count where it disagrees with the (possibly combined) soft signs
        // actually received, over bits that were not erased.
        int state = 0;
        size_t reenc_n = 0;
        for (size_t i = 0; i < g->steps; i++) {
            int u = (i < g->body_len * 8) ? ((body_decoded[i >> 3] >> (7 - (i & 7))) & 1) : 0;
            int c1, c2;
            conv_step(state, u, &c1, &c2);
            reenc_bits[reenc_n++] = (uint8_t)c1;
            reenc_bits[reenc_n++] = (uint8_t)c2;
            state = ((u << 5) | (state >> 1)) & 0x3F;
        }
        size_t flips = 0, erased = 0;
        for (size_t n = 0; n < g->n_coded; n++) {
            if (soft[n] == 0.0f) { erased++; continue; }
            int hard = (soft[n] > 0.0f) ? 1 : 0;
            if (hard != (int)reenc_bits[n]) flips++;
        }
        fprintf(stderr, "[+] FEC Engine (%s): Viterbi corrected %zu channel bit-flips (%zu erased) of %zu coded bits, metric %.1f\n",
                tag, flips, erased, g->n_coded, body_metric);
    }

    uint16_t recv_crc = ((uint16_t)body_decoded[g->packed_len] << 8)
                       | body_decoded[g->packed_len + 1];

    unpack7(body_decoded, g->text_len, payload);
    uint16_t calc_crc = crc16(payload, g->text_len);
    *payload_len = g->text_len;

    if (verbose) {
        fprintf(stderr, "[*] %s: Received CRC: 0x%04X, Calculated: 0x%04X -> %s\n",
                tag, recv_crc, calc_crc, (recv_crc == calc_crc) ? "PASS" : "FAIL");
    }

    return (recv_crc == calc_crc) ? 1 : 0;
}

// =============================================================================
// TIME-DIVERSITY DECODE LADDER (selection first, chase combining as rescue)
//
// Resolve the length: each copy's header alone in arrival order, then the
// chase-combined header if every copy's failed CRC-8. Then the body: each
// copy alone in arrival order (first CRC-16 pass wins -- identical airtime
// cost to today, and the common clean-channel case exits on copy 1 exactly as
// before), accumulating each failed copy's soft stream; if all copies fail,
// one Viterbi run over the accumulated sum. The accumulator is only as good
// as what goes into it, which is why per-copy sync anchoring matters: the
// natural-order coded index n is aligned across copies by construction, each
// copy's body indexed from its OWN sync on the shared symbol grid.
// =============================================================================
static int decode_with_diversity(const uint8_t *bits, const float *confidences,
                                 const double *energies, size_t num_symbols,
                                 const int *syncs, const double *erefs, int n_copies,
                                 uint8_t *payload, size_t *payload_len, int verbose) {
    *payload_len = 0;

    // --- Resolve the frame length ---
    float hsoft[HDR_CODED_BITS];
    float hcomb[HDR_CODED_BITS];
    memset(hcomb, 0, sizeof(hcomb));

    size_t text_len = 0;
    int have_len = 0;
    double hdr_metric = 0.0;

    for (int c = 0; c < n_copies; c++) {
        gather_header_soft(bits, confidences, energies, erefs[c],
                           num_symbols, syncs[c], hsoft);
        for (int i = 0; i < HDR_CODED_BITS; i++) hcomb[i] += hsoft[i];

        if (!have_len && header_from_soft(hsoft, &text_len, &hdr_metric)) {
            have_len = 1;
            if (verbose) fprintf(stderr, "[+] Length header: copy %d CRC-8 pass (metric %.1f)\n",
                                 c + 1, hdr_metric);
        }
    }
    if (!have_len && n_copies >= 2) {
        if (header_from_soft(hcomb, &text_len, &hdr_metric)) {
            have_len = 1;
            if (verbose) fprintf(stderr, "[+] Length header: rescued by chase combining %d copies (metric %.1f)\n",
                                 n_copies, hdr_metric);
        }
    }
    if (!have_len) {
        if (verbose) fprintf(stderr, "[-] Bad length header on all %d cop%s (and combined); frame rejected\n",
                             n_copies, n_copies == 1 ? "y" : "ies");
        return 0;
    }

    frame_geom g;
    if (!derive_geometry(text_len, &g)) {
        if (verbose) fprintf(stderr, "[-] Header length exceeds decoder envelope; frame rejected\n");
        return 0;
    }

    if (verbose) {
        fprintf(stderr, "[+] Header: %zu chars (%zu packed bytes), interleaver %zux%zu\n",
                g.text_len, g.packed_len, g.R, g.C);
        for (int c = 0; c < n_copies; c++) {
            size_t frame_end = (size_t)syncs[c] + SYNC_BITS + HDR_CODED_BITS + g.air_bits;
            fprintf(stderr, "[+] Structural Squelch: copy %d body spans bits %zu..%zu.%s\n",
                    c + 1, (size_t)syncs[c] + SYNC_BITS + HDR_CODED_BITS, frame_end,
                    frame_end > num_symbols ? " Tail past capture treated as erasures." : "");
        }
    }

    // --- Body ladder: selection, then chase combining ---
    memset(comb_soft, 0, g.n_coded * sizeof(float));

    for (int c = 0; c < n_copies; c++) {
        gather_body_soft(bits, confidences, energies, erefs[c],
                         num_symbols, syncs[c], &g, vb_soft);
        for (size_t n = 0; n < g.n_coded; n++) comb_soft[n] += vb_soft[n];

        char tag[24];
        snprintf(tag, sizeof(tag), "copy %d", c + 1);
        if (decode_body_soft(vb_soft, &g, payload, payload_len, verbose, tag)) {
            return 1;
        }
    }

    if (n_copies >= 2) {
        if (decode_body_soft(comb_soft, &g, payload, payload_len, verbose, "chase-combined")) {
            if (verbose) fprintf(stderr, "[+] Chase combining rescued the frame: all %d copies failed alone\n",
                                 n_copies);
            return 1;
        }
    }

    return 0;   // best-effort payload from the last attempt is already deposited
}

int main(int argc, char **argv) {
    if (argc < 2) { fprintf(stderr, "Usage: %s <input.wav> [-v]\n", argv[0]); return 1; }
    int verbose = 0; const char *wavfile = argv[1];
    if (argc > 2 && strcmp(argv[2], "-v") == 0) verbose = 1;

    init_tables();
    int16_t *audio = NULL; uint32_t num_samples = 0;
    if (!read_wav(wavfile, &audio, &num_samples)) { fprintf(stderr, "Failed to read WAV file\n"); return 1; }

    // === LEADING-EDGE SQUELCH ===
    uint32_t start_sample = 0; double energy_threshold = 0.03; int squelch_broken = 0;
    for (uint32_t i = 0; i + 2 * SAMPLES_PER_SYM < num_samples; i += SAMPLES_PER_SYM) {
        double rms = 0.0, rms2 = 0.0;
        for (int s = 0; s < SAMPLES_PER_SYM; s++) {
            rms += ((double)audio[i + s] / 32767.0) * ((double)audio[i + s] / 32767.0);
            rms2 += ((double)audio[i + SAMPLES_PER_SYM + s] / 32767.0) * ((double)audio[i + SAMPLES_PER_SYM + s] / 32767.0);
        }
        if (sqrt(rms / SAMPLES_PER_SYM) > energy_threshold && sqrt(rms2 / SAMPLES_PER_SYM) > energy_threshold * 0.5) {
            start_sample = i; squelch_broken = 1; break;
        }
    }
    if (!squelch_broken) { fprintf(stderr, "[-] Error: Squelch remained unbroken.\n"); free(audio); return 1; }

    // Back up one symbol: the squelch advances in whole symbols from sample 0, so it
    // can trip a symbol late and cost us front-of-preamble symbols (find_sync's
    // look-back window would then read into pre-signal noise). One symbol of slack
    // is free here because PASS 1 scans forward for the sync word regardless.
    if (start_sample >= (uint32_t)SAMPLES_PER_SYM) start_sample -= SAMPLES_PER_SYM;

    // ============================================================
    // PASS 1 - Phase-Slip Synchronized Grid Alignment
    // Try fractional symbol phase steps to guarantee center-sampling
    // ============================================================
    size_t coarse_syms = (num_samples - start_sample) / SAMPLES_PER_SYM;
    if (coarse_syms < (size_t)(PREAMBLE_BITS + SYNC_BITS)) { free(audio); return 1; }

    int coarse_sync = -1;
    long synchronized_start_sample = start_sample;

    // Scan in finer phase steps (SAMPLES_PER_SYM / PHASE_STEPS increments)
    for (int phase_step = 0; phase_step < PHASE_STEPS; phase_step++) {
        long test_offset = (long)start_sample + (phase_step * (SAMPLES_PER_SYM / PHASE_STEPS));
        size_t test_syms = (num_samples - test_offset) / SAMPLES_PER_SYM;

        uint8_t *coarse_bits = (uint8_t*)calloc(test_syms, 1);
        if (!coarse_bits) continue;

        for (size_t i = 0; i < test_syms; i++) {
            double me, se;
            tone_energies(audio, num_samples, test_offset + (long)i * SAMPLES_PER_SYM, &me, &se);
            coarse_bits[i] = (me > se) ? 1 : 0;
        }

        int sync_found = find_sync_from(coarse_bits, test_syms, 0);
        free(coarse_bits);

        if (sync_found >= 0) {
            coarse_sync = sync_found;
            synchronized_start_sample = test_offset;
            if (verbose) fprintf(stderr, "[+] Phase-Slip Lock! Grid phase verified on step %d\n", phase_step);
            break;
        }
    }

    if (coarse_sync < 0) {
        fprintf(stderr, "No sync word found\n");
        free(audio);
        return 1;
    }

    long preamble_sym = coarse_sync - PREAMBLE_BITS;
    long acq_base = synchronized_start_sample + preamble_sym * SAMPLES_PER_SYM;
    if (acq_base < 0) acq_base = 0;

    // ============================================================
    // PASS 2 - Fine Timing (early/late tracking from the PASS-1 grid)
    // ============================================================
    // PASS 1 already locked the symbol phase to within +/-(SAMPLES_PER_SYM/
    // PHASE_STEPS)/2 samples on a grid where the sync word demonstrably decodes
    // (<=1 mismatch by construction), and INTEG_LEN < SAMPLES_PER_SYM leaves ample
    // slack to read clean tones at that phase. The old energy-maximizing best_off
    // search re-derived timing over the ACQ_SYMS preamble symbols, but the preamble
    // ALTERNATES tones, so a half-symbol slip still lands on a tone and scores
    // nearly identical energy -- the metric could not find symbol-center and would
    // jump a quarter-symbol or more off a perfectly good lock, losing the sync. We
    // anchor to the PASS-1 grid and let the early/late gate below do the fine work.
    //
    // The grid runs continuously to the end of the capture, so with '-r N' every
    // repeat rides the SAME tracked grid -- the transmitter's phase-continuous
    // back-to-back repeats mean the tracker never sees a discontinuity at the
    // copy boundary, and each copy's own sync word anchors its bit indices.
    long best_off = 0;

    double pos = (double)acq_base + (double)best_off;
    size_t cap = (num_samples - acq_base) / SAMPLES_PER_SYM + 4;
    uint8_t *bits        = (uint8_t*)calloc(cap, sizeof(uint8_t));
    float   *confidences = (float*)  calloc(cap, sizeof(float));
    double  *energies    = (double*) calloc(cap, sizeof(double));

    size_t nsym = 0;
    while (nsym < cap) {
        long base = (long)llround(pos);
        if (base < 0 || (size_t)(base + INTEG_LEN) > num_samples) break;

        double me, se; tone_energies(audio, num_samples, base, &me, &se);
        bits[nsym] = (me > se) ? 1 : 0;
        double tot = me + se;
        confidences[nsym] = (tot > 1e-12) ? (float)(fabs(me - se) / tot) : 0.0f;
        energies[nsym] = dominant(me, se);

        // Early/Late tracking
        double meE, seE, meL, seL;
        tone_energies(audio, num_samples, base - GATE, &meE, &seE);
        tone_energies(audio, num_samples, base + GATE, &meL, &seL);
        double Ee = dominant(meE, seE); double El = dominant(meL, seL);
        //if      (El > Ee * SLEW_HYST) pos += 1.0;
        //else if (Ee > El * SLEW_HYST) pos -= 1.0;
        if      (El > Ee * SLEW_HYST) pos += 0.50;
        else if (Ee > El * SLEW_HYST) pos -= 0.50;

        pos += SAMPLES_PER_SYM; nsym++;
    }

    // === MULTI-COPY SYNC SCAN ===
    // Every '-r N' repeat carries its own preamble and sync, so each copy is
    // located independently: scan resumes just past each hit. Per-copy nominal
    // energy is measured around each copy's OWN sync -- a copy inside a fade
    // gets its reliabilities normalized to its own level, not copy 1's.
    int    syncs[MAX_COPIES];
    double erefs[MAX_COPIES];
    int n_copies = 0;
    {
        size_t scan_from = 0;
        while (n_copies < MAX_COPIES) {
            int sp = find_sync_from(bits, nsym, scan_from);
            if (sp < 0) break;

            double ref = 0.0; int rc = 0;
            for (int k = -16; k < 16; k++) {
                int idx = sp + k;
                if (idx >= 0 && idx < (int)nsym) { ref += energies[idx]; rc++; }
            }
            syncs[n_copies] = sp;
            erefs[n_copies] = rc ? ref / rc : 0.0;
            n_copies++;
            scan_from = (size_t)sp + SYNC_BITS;
        }
    }

    if (n_copies == 0) {
        fprintf(stderr, "No sync word found after timing lock\n");
        free(audio); free(bits); free(confidences); free(energies);
        return 1;
    }
    if (verbose) {
        fprintf(stderr, "[+] Time diversity: %d frame cop%s located (sync at bit%s",
                n_copies, n_copies == 1 ? "y" : "ies", n_copies == 1 ? "" : "s");
        for (int c = 0; c < n_copies; c++) fprintf(stderr, "%s %d", c ? "," : "", syncs[c]);
        fprintf(stderr, ")\n");
    }

    uint8_t payload[MAX_PAYLOAD]; size_t payload_len = 0;
    int crc_ok = decode_with_diversity(bits, confidences, energies, nsym,
                                       syncs, erefs, n_copies,
                                       payload, &payload_len, verbose);

    if (payload_len > 0) {
        fwrite(payload, 1, payload_len, stdout); printf("\n");
    } else { printf("(Empty payload)\n"); }

    free(audio); free(bits); free(confidences); free(energies);
    return crc_ok ? 0 : 2;
}
