#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <errno.h>
#include <assert.h>

// === STANDARD PACKET RADIO PARAMETERS ===
#define SAMPLE_RATE     48000
#define BAUD_RATE       300       // 3x throughput; conv FEC margin + passband verified
#define SAMPLES_PER_SYM (SAMPLE_RATE / BAUD_RATE)  // 160 samples/symbol

#define TONE_MARK       1200.0f  // Mark (binary 1) - Bell 202 Lower Tone
#define TONE_SPACE      2400.0f  // Space (binary 0) - Bell 202 Upper Tone

#define PREAMBLE_BITS   128
#define SYNC_BITS       16
#define SYNC_WORD       0xD3D3
#define MAX_PAYLOAD     2048

// === CONVOLUTIONAL FEC (replaces Hamming(7,4) + depth-32 interleaver) ===
// K=7, rate 1/2, the NASA-standard generator pair (171, 133 octal). Free
// distance 10; with the receiver's soft-decision Viterbi this is worth ~5 dB
// of coding gain versus ~1.5 dB for hard-decision Hamming -- at HALF the
// airtime of Hamming + '-r 2' time diversity. Encoder is a 6-bit shift
// register; each input bit emits two coded bits. The register starts at zero
// and is flushed with CONV_FLUSH zero bits so the trellis terminates in the
// all-zero state, which lets the Viterbi decoder anchor its traceback.
// MUST match fsk_demod.c exactly.
#define CONV_G1     0x79    // 171 octal = 1111001b, taps on [u(t) u(t-1) u(t-2) u(t-3) u(t-6)]
#define CONV_G2     0x5B    // 133 octal = 1011011b, taps on [u(t) u(t-2) u(t-3) u(t-5) u(t-6)]
#define CONV_FLUSH  6       // K-1 zero bits to terminate the trellis

// === FRAME HEADER ===
// The full-frame interleaver's geometry depends on the payload length, so the
// receiver must learn the length BEFORE it can de-interleave anything. The
// length therefore moves out of the interleaved body into a small stand-alone
// coded block right after the sync word:
//   [len_hi, len_lo, CRC-8(len)] -> 24 info bits + 6 flush -> 60 coded bits.
// It is convolutionally coded (same code) but NOT interleaved -- at 600 ms it
// is short enough that a burst which kills it would likely have killed the
// sync/preamble anyway. The CRC-8 rejects a corrupted-but-plausible length
// before it can scramble the de-interleave geometry.
#define HDR_INFO_BITS   24
#define HDR_STEPS       (HDR_INFO_BITS + CONV_FLUSH)   // 30 trellis steps
#define HDR_CODED_BITS  (HDR_STEPS * 2)                // 60 on-air bits

// === COMPILE-TIME WORST-CASE SIZING ===
// The chain, all derived from MAX_PAYLOAD:
//   text (<= MAX_PAYLOAD chars)
//     -> 7-bit pack: ceil(n*7/8) bytes
//     -> body byte stream: packed + CRC16(raw text)   (length lives in header)
//     -> trellis steps: 8 bits/byte + CONV_FLUSH
//     -> 2 coded bits per step
//     -> square interleaver pads to R*C, R = floor(sqrt(N)), C = ceil(N/R)
#define PACKED_MAX          ((MAX_PAYLOAD * 7 + 7) / 8)
#define BODY_BYTES_MAX      (PACKED_MAX + 2)
#define BODY_STEPS_MAX      ((size_t)BODY_BYTES_MAX * 8 + CONV_FLUSH)
#define BODY_CODED_MAX      (BODY_STEPS_MAX * 2)
#define IL_ROWS_MAX         170     // floor(sqrt(BODY_CODED_MAX)) = 169 at current params
#define BODY_AIR_MAX        (BODY_CODED_MAX + IL_ROWS_MAX)  // R*C < N + R
#define FRAME_BITS_MAX      (PREAMBLE_BITS + SYNC_BITS + HDR_CODED_BITS + BODY_AIR_MAX)

// Protocol invariants enforced at compile time. If any of these trips, the
// on-air format assumptions are broken and the build should fail loudly.
_Static_assert(SAMPLE_RATE % BAUD_RATE == 0,
               "SAMPLES_PER_SYM must be an integer: SAMPLE_RATE % BAUD_RATE != 0");
_Static_assert(SAMPLES_PER_SYM >= 10,
               "raised-edge shaping window (5+5 samples) exceeds the symbol");
_Static_assert(MAX_PAYLOAD <= 0xFFFF,
               "payload length must fit the 2-byte length header");
_Static_assert((size_t)IL_ROWS_MAX * IL_ROWS_MAX >= BODY_CODED_MAX,
               "IL_ROWS_MAX no longer covers sqrt(BODY_CODED_MAX); raise it");

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// === STATIC WORKING STORAGE (BSS, no heap) ===
// Sizes at current parameters: payload 2 KB, body ~1.8 KB, coded_nat ~28 KB,
// frame_bits ~29 KB, sym_pcm ~1 KB. Total well under 64 KB, fixed at link time.
static uint8_t payload_buffer[MAX_PAYLOAD];
static uint8_t body_bytes[BODY_BYTES_MAX];
static uint8_t coded_nat[BODY_CODED_MAX];   // coded body bits in NATURAL order
static uint8_t frame_bits[FRAME_BITS_MAX];
static int16_t sym_pcm[SAMPLES_PER_SYM];    // one symbol of PCM for streaming writes

// Function prototypes
static uint16_t crc16(const uint8_t *data, size_t len);
static uint8_t crc8(const uint8_t *data, size_t len);
static size_t pack7(const uint8_t *in, size_t n, uint8_t *out);
static size_t conv_encode(const uint8_t *bytes, size_t n_bytes, uint8_t *out_bits);
static size_t isqrt_floor(size_t n);
static size_t build_frame(size_t text_len, size_t body_len);
static int stream_wav(const char *path, size_t total_bits, size_t repeats,
                      size_t gap_syms, uint64_t total_samples);

// Pack n 7-bit characters MSB-first into a dense bitstream. ASCII text never uses
// the 8th bit, so 8 chars collapse into 7 bytes (12.5% off the payload) with no
// alphabet restriction and no model -- fully reversible. The top bit of each input
// byte is dropped, so this is text-only; binary payloads would lose their MSBs.
// Returns the number of packed bytes written: ceil(n*7/8).
static size_t pack7(const uint8_t *in, size_t n, uint8_t *out) {
    size_t outbytes = (n * 7 + 7) / 8;
    memset(out, 0, outbytes);
    size_t bit = 0;
    for (size_t i = 0; i < n; i++) {
        uint8_t c = in[i] & 0x7F;
        for (int b = 6; b >= 0; b--) {
            if (c & (1 << b)) out[bit >> 3] |= (uint8_t)(0x80 >> (bit & 7));
            bit++;
        }
    }
    return outbytes;
}

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
// fsk_demod.c.
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

// One trellis step of the K=7 rate-1/2 encoder. The 6-bit state holds the six
// previous input bits, newest at bit 5. The tap window is the current input
// above the state: w = [u(t) u(t-1) ... u(t-6)], and each generator is the
// parity of its tapped bits. MUST match the transition model in fsk_demod.c's
// Viterbi bit for bit.
static inline void conv_step(int state, int u, int *c1, int *c2) {
    int w = (u << 6) | state;
    *c1 = __builtin_parity(w & CONV_G1);
    *c2 = __builtin_parity(w & CONV_G2);
}

// Convolutionally encode n_bytes (MSB-first per byte) plus CONV_FLUSH zero
// flush bits. Emits 2 coded bits per step into out_bits (one bit per byte),
// c1 first. Returns the number of coded bits written: (n_bytes*8+CONV_FLUSH)*2.
static size_t conv_encode(const uint8_t *bytes, size_t n_bytes, uint8_t *out_bits) {
    int state = 0;
    size_t out = 0;
    size_t n_info = n_bytes * 8;
    for (size_t i = 0; i < n_info + CONV_FLUSH; i++) {
        int u = 0;
        if (i < n_info) u = (bytes[i >> 3] >> (7 - (i & 7))) & 1;
        int c1, c2;
        conv_step(state, u, &c1, &c2);
        out_bits[out++] = (uint8_t)c1;
        out_bits[out++] = (uint8_t)c2;
        state = ((u << 5) | (state >> 1)) & 0x3F;
    }
    return out;
}

// Integer floor(sqrt(n)) for the interleaver geometry. Both ends compute this
// from the same coded length, so both derive the identical R x C grid.
static size_t isqrt_floor(size_t n) {
    size_t r = 1;
    while ((r + 1) * (r + 1) <= n) r++;
    return r;
}

// Build the on-air bit frame into the static frame_bits[] buffer.
// Returns the number of bits written, or 0 if a guard rail tripped.
//
// Frame layout:
//   [128-bit preamble] [16-bit sync] [60-bit coded header] [R*C interleaved body]
//
// The body is the coded [packed7(text) + CRC16(text)] stream spread by a
// FULL-FRAME square block interleaver. Why square: for an R x C grid, adjacent
// on-air bits sit R apart in the coded stream (spacing -- how isolated each
// burst-induced error looks to Viterbi), and a contiguous burst up to C on-air
// bits long never lands two hits in the same coded neighborhood (span -- how
// long a fade can be fully spread). R = floor(sqrt(N)) maximizes the smaller
// of the two, and both grow with frame length: a 250-char frame spreads
// ~600 ms bursts with 59-bit error spacing, versus the old fixed 320 ms /
// 32-bit depth-32 design. Latency is irrelevant for one-shot store-and-
// forward, so there is no reason the interleaver span should be less than the
// whole frame.
//
// Geometry: coded bit n lives at (row = n % R, col = n / R); air position is
// row-major, p = row*C + col. Grid slots past the coded length (n >= N) are
// zero padding. MUST match fsk_demod.c.
static size_t build_frame(size_t text_len, size_t body_len) {
    if (body_len > BODY_BYTES_MAX) {
        fprintf(stderr, "Error: internal sizing fault (body %zu > %d)\n",
                body_len, BODY_BYTES_MAX);
        return 0;
    }

    // Coded body in natural order.
    size_t n_coded = conv_encode(body_bytes, body_len, coded_nat);
    if (n_coded > BODY_CODED_MAX) {
        fprintf(stderr, "Error: internal sizing fault (coded %zu > %zu)\n",
                n_coded, (size_t)BODY_CODED_MAX);
        return 0;
    }

    // Square interleaver geometry, derived from n_coded on both ends.
    size_t R = isqrt_floor(n_coded);
    size_t C = (n_coded + R - 1) / R;
    size_t air_bits = R * C;

    size_t total_bits = PREAMBLE_BITS + SYNC_BITS + HDR_CODED_BITS + air_bits;
    if (R > IL_ROWS_MAX || total_bits > FRAME_BITS_MAX) {
        fprintf(stderr, "Error: internal sizing fault (frame %zu bits > %zu max)\n",
                total_bits, (size_t)FRAME_BITS_MAX);
        return 0;
    }

    size_t bit_idx = 0;

    // 1. Preamble: raw un-coded alternating bits, NOT interleaved, so the receiver
    //    clock can settle on a known pattern.
    for (int i = 0; i < PREAMBLE_BITS; i++) {
        frame_bits[bit_idx++] = (i % 2) ? 1 : 0;
    }

    // 2. Sync word: raw un-coded 0xD3D3, NOT interleaved, so the byte-alignment
    //    tracker catches it cleanly in the on-air bit order.
    for (int i = 15; i >= 0; i--) {
        frame_bits[bit_idx++] = (SYNC_WORD >> i) & 1;
    }

    // 3. Coded length header: [len_hi, len_lo, CRC-8] through the same
    //    convolutional code, terminated, straight onto the air (no interleave).
    {
        uint8_t hdr[3];
        hdr[0] = (uint8_t)((text_len >> 8) & 0xFF);
        hdr[1] = (uint8_t)(text_len & 0xFF);
        hdr[2] = crc8(hdr, 2);
        uint8_t hdr_coded[HDR_CODED_BITS];
        size_t n = conv_encode(hdr, 3, hdr_coded);
        if (n != HDR_CODED_BITS) {
            fprintf(stderr, "Error: internal header coding fault (%zu != %d)\n",
                    n, HDR_CODED_BITS);
            return 0;
        }
        for (size_t i = 0; i < n; i++) frame_bits[bit_idx++] = hdr_coded[i];
    }

    // 4. Body: emit the R x C grid row-major. Slot (row, col) carries coded bit
    //    n = col*R + row, or 0 if that slot is padding.
    for (size_t row = 0; row < R; row++) {
        for (size_t col = 0; col < C; col++) {
            size_t n = col * R + row;
            frame_bits[bit_idx++] = (n < n_coded) ? coded_nat[n] : 0;
        }
    }

    // Guard rail: bit accounting must balance exactly.
    if (bit_idx != total_bits) {
        fprintf(stderr, "Error: internal frame accounting fault (%zu != %zu)\n",
                bit_idx, total_bits);
        return 0;
    }

    return total_bits;
}

// Synthesize the FSK waveform and stream it straight into the WAV file, one
// symbol (480 samples) at a time through the static sym_pcm[] buffer. The
// sample count is fully deterministic from the bit count, so the header is
// written first with exact sizes and the PCM follows.
//
// gap_syms symbols of SILENCE are inserted between repeats (never after the
// last). The gap is quantized to WHOLE SYMBOLS by the caller, on purpose: the
// receiver demodulates the entire capture on one continuous symbol grid, and
// an integer-symbol gap means every copy's bits land ON that grid -- the
// early/late tracker free-runs on discriminator noise during the silence but
// never has to swallow a fractional-symbol phase step at the copy boundary.
// (The field capture that proved this out: a 3.000 s Python-side gap at 100
// baud happened to be exactly 300 symbols, and copy 2's sync was found 2 bits
// off the prediction -- pure tracker walk, zero grid slip. This makes that
// accident a guarantee at any baud.) The oscillator's phase accumulator is
// left untouched across the gap; the per-symbol raised-edge shaping tapers
// the resume, same as every other symbol boundary.
//
// Returns 1 on success, 0 on failure. On failure the partial output file is
// removed so a truncated WAV can never be mistaken for a good one.
static int stream_wav(const char *path, size_t total_bits, size_t repeats,
                      size_t gap_syms, uint64_t total_samples) {
    FILE *f = fopen(path, "wb");
    if (!f) {
        fprintf(stderr, "Error: Could not open output file %s: %s\n",
                path, strerror(errno));
        return 0;
    }

    uint32_t subchunk2_size = (uint32_t)(total_samples * sizeof(int16_t));
    uint32_t chunk_size = 36 + subchunk2_size;
    uint32_t sample_rate = SAMPLE_RATE;
    uint32_t byte_rate = SAMPLE_RATE * sizeof(int16_t);
    uint16_t audio_format = 1, num_channels = 1;
    uint16_t block_align = sizeof(int16_t), bits_per_sample = 16;
    uint32_t fmt_size = 16;

    int ok = 1;
    ok = ok && fwrite("RIFF", 1, 4, f) == 4;
    ok = ok && fwrite(&chunk_size, 4, 1, f) == 1;
    ok = ok && fwrite("WAVE", 1, 4, f) == 4;
    ok = ok && fwrite("fmt ", 1, 4, f) == 4;
    ok = ok && fwrite(&fmt_size, 4, 1, f) == 1;
    ok = ok && fwrite(&audio_format, 2, 1, f) == 1;
    ok = ok && fwrite(&num_channels, 2, 1, f) == 1;
    ok = ok && fwrite(&sample_rate, 4, 1, f) == 1;
    ok = ok && fwrite(&byte_rate, 4, 1, f) == 1;
    ok = ok && fwrite(&block_align, 2, 1, f) == 1;
    ok = ok && fwrite(&bits_per_sample, 2, 1, f) == 1;
    ok = ok && fwrite("data", 1, 4, f) == 4;
    ok = ok && fwrite(&subchunk2_size, 4, 1, f) == 1;

    if (!ok) {
        fprintf(stderr, "Error: Failed writing WAV header to %s: %s\n",
                path, strerror(errno));
        fclose(f);
        remove(path);
        return 0;
    }

    static const float shaping[5] = {0.1f, 0.5f, 1.0f, 0.5f, 0.1f};
    static const int16_t silence[SAMPLES_PER_SYM] = {0};   // one symbol of gap
    double phase = 0.0;   // continuous phase across every symbol and repeat

    for (size_t rep = 0; rep < repeats; rep++) {
        // Inter-copy gap BEFORE every repeat but the first: gap_syms whole
        // symbols of silence, streamed one symbol at a time like everything
        // else. Keyed FM carries full carrier through this regardless.
        if (rep > 0) {
            for (size_t gs = 0; gs < gap_syms; gs++) {
                if (fwrite(silence, sizeof(int16_t), SAMPLES_PER_SYM, f) != SAMPLES_PER_SYM) {
                    fprintf(stderr, "Error: Failed writing gap PCM to %s: %s\n",
                            path, strerror(errno));
                    fclose(f);
                    remove(path);
                    return 0;
                }
            }
        }

        for (size_t bit = 0; bit < total_bits; bit++) {
            float freq = frame_bits[bit] ? TONE_MARK : TONE_SPACE;

            for (int s = 0; s < SAMPLES_PER_SYM; s++) {
                float amplitude = 1.0f;
                if (s < 5) amplitude = shaping[s];
                else if (s >= SAMPLES_PER_SYM - 5) amplitude = shaping[SAMPLES_PER_SYM - 1 - s];

                // Continuous phase tracking integration
                float v = amplitude * sinf((float)phase);
                if (v > 1.0f) v = 1.0f;
                else if (v < -1.0f) v = -1.0f;
                sym_pcm[s] = (int16_t)(v * 32767.0f);

                phase += 2.0 * M_PI * (double)freq / (double)SAMPLE_RATE;
                if (phase >= 2.0 * M_PI) phase -= 2.0 * M_PI;
            }

            if (fwrite(sym_pcm, sizeof(int16_t), SAMPLES_PER_SYM, f) != SAMPLES_PER_SYM) {
                fprintf(stderr, "Error: Failed writing PCM data to %s: %s\n",
                        path, strerror(errno));
                fclose(f);
                remove(path);
                return 0;
            }
        }
    }

    // fclose can surface deferred write errors (e.g. disk full) -- check it.
    if (fclose(f) != 0) {
        fprintf(stderr, "Error: Failed finalizing %s: %s\n", path, strerror(errno));
        remove(path);
        return 0;
    }

    return 1;
}

int main(int argc, char **argv) {
    const char *msg_str = NULL;
    const char *file_path = NULL;
    const char *out_wav_path = "fsk_encoded.wav";
    size_t payload_len = 0;
    int verbose = 0;
    long num_repeats = 1;
    double gap_sec = 0.0;   // inter-copy silence, quantized to whole symbols

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-m") == 0 && i + 1 < argc) {
            msg_str = argv[++i];
        } else if (strcmp(argv[i], "-f") == 0 && i + 1 < argc) {
            file_path = argv[++i];
        } else if (strcmp(argv[i], "-o") == 0 && i + 1 < argc) {
            out_wav_path = argv[++i];
        } else if (strcmp(argv[i], "-r") == 0 && i + 1 < argc) {
            char *end = NULL;
            errno = 0;
            num_repeats = strtol(argv[++i], &end, 10);
            if (errno != 0 || end == argv[i] || *end != '\0' || num_repeats < 1) {
                fprintf(stderr, "Error: -r expects a positive integer (got '%s')\n", argv[i]);
                return 1;
            }
        } else if (strcmp(argv[i], "-g") == 0 && i + 1 < argc) {
            char *end = NULL;
            errno = 0;
            gap_sec = strtod(argv[++i], &end);
            if (errno != 0 || end == argv[i] || *end != '\0'
                || gap_sec < 0.0 || gap_sec > 60.0) {
                fprintf(stderr, "Error: -g expects seconds in [0, 60] (got '%s')\n", argv[i]);
                return 1;
            }
        } else if (strcmp(argv[i], "-v") == 0) {
            verbose = 1;
        } else {
            fprintf(stderr, "Error: Unknown or incomplete option '%s'\n", argv[i]);
            fprintf(stderr, "Usage: %s [-m <message> | -f <file>] -o <output.wav> [-r repeats] [-g gap_sec] [-v]\n", argv[0]);
            return 1;
        }
    }

    if (msg_str != NULL) {
        payload_len = strlen(msg_str);
        if (payload_len == 0) {
            fprintf(stderr, "Error: Message is empty\n");
            return 1;
        }
        if (payload_len > MAX_PAYLOAD) {
            fprintf(stderr, "Error: Message too long (%zu bytes, max %d bytes)\n",
                    payload_len, MAX_PAYLOAD);
            return 1;
        }
        memcpy(payload_buffer, msg_str, payload_len);
        if (verbose) fprintf(stderr, "[*] Message: %zu bytes\n", payload_len);
    } else if (file_path != NULL) {
        FILE *f_in = fopen(file_path, "rb");
        if (!f_in) {
            fprintf(stderr, "Error: Could not open input file %s: %s\n",
                    file_path, strerror(errno));
            return 1;
        }

        if (fseek(f_in, 0, SEEK_END) != 0) {
            fprintf(stderr, "Error: Could not seek in %s: %s\n", file_path, strerror(errno));
            fclose(f_in);
            return 1;
        }
        long file_size = ftell(f_in);
        if (file_size < 0) {
            fprintf(stderr, "Error: Could not size %s: %s\n", file_path, strerror(errno));
            fclose(f_in);
            return 1;
        }
        rewind(f_in);

        if (verbose) fprintf(stderr, "[*] File size: %ld bytes\n", file_size);

        if (file_size == 0) {
            fprintf(stderr, "Error: File is empty\n");
            fclose(f_in);
            return 1;
        }
        if (file_size > MAX_PAYLOAD) {
            fprintf(stderr, "Error: File too large (%ld bytes, max %d bytes)\n",
                    file_size, MAX_PAYLOAD);
            fclose(f_in);
            return 1;
        }

        payload_len = fread(payload_buffer, 1, MAX_PAYLOAD, f_in);
        if (payload_len != (size_t)file_size || ferror(f_in)) {
            fprintf(stderr, "Error: Failed to read %s (%zu of %ld bytes): %s\n",
                    file_path, payload_len, file_size, strerror(errno));
            fclose(f_in);
            return 1;
        }
        fclose(f_in);
    } else {
        fprintf(stderr, "Usage: %s [-m <message> | -f <file>] -o <output.wav> [-r repeats] [-g gap_sec] [-v]\n", argv[0]);
        return 1;
    }

    if (verbose) {
        int high = 0;
        for (size_t i = 0; i < payload_len; i++) if (payload_buffer[i] & 0x80) high++;
        size_t packed = (payload_len * 7 + 7) / 8;
        fprintf(stderr, "[*] 7-bit pack: %zu chars -> %zu bytes (saved %zu, %.0f%%)\n",
                payload_len, packed, payload_len - packed,
                payload_len ? 100.0 * (payload_len - packed) / payload_len : 0.0);
        if (high) fprintf(stderr, "[!] %d byte(s) had the high bit set; 7-bit packing dropped it (text-only)\n", high);
    }

    // Mask the payload to 7 bits BEFORE the CRC is computed. The channel only
    // carries 7 bits per character, so the CRC must cover the text as
    // transmitted, not the raw input -- otherwise any high-bit byte (e.g. the
    // 0xC2 0xB0 of a UTF-8 degree symbol) guarantees a CRC mismatch at the
    // receiver even over a perfect channel, because the receiver can only ever
    // reconstruct the masked text. After this point TX and RX hash identical
    // bytes; the mangling of non-ASCII input is visible (and warned about
    // above) rather than disguised as link failure.
    for (size_t i = 0; i < payload_len; i++) payload_buffer[i] &= 0x7F;

    // Assemble the coded body byte stream: 7-bit-packed(text) + CRC16(text).
    // The length no longer rides in the body -- it lives in the stand-alone
    // coded header (see build_frame) so the receiver can derive the full-frame
    // interleaver geometry before de-interleaving. The CRC is computed over the
    // RAW text, so a pass guarantees the unpacked message is exact.
    size_t packed_cap = (payload_len * 7 + 7) / 8;
    size_t body_len   = packed_cap + 2;
    if (body_len > BODY_BYTES_MAX) {
        fprintf(stderr, "Error: internal sizing fault (body %zu > %d)\n",
                body_len, BODY_BYTES_MAX);
        return 1;
    }

    size_t packed_len = pack7(payload_buffer, payload_len, body_bytes);
    uint16_t crc = crc16(payload_buffer, payload_len);
    body_bytes[packed_len]     = (crc >> 8) & 0xFF;
    body_bytes[packed_len + 1] = crc & 0xFF;

    size_t total_bits = build_frame(payload_len, body_len);
    if (total_bits == 0) {
        fprintf(stderr, "Error: Frame construction failed\n");
        return 1;
    }

    // Quantize the inter-copy gap to WHOLE SYMBOLS so every repeat lands on
    // the receiver's continuous symbol grid (see stream_wav). Gap only exists
    // between copies, so -r 1 has none by construction.
    size_t gap_syms = (num_repeats > 1) ? (size_t)llround(gap_sec * BAUD_RATE) : 0;
    if (verbose && num_repeats == 1 && gap_sec > 0.0)
        fprintf(stderr, "[*] Note: -g %.3f ignored with -r 1 (no copy boundaries)\n", gap_sec);

    // Guard rail: total sample count and WAV byte sizes must fit their uint32
    // header fields. At 100 baud this only matters for absurd -r values, but an
    // absurd -r value is exactly when it matters.
    uint64_t gap_samples_total = (uint64_t)gap_syms * SAMPLES_PER_SYM
                                 * (uint64_t)(num_repeats - 1);
    uint64_t total_samples = (uint64_t)total_bits * SAMPLES_PER_SYM * (uint64_t)num_repeats
                             + gap_samples_total;
    uint64_t data_bytes = total_samples * sizeof(int16_t);
    if (data_bytes + 36 > 0xFFFFFFFFull) {
        fprintf(stderr, "Error: Output exceeds WAV 4 GB limit "
                "(%llu samples with -r %ld); reduce repeats or gap\n",
                (unsigned long long)total_samples, num_repeats);
        return 1;
    }

    if (!stream_wav(out_wav_path, total_bits, (size_t)num_repeats, gap_syms, total_samples)) {
        fprintf(stderr, "Error: Modulation/write failed\n");
        return 1;
    }

    if (verbose) {
        size_t n_coded = (body_len * 8 + CONV_FLUSH) * 2;
        size_t R = isqrt_floor(n_coded);
        size_t C = (n_coded + R - 1) / R;
        printf("[*] Wrote %llu samples to %s (%.2f seconds)\n",
               (unsigned long long)total_samples, out_wav_path,
               (double)total_samples / SAMPLE_RATE);
        printf("[*] Time diversity: %ld cop%s, inter-copy gap %.3f s -> %zu symbols (grid-aligned)\n",
               num_repeats, num_repeats == 1 ? "y" : "ies",
               (double)gap_syms / BAUD_RATE, gap_syms);
        printf("[*] FEC Mode: 7-bit pack + conv K=7 r=1/2 (171,133) + full-frame interleaver (%zux%zu)\n",
               R, C);
        printf("[*] Burst spread: %.1f ms across the frame, error spacing %zu coded bits\n",
               (double)C * 1000.0 / BAUD_RATE, R);
        printf("[*] Memory: all-static working set (~%zu KB BSS), zero heap allocations\n",
               (sizeof(payload_buffer) + sizeof(body_bytes) + sizeof(coded_nat)
                + sizeof(frame_bits) + sizeof(sym_pcm)) / 1024);
    }

    return 0;
}
