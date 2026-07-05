/* fsk_rxd.c  --  squelch-gated RX capture daemon ("radio mailbox")
 * Hardened Edition: Static memory lifecycle management and strict bounds tracking.
 * Multi-threaded Async I/O Edition.
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
#define DEFAULT_FREQ_HZ        146430000u   
#define DEFAULT_SAMP_RATE      1200000u     
#define DEFAULT_GAIN_TENTHS    402          
#define DEFAULT_PPM            0            
#define DEFAULT_DEV_INDEX      0            
#define DEFAULT_OPEN_DB        10.0         
#define DEFAULT_MIN_CAP_SEC    5.0  
#define DEFAULT_DEEMPH_US      75.0         
#define DEFAULT_DSP_PATH       "./fsk_dsp"  
#define DEFAULT_OUTDIR         "."          
#define IQ_SCRATCH_NAME        "iq_capture.bin"  

#define HYSTERESIS_DB          4.0          
#define FLOOR_ALPHA            0.98         
#define PREROLL_MS             500.0        
#define HANG_MS                400.0        
#define MAX_CAPTURE_SEC        120.0        
#define DSP_PRESLEEP_SEC       3            
#define READ_BLOCK_BYTES       (1u << 16)   /* 65536 B */
#define FLUSH_BLOCKS           4            
#define PLL_TOL_HZ             1000         
#define PLL_RETRIES            5

#define READ_FAIL_SLEEP_US     200000   
#define READ_FAILS_REINIT      10       
#define REINIT_ATTEMPTS        5        
#define REINIT_BACKOFF_SEC     3        
#define ENABLE_OFFSET_TUNING   0

#define SQ_DC_BIAS 127.4  

// === HARDENED STATIC STORAGE CAPACITY CEILINGS ===
#define MAX_PATH_LEN           512
#define MAX_RING_BUFFER_BYTES  1600000u 
#define IO_RING_BYTES          (16u * 1024u * 1024u) // 16 MB Async Disk Buffer

static uint8_t s_read_buf[READ_BLOCK_BYTES];
static uint8_t s_ring_buffer[MAX_RING_BUFFER_BYTES];

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

// =============================================================================
// ASYNCHRONOUS I/O SUBSYSTEM (Producer-Consumer)
// =============================================================================
static uint8_t s_io_ring[IO_RING_BYTES];
static size_t io_head = 0;
static size_t io_tail = 0;
static FILE *io_file = NULL;
static int io_close_requested = 0;
static int io_quit = 0;
static pthread_mutex_t io_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t io_cond = PTHREAD_COND_INITIALIZER;
static pthread_cond_t io_empty_cond = PTHREAD_COND_INITIALIZER;

static void *io_thread_func(void *arg) {
    (void)arg;
    while (1) {
        pthread_mutex_lock(&io_mutex);
        while (io_tail == io_head && !io_close_requested && !io_quit) {
            pthread_cond_wait(&io_cond, &io_mutex);
        }
        
        if (io_quit && io_tail == io_head) {
            pthread_mutex_unlock(&io_mutex);
            break;
        }
        
        size_t head = io_head;
        size_t tail = io_tail;
        FILE *f = io_file;
        pthread_mutex_unlock(&io_mutex);
        
        if (head != tail && f != NULL) {
            size_t to_write = (head > tail) ? (head - tail) : (IO_RING_BYTES - tail);
            size_t written = fwrite(s_io_ring + tail, 1, to_write, f);
            if (written > 0) {
                pthread_mutex_lock(&io_mutex);
                io_tail = (io_tail + written) % IO_RING_BYTES;
                pthread_cond_broadcast(&io_empty_cond);
                pthread_mutex_unlock(&io_mutex);
            } else {
                pthread_mutex_lock(&io_mutex);
                io_tail = (io_tail + to_write) % IO_RING_BYTES;
                pthread_cond_broadcast(&io_empty_cond);
                pthread_mutex_unlock(&io_mutex);
                logmsg("ERR async disk write failed: %s", strerror(errno));
            }
        }
        
        pthread_mutex_lock(&io_mutex);
        if (io_close_requested && io_tail == io_head && io_file != NULL) {
            fclose(io_file);
            io_file = NULL;
            io_close_requested = 0; 
            pthread_cond_broadcast(&io_empty_cond);
        }
        pthread_mutex_unlock(&io_mutex);
    }
    return NULL;
}

static void async_io_init(void) {
    pthread_t tid;
    pthread_create(&tid, NULL, io_thread_func, NULL);
    pthread_detach(tid); 
}

static int async_io_open(const char *path) {
    pthread_mutex_lock(&io_mutex);
    io_file = fopen(path, "wb");
    int ok = (io_file != NULL);
    io_head = 0;
    io_tail = 0;
    io_close_requested = 0;
    pthread_mutex_unlock(&io_mutex);
    return ok;
}

static void async_io_write(const uint8_t *buf, size_t len) {
    pthread_mutex_lock(&io_mutex);
    while (1) {
        size_t space = (io_tail > io_head) ? (io_tail - io_head - 1) : (IO_RING_BYTES - io_head + io_tail - 1);
        if (space >= len) break;
        pthread_cond_wait(&io_empty_cond, &io_mutex); // Block hot loop if disk stalls entirely
    }
    
    size_t first = IO_RING_BYTES - io_head;
    if (first > len) first = len;
    memcpy(s_io_ring + io_head, buf, first);
    if (len > first) memcpy(s_io_ring, buf + first, len - first);
    
    io_head = (io_head + len) % IO_RING_BYTES;
    pthread_cond_signal(&io_cond);
    pthread_mutex_unlock(&io_mutex);
}

static void async_io_close(void) {
    pthread_mutex_lock(&io_mutex);
    if (io_file) {
        io_close_requested = 1;
        pthread_cond_signal(&io_cond);
        while (io_close_requested) {
            pthread_cond_wait(&io_empty_cond, &io_mutex);
        }
    }
    pthread_mutex_unlock(&io_mutex);
}

static void async_io_stop(void) {
    pthread_mutex_lock(&io_mutex);
    io_quit = 1;
    pthread_cond_signal(&io_cond);
    pthread_mutex_unlock(&io_mutex);
}
// =============================================================================

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
#if ENABLE_OFFSET_TUNING
    rtlsdr_set_offset_tuning(dev, 1);
#endif
    if (gain_tenths != g)
        logmsg("gain snapped %.1f -> %.1f dB", gain_tenths/10.0, g/10.0);

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
        rtlsdr_read_sync(dev, s_read_buf, READ_BLOCK_BYTES, &nr);
    }
    return 0;
}

static double block_power_db(const uint8_t *buf, int nbytes) {
    long nsamp = nbytes / 2;
    if (nsamp <= 0) return -120.0;
    double acc = 0.0;
    for (long k = 0; k < nsamp; k++) {
        double i = (double)buf[2*k]     - SQ_DC_BIAS;
        double q = (double)buf[2*k + 1] - SQ_DC_BIAS;
        acc += i*i + q*q;
    }
    return 10.0 * log10(acc / (double)nsamp + 1e-9);
}

static int run_dsp(const char *dsp_path, const char *iqpath, const char *wavpath,
                   uint32_t rate, double deemph_us, int verbose) {
    char ratestr[24], deemphstr[24];
    snprintf(ratestr,   sizeof ratestr,   "%u",   (unsigned)rate);
    snprintf(deemphstr, sizeof deemphstr, "%.0f", deemph_us);

    char *argv[8]; 
    argv[0] = (char *)dsp_path;
    argv[1] = (char *)iqpath;
    argv[2] = (char *)wavpath;
    argv[3] = "-r"; 
    argv[4] = ratestr;
    argv[5] = "-e"; 
    argv[6] = deemphstr;
    argv[7] = NULL;

    if (verbose) {
        logmsg("DSP launched in diagnostic verbose mode flag state");
    }

    pid_t pid = fork();
    if (pid < 0) { logmsg("ERR fork: %s", strerror(errno)); return -1; }

    if (pid == 0) {                       
        if (dup2(STDERR_FILENO, STDOUT_FILENO) < 0) {
            _exit(126);
        }
        execv(dsp_path, argv);
        fprintf(stderr, "exec %s failed: %s\n", dsp_path, strerror(errno));
        _exit(127);
    }

    int status = 0;                       
    while (waitpid(pid, &status, 0) < 0) {
        if (errno != EINTR) { logmsg("ERR waitpid: %s", strerror(errno)); return -1; }
    }
    if (WIFEXITED(status))   return WEXITSTATUS(status);
    if (WIFSIGNALED(status)) { logmsg("DSP killed by signal %d", WTERMSIG(status)); return -1; }
    return -1;
}

static void finalize_capture(const char *iqpath, const char *dsp_path,
                             const char *outdir, const char *stamp, long cap_bytes,
                             double peak_db, uint32_t rate, double min_cap_sec,
                             double deemph_us, int verbose, const char *reason) {
    
    async_io_close(); // Drains the ring buffer completely and closes the file safely

    double secs = cap_bytes / 2.0 / rate;

    if (secs < min_cap_sec) {
        logmsg("DISCARD (%s) %.2fs < %.0fs min  peak=%.1f dB  (no DSP)",
               reason, secs, min_cap_sec, peak_db);
        remove(iqpath);
        return;
    }

    char wavpath[MAX_PATH_LEN * 2];
    int path_check = snprintf(wavpath, sizeof wavpath, "%s/fsk_%s.wav", outdir, stamp);
    if (path_check < 0 || (size_t)path_check >= sizeof(wavpath)) {
        logmsg("ERR wavpath string formatting limits breached. Discarding output block.");
        remove(iqpath);
        return;
    }

    logmsg("CAPTURE keep (%s)  dur=%.2fs  peak=%.1f dB  -> %s",
           reason, secs, peak_db, wavpath);

    logmsg("settling %d s, then launching %s", DSP_PRESLEEP_SEC, dsp_path);
    sleep(DSP_PRESLEEP_SEC);

    int rc = run_dsp(dsp_path, iqpath, wavpath, rate, deemph_us, verbose);
    if (rc == 0) {
        remove(iqpath);
        logmsg("DSP ok  -> %s  (removed %s)", wavpath, iqpath);
    } else {
        logmsg("DSP FAILED rc=%d  keeping %s for reprocessing", rc, iqpath);
    }
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

static void usage(const char *p) {
    fprintf(stderr,
      "usage: %s [-f freq_hz] [-p ppm] [-s samp_rate] [-g gain_tenths] [-d dev_index]\n"
      "          [-q open_dB] [-m min_sec] [-e deemph_us] [-x dsp_path] [-o outdir] [-v]\n"
      "  defaults: -f %u -p %d -s %u -g %d -d %d -q %.0f -m %.0f -e %.0f -x %s -o %s\n",
      p, DEFAULT_FREQ_HZ, DEFAULT_PPM, DEFAULT_SAMP_RATE, DEFAULT_GAIN_TENTHS,
      DEFAULT_DEV_INDEX, DEFAULT_OPEN_DB, DEFAULT_MIN_CAP_SEC, DEFAULT_DEEMPH_US,
      DEFAULT_DSP_PATH, DEFAULT_OUTDIR);
}

int main(int argc, char **argv) {
    uint32_t freq = DEFAULT_FREQ_HZ, rate = DEFAULT_SAMP_RATE;
    int gain = DEFAULT_GAIN_TENTHS, ppm = DEFAULT_PPM, dev_index = DEFAULT_DEV_INDEX;
    double open_db = DEFAULT_OPEN_DB, min_cap = DEFAULT_MIN_CAP_SEC;
    double deemph_us = DEFAULT_DEEMPH_US;
    int verbose = 0;
    const char *outdir = DEFAULT_OUTDIR;
    const char *dsp_path = DEFAULT_DSP_PATH;

    int c;
    while ((c = getopt(argc, argv, "f:p:s:g:d:q:m:e:x:o:vh")) != -1) {
        switch (c) {
            case 'f': freq = (uint32_t)strtoul(optarg, NULL, 10); break;
            case 'p': ppm  = atoi(optarg); break;
            case 's': rate = (uint32_t)strtoul(optarg, NULL, 10); break;
            case 'g': gain = atoi(optarg); break;
            case 'd': dev_index = atoi(optarg); break;
            case 'q': open_db = atof(optarg); break;
            case 'm': min_cap = atof(optarg); break;
            case 'e': deemph_us = atof(optarg); break;
            case 'x': dsp_path = optarg; break;
            case 'o': outdir = optarg; break;
            case 'v': verbose = 1; break;
            case 'h': default: usage(argv[0]); return (c == 'h') ? 0 : 2;
        }
    }
    
    if (strlen(outdir) >= MAX_PATH_LEN || strlen(dsp_path) >= MAX_PATH_LEN) {
        fprintf(stderr, "Error: Provided paths exceed maximum safe limits of %d bytes.\n", MAX_PATH_LEN);
        return 1;
    }

    double close_db = open_db - HYSTERESIS_DB;

    const long   samp_per_block = READ_BLOCK_BYTES / 2;
    const double block_ms       = samp_per_block * 1000.0 / (double)rate;
    const int    hang_blocks    = (int)ceil(HANG_MS / block_ms);
    const long   max_cap_bytes  = (long)(MAX_CAPTURE_SEC * rate) * 2;
    
    size_t rb_size = (size_t)(PREROLL_MS / 1000.0 * rate) * 2;
    if (rb_size < READ_BLOCK_BYTES) rb_size = READ_BLOCK_BYTES;
    
    if (rb_size > MAX_RING_BUFFER_BYTES) {
        fprintf(stderr, "Error: Configured sample rate requires a ring buffer (%zu bytes) larger than safe static stack allocation boundaries (%u bytes).\n", 
                rb_size, MAX_RING_BUFFER_BYTES);
        return 1;
    }

    char iqpath[MAX_PATH_LEN * 2];
    if (snprintf(iqpath, sizeof iqpath, "%s/%s", outdir, IQ_SCRATCH_NAME) >= (int)sizeof(iqpath)) {
        fprintf(stderr, "Error: Configured scratch path output target string footprint overflowed storage boundaries.\n");
        return 1;
    }

    signal(SIGINT,  on_signal);
    signal(SIGTERM, on_signal);

    async_io_init(); 

    rtlsdr_dev_t *dev = NULL;
    if (rtlsdr_open(&dev, (uint32_t)dev_index) < 0) {
        logmsg("ERR cannot open RTL-SDR device %d", dev_index);
        return 1;
    }
    if (configure_dongle(dev, freq, rate, gain, ppm) < 0) {
        rtlsdr_close(dev); return 1;
    }

    size_t rb_head = 0, rb_count = 0;

    logmsg("fsk_rxd up: f=%u Hz  s=%u sps  g=%.1f dB  ppm=%d  dev=%d",
           freq, rate, snap_gain(dev, gain)/10.0, ppm, dev_index);
    logmsg("squelch: open=+%.1f close=+%.1f dB  block=%.1f ms  hang=%d blk (%.0f ms)  "
           "preroll=%.0f ms  min=%.0f s  maxcap=%.0f s",
           open_db, close_db, block_ms, hang_blocks, hang_blocks*block_ms,
           PREROLL_MS, min_cap, MAX_CAPTURE_SEC);

    int capturing = 0, floor_init = 0, hang = 0, read_fails = 0;
    double floor_db = -120.0, peak_db = -120.0;
    long cap_bytes = 0;
    char stamp[32];

    while (!g_stop) {
        int nread = 0;
        int r = rtlsdr_read_sync(dev, s_read_buf, READ_BLOCK_BYTES, &nread);
        if (r < 0 || nread <= 0) {
            if (g_stop) break;
            read_fails++;
            logmsg("WARN read_sync r=%d n=%d (consecutive fail %d/%d)",
                   r, nread, read_fails, READ_FAILS_REINIT);
            if (read_fails < READ_FAILS_REINIT) {
                usleep(READ_FAIL_SLEEP_US);   
                continue;
            }

            if (capturing) {
                finalize_capture(iqpath, dsp_path, outdir, stamp, cap_bytes,
                                 peak_db, rate, min_cap, deemph_us, verbose, "device-fail");
                capturing = 0; hang = 0;
            }

            if (reinit_device(&dev, dev_index, freq, rate, gain, ppm) < 0) {
                logmsg("FATAL device unrecoverable after %d attempts; exiting "
                       "for supervisor restart", REINIT_ATTEMPTS);
                return 1;
            }

            read_fails = 0;
            floor_init = 0;
            rb_head = 0; rb_count = 0;
            continue;
        }
        read_fails = 0;
        double db = block_power_db(s_read_buf, nread);

        if (!capturing) {
            size_t n = (size_t)nread;
            if (n >= rb_size) {
                memcpy(s_ring_buffer, s_read_buf + (n - rb_size), rb_size);
                rb_head = 0; rb_count = rb_size;
            } else {
                size_t first = rb_size - rb_head;
                if (first > n) first = n;
                memcpy(s_ring_buffer + rb_head, s_read_buf, first);
                if (n > first) memcpy(s_ring_buffer, s_read_buf + first, n - first);
                rb_head = (rb_head + n) % rb_size;
                rb_count += n; if (rb_count > rb_size) rb_count = rb_size;
            }

            if (!floor_init) { floor_db = db; floor_init = 1; }
            else floor_db = FLOOR_ALPHA*floor_db + (1.0-FLOOR_ALPHA)*db;

            if (db > floor_db + open_db) {
                time_t now = time(NULL);
                struct tm tmv; gmtime_r(&now, &tmv);
                strftime(stamp, sizeof stamp, "%y%m%d.%H%M%S", &tmv);

                if (!async_io_open(iqpath)) {
                    logmsg("ERR cannot open async stream to %s: %s", iqpath, strerror(errno));
                    continue;   
                }
                logmsg("CAPTURE start %s -> %s  power=%.1f dB  floor=%.1f dB  open=+%.1f",
                       stamp, iqpath, db, floor_db, open_db);

                size_t written = 0;
                if (rb_count < rb_size) {
                    async_io_write(s_ring_buffer, rb_count);
                    written = rb_count;
                } else {
                    async_io_write(s_ring_buffer + rb_head, rb_size - rb_head);
                    async_io_write(s_ring_buffer, rb_head);
                    written = rb_size;
                }
                cap_bytes = (long)written;
                peak_db = db;
                capturing = 1; hang = 0;
                logmsg("  pre-roll %zu bytes (%.0f ms)",
                       written, written/2.0*1000.0/rate);
            }
        } else {
            /* --- CAPTURING: write straight through async queue --- */
            async_io_write(s_read_buf, (size_t)nread);
            cap_bytes += nread;
            if (db > peak_db) peak_db = db;

            if (db < floor_db + close_db) {
                if (++hang >= hang_blocks) {
                    finalize_capture(iqpath, dsp_path, outdir, stamp, cap_bytes,
                                     peak_db, rate, min_cap, deemph_us, verbose, "hang");
                    capturing = 0; hang = 0; rb_head = 0; rb_count = 0;
                    continue;
                }
            } else {
                hang = 0;   
            }

            if (cap_bytes >= max_cap_bytes) {
                finalize_capture(iqpath, dsp_path, outdir, stamp, cap_bytes,
                                 peak_db, rate, min_cap, deemph_us, verbose, "maxcap");
                capturing = 0; hang = 0; rb_head = 0; rb_count = 0;
            }
        }
    }

    if (capturing) {
        finalize_capture(iqpath, dsp_path, outdir, stamp, cap_bytes,
                         peak_db, rate, min_cap, deemph_us, verbose, "signal");
    }
    
    logmsg("fsk_rxd shutting down cleanly");
    if (dev) rtlsdr_close(dev);
    
    async_io_stop(); // Unblock and retire the disk thread
    return 0;
}
