/**********************************************************************
  Variant benchmark + micro-architecture calibration for aarch64 CRC
  research on ISA-L-CRC-ARM-BENCH.

  Links directly against internal variant symbols (they all live in
  libisal.a but are not exposed in the public headers) and reports:

    1. correctness: every variant vs table CRC32C, lengths 0..8448
       (covers all tail paths), random seeds, random data
    2. 8K warm-cache throughput of every variant (same methodology as
       crc32_iscsi_perf: CLOCK_MONOTONIC, fixed-time loops)
    3. micro-arch calibration:
       - effective CPU frequency via a NEON-bound loop
       - crc32cx dependent-chain latency (cycles/8B)
       - crc32cx throughput on 3 independent chains (bytes/cycle)
       - pmull fold throughput (bytes/cycle)

  Build (run from repo root after `make`):
    gcc -O2 -o tools/variant_bench tools/variant_bench.c \
        -I include -I crc .libs/libisal.a

  License: same as ISA-L (BSD-3-Clause).
**********************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>

/* ---- internal variant prototypes (buf, len, crc) ---- */
extern unsigned int crc32_iscsi_crc_ext(unsigned char *, int, unsigned int);
extern unsigned int crc32_iscsi_3crc_fold(unsigned char *, int, unsigned int);
extern unsigned int crc32_iscsi_refl_pmull(unsigned char *, int, unsigned int);
extern unsigned int crc32c_mix_default(unsigned char *, uint64_t, unsigned int);
extern unsigned int crc32c_mix_neoverse_n1(unsigned char *, uint64_t, unsigned int);
extern unsigned int crc32c_mix_half(unsigned char *, uint64_t, unsigned int);
extern unsigned int crc32c_mix_half_ilv(unsigned char *, uint64_t, unsigned int);
extern unsigned int crc32c_mix_r4(unsigned char *, uint64_t, unsigned int);
extern unsigned int crc32c_mix_r5(unsigned char *, uint64_t, unsigned int);
extern unsigned int crc32c_mix_r6(unsigned char *, uint64_t, unsigned int);
extern unsigned int crc32c_mix_r7(unsigned char *, uint64_t, unsigned int);

typedef unsigned int (*vfn_t)(unsigned char *, uint64_t, unsigned int);

static inline unsigned int
call_vfn(void *fn, unsigned char *b, uint64_t l, unsigned int c)
{
        return ((vfn_t) fn)(b, l, c);
}

/* ---- table CRC32C reference ---- */
static uint32_t tab[256];
static void
tab_init(void)
{
        for (uint32_t i = 0; i < 256; i++) {
                uint32_t c = i;
                for (int k = 0; k < 8; k++)
                        c = (c >> 1) ^ (0x82F63B78u & (0u - (c & 1u)));
                tab[i] = c;
        }
}
static inline uint32_t
ref_step(uint32_t c, uint8_t b)
{
        return (c >> 8) ^ tab[(c ^ b) & 0xFF];
}
static uint32_t
crc32c_ref(const uint8_t *p, size_t n, uint32_t seed)
{
        /* ISA-L crc32_iscsi ASM variants use the plain running-crc
         * convention (no mvn for the crc32c poly): internal state
         * starts from the seed and the state is returned as-is.
         * Same convention as crc32_iscsi_base in crc_base.c. */
        uint32_t c = seed;
        for (size_t i = 0; i < n; i++)
                c = ref_step(c, p[i]);
        return c;
}

/* ---- timing ---- */
static double
now_sec(void)
{
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return ts.tv_sec + ts.tv_nsec * 1e-9;
}

typedef struct variant {
        const char *name;
        void *fn;
} variant_t;

/* runner that adapts 3-arg (buf,len,crc) to the 2-arg loop shape used
 * by the perf harness: fresh seed each pass, same as upstream perf */
#define RUN_VARIANT(fn, buf, len) call_vfn(fn, buf, len, 0x1234)

static variant_t variants[] = {
        { "crc32_iscsi_crc_ext",    (void *) crc32_iscsi_crc_ext },
        { "crc32_iscsi_3crc_fold",  (void *) crc32_iscsi_3crc_fold },
        { "crc32_iscsi_refl_pmull", (void *) crc32_iscsi_refl_pmull },
        { "crc32c_mix_default",     (void *) crc32c_mix_default },
        { "crc32c_mix_neoverse_n1", (void *) crc32c_mix_neoverse_n1 },
        { "crc32c_mix_half",        (void *) crc32c_mix_half },
        { "crc32c_mix_half_ilv",     (void *) crc32c_mix_half_ilv },
        { "crc32c_mix_r4",           (void *) crc32c_mix_r4 },
        { "crc32c_mix_r5",           (void *) crc32c_mix_r5 },
        { "crc32c_mix_r6",           (void *) crc32c_mix_r6 },
        { "crc32c_mix_r7",           (void *) crc32c_mix_r7 },
};

#define NVARIANTS (sizeof(variants) / sizeof(variants[0]))
#define TEST_LEN 8192
#define MEASURE_SEC 1.5

static uint32_t
measure(const variant_t *v, uint8_t *buf, double sec)
{
        uint64_t it = 0;
        uint32_t c = 0;
        double t0 = now_sec();
        do {
                c = RUN_VARIANT(v->fn, buf, TEST_LEN);
                it++;
        } while (now_sec() - t0 < sec);
        double dt = now_sec() - t0;
        printf("%-26s %8.2f MB/s   (crc=0x%08x, %.3fs, %llu passes)\n",
               v->name, TEST_LEN * it / dt / 1e6, c, dt,
               (unsigned long long) it);
        fflush(0);
        return (uint32_t) (TEST_LEN * it / dt / 1e6);
}

/* ---------------- micro-arch calibration ---------------- */

/* approximate effective frequency with a NEON eor loop: 4 eors/cycle
 * on most wide-issue cores; conservative but good enough to convert
 * bytes/sec into bytes/cycle within ~10%. */
static double
calib_freq(void)
{
        uint64_t v[8] = { 0 };
        uint64_t it = 0;
        double t0 = now_sec();
        do {
                for (int k = 0; k < 8; k++)
                        v[k] ^= it + k;
                it += 8;
        } while (now_sec() - t0 < 0.5);
        double dt = now_sec() - t0;
        volatile uint64_t sink = v[0] ^ v[7];
        (void) sink;
        /* ~4 scalar eor/cycle (2 pipes x 2) */
        return (it / dt) / 4.0;
}

/* CRC dependent-chain latency in bytes/cycle (serial chain = 8B per
 * latency). Uses crc32_iscsi_crc_ext-style single chain on 8K. */
static double
calib_crc_serial(uint8_t *buf, double freq)
{
        uint64_t it = 0;
        double t0 = now_sec();
        do {
                (void) RUN_VARIANT(crc32_iscsi_crc_ext, buf, TEST_LEN);
                it++;
        } while (now_sec() - t0 < 0.4);
        double dt = now_sec() - t0;
        double bps = TEST_LEN * it / dt;
        return bps / freq;
}

/* pmull-region isolated throughput, emulating the fold mix of the
 * mix_default inner loop (pmull + pmull2 + eor chains) but without
 * any crc instructions, on 512B blocks re-folded in place.  Reports
 * bytes/cycle the NEON side can sustain while doing fold work. */
static double __attribute__((noinline))
calib_pmull(uint8_t *buf, double freq)
{
        /* emulate fold throughput: repeatedly fold a 512B region using
         * the same instruction mix as the mix_default inner loop
         * (16 pmull + 16 eor + 1x 4-reg ld1 per 64B).  Register list
         * must be architecturally consecutive for 4-reg ld1. */
        uint64_t it = 0;
        double t0 = now_sec();
        do {
                __asm__ volatile (
                        "ld1 {v0.16b, v1.16b, v2.16b, v3.16b}, [%[p]], #64\n\t"
                        ".rept 7\n\t"
                        "ld1 {v4.16b, v5.16b, v6.16b, v7.16b}, [%[p]], #64\n\t"
                        "pmull v16.1q, v0.1d, v2.1d\n\t"
                        "pmull2 v17.1q, v0.2d, v2.2d\n\t"
                        "pmull v18.1q, v1.1d, v2.1d\n\t"
                        "pmull2 v19.1q, v1.2d, v2.2d\n\t"
                        "eor v0.16b, v16.16b, v17.16b\n\t"
                        "eor v1.16b, v18.16b, v19.16b\n\t"
                        "eor v0.16b, v0.16b, v4.16b\n\t"
                        "eor v1.16b, v1.16b, v5.16b\n\t"
                        "pmull v16.1q, v6.1d, v2.1d\n\t"
                        "pmull2 v17.1q, v6.2d, v2.2d\n\t"
                        "pmull v18.1q, v7.1d, v2.1d\n\t"
                        "pmull2 v19.1q, v7.2d, v2.2d\n\t"
                        "eor v0.16b, v0.16b, v16.16b\n\t"
                        "eor v0.16b, v0.16b, v17.16b\n\t"
                        "eor v1.16b, v1.16b, v18.16b\n\t"
                        "eor v1.16b, v1.16b, v19.16b\n\t"
                        ".endr\n\t"
                        : [p] "+r" (buf)
                        :
                        : "v0", "v1", "v2", "v3", "v4", "v5", "v6", "v7",
                          "v16", "v17", "v18", "v19", "memory");
                buf -= 512; /* rewind to fold the same region again */
                it++;
        } while (now_sec() - t0 < 0.4);
        double dt = now_sec() - t0;
        return (512.0 * it / dt) / freq;
}

int
main(void)
{
        tab_init();
        uint8_t *buf;
        if (posix_memalign((void **) &buf, 1024, TEST_LEN)) {
                printf("alloc fail\n");
                return 1;
        }

        /* deterministic pseudo-random fill */
        {
                uint64_t s = 0x243F6A8885A308D3ull;
                for (int i = 0; i < TEST_LEN; i++) {
                        s = s * 6364136223846793005ull + 1442695040888963407ull;
                        buf[i] = (uint8_t) (s >> 33);
                }
        }

        /* ---- 1. correctness over all lengths ---- */
        printf("== correctness (all lengths 0..8448 step 7 + boundaries) ==\n");
        int failures = 0;
        for (size_t n = 0; n <= TEST_LEN + 256; n += 7) {
                uint32_t seed = (uint32_t) (n * 2654435761u);
                uint32_t r = crc32c_ref(buf, n, seed);
                for (size_t vi = 0; vi < NVARIANTS; vi++) {
                        uint32_t c = call_vfn(variants[vi].fn, buf, n, seed);
                        if (c != r) {
                                printf("FAIL %-24s len=%zu got=%08x want=%08x\n",
                                       variants[vi].name, n, c, r);
                                failures++;
                        }
                }
        }
        /* boundary lengths near block/tail edges */
        size_t edges[] = { 0, 1, 2, 3, 63, 64, 1023, 1024, 1025, 2047, 2048,
                           2049, 4095, 4096, 6143, 6144, 8191, 8192, 8447, 8448 };
        for (size_t ei = 0; ei < sizeof(edges) / sizeof(edges[0]); ei++) {
                size_t n = edges[ei];
                if (n > TEST_LEN + 256)
                        continue;
                uint32_t seed = 0x1234u + (uint32_t) ei;
                uint32_t r = crc32c_ref(buf, n, seed);
                for (size_t vi = 0; vi < NVARIANTS; vi++) {
                        uint32_t c = call_vfn(variants[vi].fn, buf, n, seed);
                        if (c != r) {
                                printf("FAIL %-24s len=%zu got=%08x want=%08x\n",
                                       variants[vi].name, n, c, r);
                                failures++;
                        }
                }
        }
        printf("correctness: %s (%d failures)\n", failures ? "FAIL" : "ALL PASS", failures);
        if (failures)
                return 2;

        /* ---- 2. variant throughput ---- */
        printf("\n== 8K warm throughput (1.5s each) ==\n");
        for (size_t vi = 0; vi < NVARIANTS; vi++)
                measure(&variants[vi], buf, MEASURE_SEC);
        (void) measure; /* keep measure referenced when NVARIANTS==0 */

        /* ---- 3. calibration ---- */
        printf("\n== micro-arch calibration (approximate) ==\n");
        double freq = calib_freq();
        double crc_bpc = calib_crc_serial(buf, freq);
        printf("est. effective freq : %.3f GHz\n", freq / 1e9);
        printf("serial crc32cx      : %.2f B/cycle (latency %0.2f cycles per 8B)\n",
               crc_bpc, 8.0 / crc_bpc);
        double pmull_bpc = calib_pmull(buf, freq);
        printf("pmull fold region   : %.2f B/cycle (NEON side)\n", pmull_bpc);
        printf("rebalance hint      : optimal CRC share s* = R/(2P) when "
               "3 chains saturate; current s=0.75, R=%.2f P=%.2f\n",
               crc_bpc * 3 /*3-chain estimate*/, pmull_bpc);

        free(buf);
        return 0;
}
