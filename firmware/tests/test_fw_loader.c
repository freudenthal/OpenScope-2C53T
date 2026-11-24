/*
 * Host tests for the CDC firmware loader's state machine.
 *
 * The RAM installer cannot run on a host and is compiled out
 * (FW_LOADER_HOST_TEST); what is tested is everything that decides WHETHER
 * it may run — the gates that stand between a byte stream and an erased
 * app slot:
 *
 *   1. Size gate: zero, odd, tiny and over-staging sizes are refused
 *      before a single byte is consumed.
 *   2. The happy path: a valid image streamed in awkward chunk sizes
 *      lands byte-exact in the staging region and reaches STAGED.
 *   3. CRC gate: one flipped bit anywhere = ERROR, never STAGED.
 *   4. Vector gate: right size and CRC but a non-app vector table (SP not
 *      in SRAM / PC out of slot / even PC) = ERROR.
 *   5. Timeout: a transfer that goes silent returns the shell to life as
 *      ERROR, and a fresh fwload recovers.
 *   6. apply() refuses anything not STAGED.
 *   7. No-buffer gate: until the shell task lends a scratch area, begin()
 *      and install_slot() refuse with ERR_NO_BUFFER and consume nothing.
 *   8. Slot check (stage-167): begin() erases a slot only when each byte
 *      its staging will erase is ours (FWC1/FWCP manifest span) or 0xFF;
 *      a refusal names the first foreign byte and changes nothing.
 */

#include "../src/drivers/fw_loader.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* Simulated 2 MB fwcache region (chip 0xD00000..0xF00000). */
uint8_t fw_loader_test_w25q[0x200000];

static int failures = 0;

#define CHECK(cond, ...)                                                      \
    do {                                                                      \
        if (!(cond)) {                                                        \
            printf("FAIL %s:%d: ", __FILE__, __LINE__);                       \
            printf(__VA_ARGS__);                                              \
            printf("\n");                                                     \
            failures++;                                                       \
        }                                                                     \
    } while (0)

/* Same CRC the loader and the host script use. */
static uint32_t crc32_ref(const uint8_t *p, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFu;
    for (uint32_t i = 0; i < len; ++i) {
        crc ^= p[i];
        for (int b = 0; b < 8; ++b)
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return crc ^ 0xFFFFFFFFu;
}

/* A minimal valid image: SRAM-shaped SP, odd (thumb) in-slot PC, noise body. */
static void make_image(uint8_t *img, uint32_t size)
{
    for (uint32_t i = 0; i < size; ++i)
        img[i] = (uint8_t)(i * 2654435761u >> 24);
    uint32_t sp = 0x20030000u, pc = 0x08007311u;
    memcpy(img, &sp, 4);
    memcpy(img + 4, &pc, 4);
}

static void feed_in_chunks(const uint8_t *img, uint32_t size, uint16_t chunk)
{
    for (uint32_t off = 0; off < size; ) {
        uint16_t n = (uint16_t)((size - off) < chunk ? (size - off) : chunk);
        fw_loader_feed(img + off, n);
        off += n;
    }
}

static void test_size_gate(void)
{
    CHECK(!fw_loader_begin(0, 0x1234, 1), "size 0 must be refused");
    CHECK(!fw_loader_begin(8193, 0x1234, 1), "odd size must be refused");
    CHECK(!fw_loader_begin(4096, 0x1234, 1), "size < 8192 must be refused");
    CHECK(!fw_loader_begin(757762, 0x1234, 1), "size > app ceiling must be refused");
    CHECK(!fw_loader_begin(20000, 0x1234, 2), "slot 2 must be refused");
    CHECK(fw_loader_error() == FW_LOADER_ERR_SIZE, "err must be SIZE");
    CHECK(!fw_loader_active(), "a refused begin must not activate RX routing");
}

static void test_happy_path(void)
{
    enum { SZ = 20000 };
    static uint8_t img[SZ];
    make_image(img, SZ);
    uint32_t crc = crc32_ref(img, SZ);

    CHECK(fw_loader_begin(SZ, crc, 1), "begin must accept a sane image");
    CHECK(fw_loader_active(), "must be receiving");
    /* 63 is deliberately not a divisor of anything: pages fill unevenly. */
    feed_in_chunks(img, SZ, 63);
    CHECK(fw_loader_state() == FW_LOADER_STAGED,
          "valid stream must stage (state=%d err=%d)",
          fw_loader_state(), fw_loader_error());
    CHECK(!fw_loader_active(), "routing must return to the shell when done");
    CHECK(memcmp(&fw_loader_test_w25q[0x00E01000u - 0x00D00000u], img, SZ) == 0,
          "staged bytes must be byte-exact in slot B");
    CHECK(fw_loader_slot_size(1) == SZ, "slot B manifest must carry the size");
    CHECK(fw_loader_slot_crc(1) == crc, "slot B manifest must carry the crc");
    CHECK(fw_loader_slot_size(0) == 0, "slot A must stay empty");
    CHECK(fw_loader_apply(), "apply must accept a STAGED image (host no-op)");
    CHECK(fw_loader_install_slot(1), "fwswap of a valid cached slot must pass");
    CHECK(!fw_loader_install_slot(0), "fwswap of an empty slot must refuse");
}

static void test_crc_gate(void)
{
    enum { SZ = 16384 };
    static uint8_t img[SZ];
    make_image(img, SZ);
    uint32_t crc = crc32_ref(img, SZ);
    img[9000] ^= 0x40;   /* corrupt AFTER computing the announced CRC */

    CHECK(fw_loader_begin(SZ, crc, 0), "begin ok");
    feed_in_chunks(img, SZ, 512);
    CHECK(fw_loader_state() == FW_LOADER_ERROR &&
          fw_loader_error() == FW_LOADER_ERR_CRC,
          "one flipped bit must land in ERROR/CRC (state=%d err=%d)",
          fw_loader_state(), fw_loader_error());
    CHECK(!fw_loader_apply(), "apply must refuse a CRC-failed stage");
    CHECK(fw_loader_slot_size(0) == 0,
          "a CRC-failed intake must leave slot A without a manifest");
}

static void test_vector_gate(void)
{
    enum { SZ = 16384 };
    static uint8_t img[SZ];

    /* SP not in SRAM */
    make_image(img, SZ);
    uint32_t bad_sp = 0x08007000u;
    memcpy(img, &bad_sp, 4);
    CHECK(fw_loader_begin(SZ, crc32_ref(img, SZ), 1), "begin ok");
    feed_in_chunks(img, SZ, 2048);
    CHECK(fw_loader_error() == FW_LOADER_ERR_VECTOR, "flash-shaped SP must fail");

    /* PC below the app slot */
    make_image(img, SZ);
    uint32_t bad_pc = 0x08000101u;
    memcpy(img + 4, &bad_pc, 4);
    CHECK(fw_loader_begin(SZ, crc32_ref(img, SZ), 1), "begin ok");
    feed_in_chunks(img, SZ, 2048);
    CHECK(fw_loader_error() == FW_LOADER_ERR_VECTOR, "bootloader PC must fail");

    /* Even (non-thumb) PC */
    make_image(img, SZ);
    uint32_t even_pc = 0x08007310u;
    memcpy(img + 4, &even_pc, 4);
    CHECK(fw_loader_begin(SZ, crc32_ref(img, SZ), 1), "begin ok");
    feed_in_chunks(img, SZ, 2048);
    CHECK(fw_loader_error() == FW_LOADER_ERR_VECTOR, "even PC must fail");
}

static void test_timeout_recovers(void)
{
    enum { SZ = 16384 };
    static uint8_t img[SZ];
    make_image(img, SZ);
    uint32_t crc = crc32_ref(img, SZ);

    CHECK(fw_loader_begin(SZ, crc, 1), "begin ok");
    fw_loader_feed(img, 1000);          /* partial, then silence */
    for (int i = 0; i < 400; ++i)
        fw_loader_poll();
    CHECK(fw_loader_state() == FW_LOADER_ERROR &&
          fw_loader_error() == FW_LOADER_ERR_TIMEOUT,
          "silence must time out (state=%d err=%d)",
          fw_loader_state(), fw_loader_error());
    CHECK(!fw_loader_active(), "timeout must return routing to the shell");

    /* And a full retry succeeds from that state. */
    CHECK(fw_loader_begin(SZ, crc, 1), "retry begin ok");
    feed_in_chunks(img, SZ, 4096);
    CHECK(fw_loader_state() == FW_LOADER_STAGED, "retry must stage");
}

/* The loader owns no buffer; on target the shell task lends its bus scratch.
 * Before that has happened nothing may start — and a lend that is too small
 * must count as no lend at all. */
static void test_no_buffer_gate(void)
{
    static uint8_t tiny[FW_LOADER_SCRATCH_MIN - 1];

    CHECK(!fw_loader_begin(20000, 0x1234, 1), "begin without scratch must refuse");
    CHECK(fw_loader_error() == FW_LOADER_ERR_NO_BUFFER, "err must be NO_BUFFER");
    CHECK(!fw_loader_active(), "a refused begin must not activate RX routing");
    CHECK(!fw_loader_install_slot(0), "install without scratch must refuse");
    CHECK(fw_loader_error() == FW_LOADER_ERR_NO_BUFFER, "err must be NO_BUFFER");

    fw_loader_attach_scratch(tiny, sizeof(tiny));
    CHECK(!fw_loader_begin(20000, 0x1234, 1), "undersized scratch must count as none");
    CHECK(fw_loader_error() == FW_LOADER_ERR_NO_BUFFER, "err must be NO_BUFFER");
}

/* Install breadcrumbs (EXP-57 follow-up): the decoder the boot capture and
 * `fwstat` rely on. The negative control matters most: backup registers
 * power up with arbitrary content on some parts, so words WITHOUT the marker
 * must never read as a record. */
static void test_breadcrumb_decode(void)
{
    uint16_t none[5] = { 0x0000, 0x0004, 0x0808, 0x1000, 0x0014 };
    fwl_breadcrumb_t b = fw_loader_breadcrumb_decode(none);
    CHECK(!b.present, "words without the marker must not decode as a record");

    uint16_t junk[5] = { 0xF1A4, 0x0004, 0x0808, 0x1000, 0x0014 };
    b = fw_loader_breadcrumb_decode(junk);
    CHECK(!b.present, "a near-miss marker (0xF1A4) must not decode as a record");

    uint16_t erase[5] = { FWL_BC_MAGIC, FWL_BC_ERASE_ERR, 0x0808, 0x1000, 0x0014 };
    b = fw_loader_breadcrumb_decode(erase);
    CHECK(b.present && b.code == FWL_BC_ERASE_ERR && !b.spi2_stall,
          "erase record: present=%d code=%u stall=%d", b.present, b.code, b.spi2_stall);
    CHECK(b.addr == 0x08081000u, "page address 0x%08X, want 0x08081000", (unsigned)b.addr);
    CHECK(b.sts == 0x0014u, "sts 0x%04X, want 0x0014", b.sts);

    uint16_t stall[5] = { FWL_BC_MAGIC, FWL_BC_DONE | FWL_BC_SPI2_STALL, 0x0809, 0x6800, 0 };
    b = fw_loader_breadcrumb_decode(stall);
    CHECK(b.present && b.code == FWL_BC_DONE && b.spi2_stall,
          "the stall flag rides beside the code: code=%u stall=%d", b.code, b.spi2_stall);

    CHECK(fw_loader_breadcrumb_decode(NULL).present == false, "NULL words decode as no record");

    /* Every exit the installer can take has its own name; no two share one. */
    static const uint16_t codes[] = { FWL_BC_BAD_SIZE, FWL_BC_UNLOCK, FWL_BC_ERASE_BUSY,
                                      FWL_BC_ERASE_ERR, FWL_BC_PROG_BUSY, FWL_BC_PROG_ERR,
                                      FWL_BC_VERIFY, FWL_BC_STARTED, FWL_BC_DONE };
    for (unsigned i = 0; i < sizeof codes / sizeof codes[0]; i++) {
        const char *ni = fw_loader_breadcrumb_name(codes[i]);
        CHECK(strcmp(ni, "unknown") != 0, "code %u has no name", codes[i]);
        for (unsigned j = i + 1; j < sizeof codes / sizeof codes[0]; j++)
            CHECK(strcmp(ni, fw_loader_breadcrumb_name(codes[j])) != 0,
                  "codes %u and %u share the name '%s'", codes[i], codes[j], ni);
    }
    CHECK(strcmp(fw_loader_breadcrumb_name(0x55), "unknown") == 0, "an unlisted code reads 'unknown'");
    /* The blink count is the code itself, so every failure exit must be a
     * small positive number a person can count. */
    for (unsigned i = 0; i < 7; i++)
        CHECK(codes[i] >= 1 && codes[i] <= 7, "failure exit %u blinks %u times", i, codes[i]);
}

/* ── Slot check (stage-167) ────────────────────────────────────────────
 * The slots lie inside the stock FAT volume "2:". begin() may erase a slot
 * only when every byte its staging will erase is ours or 0xFF. */
#define SLOT_A_IDX 0x000000u
#define SLOT_B_IDX 0x100000u
#define SLOT_SPAN  0x100000u

static uint8_t slot_copy[SLOT_SPAN];

static void blank_slot(uint32_t idx)
{
    memset(&fw_loader_test_w25q[idx], 0xFF, SLOT_SPAN);
}

/* begin() must refuse, name `want_off`, erase nothing, arm nothing. */
static void expect_refused(uint32_t idx, uint8_t slot, uint32_t size, uint32_t want_off,
                           const char *what)
{
    memcpy(slot_copy, &fw_loader_test_w25q[idx], SLOT_SPAN);
    CHECK(!fw_loader_begin(size, 0x1234u, slot), "%s: begin must refuse", what);
    CHECK(fw_loader_error() == FW_LOADER_ERR_SLOT_NOT_BLANK,
          "%s: err must be SLOT_NOT_BLANK (err=%d)", what, fw_loader_error());
    CHECK(fw_loader_slot_dirty_offset() == want_off,
          "%s: dirty offset 0x%X, want 0x%X", what,
          (unsigned)fw_loader_slot_dirty_offset(), (unsigned)want_off);
    CHECK(memcmp(slot_copy, &fw_loader_test_w25q[idx], SLOT_SPAN) == 0,
          "%s: a refused slot must be byte-for-byte unchanged", what);
    CHECK(!fw_loader_active(), "%s: a refused begin must not activate RX routing", what);
}

static void stage(uint8_t slot, const uint8_t *img, uint32_t size, const char *what)
{
    CHECK(fw_loader_begin(size, crc32_ref(img, size), slot), "%s: begin must accept (err=%d)",
          what, fw_loader_error());
    feed_in_chunks(img, size, 777);
    CHECK(fw_loader_state() == FW_LOADER_STAGED, "%s: must stage (state=%d err=%d)", what,
          fw_loader_state(), fw_loader_error());
}

static void test_slot_check(void)
{
    enum { SZ = 20000, BIG = 40000 };
    static uint8_t img[BIG];
    /* Staging SZ erases the manifest sector + data up to this sector end. */
    const uint32_t erase_end = (4096u + SZ + 4095u) & ~4095u;   /* 0x6000 */
    make_image(img, BIG);

    /* 1. Foreign data in the data span, no manifest. */
    blank_slot(SLOT_A_IDX);
    fw_loader_test_w25q[SLOT_A_IDX + 0x2345u] = 0x00;
    expect_refused(SLOT_A_IDX, 0, SZ, 0x2345u, "foreign byte in data span");

    /* 2. Foreign data in the manifest sector (a FAT sector, say). */
    blank_slot(SLOT_A_IDX);
    memcpy(&fw_loader_test_w25q[SLOT_A_IDX], "\xEB\x3C\x90MSDOS5.0", 11);
    expect_refused(SLOT_A_IDX, 0, SZ, 0u, "FAT boot sector at slot base");

    /* 3. The last byte the staging would erase counts; the next one does not. */
    blank_slot(SLOT_A_IDX);
    fw_loader_test_w25q[SLOT_A_IDX + erase_end - 1u] = 0x7E;
    expect_refused(SLOT_A_IDX, 0, SZ, erase_end - 1u, "byte in the last erased sector");
    blank_slot(SLOT_A_IDX);
    fw_loader_test_w25q[SLOT_A_IDX + erase_end] = 0x7E;
    stage(0, img, SZ, "byte past the last erased sector");
    CHECK(fw_loader_test_w25q[SLOT_A_IDX + erase_end] == 0x7E,
          "a byte outside the erase span must survive the staging");

    /* 4. A blank slot is accepted. fwload alone erases nothing (the
     *    stage-310 silence check relies on it); the first data chunk marks
     *    the slot ours before any data erase. */
    blank_slot(SLOT_B_IDX);
    CHECK(fw_loader_begin(SZ, crc32_ref(img, SZ), 1), "blank slot must be accepted");
    for (uint32_t i = 0; i < SLOT_SPAN; ++i) {
        if (fw_loader_test_w25q[SLOT_B_IDX + i] != 0xFF) {
            CHECK(0, "begin with no data must change nothing (offset 0x%X)", (unsigned)i);
            break;
        }
    }
    fw_loader_feed(img, 600);           /* one 512 B chunk commits */
    CHECK(memcmp(&fw_loader_test_w25q[SLOT_B_IDX], "FWCP", 4) == 0,
          "the first chunk must write the FWCP marker");
    CHECK(memcmp(&fw_loader_test_w25q[SLOT_B_IDX + 0x1000u], img, 512) == 0,
          "the first chunk must land after the manifest sector");
    CHECK(fw_loader_slot_size(1) == 0, "FWCP must never read as an installable image");

    /* 5. A torn staging of ours (FWCP + partial data) is accepted again. */
    fw_loader_feed(img + 600, 2400);
    for (int i = 0; i < 400; ++i)
        fw_loader_poll();
    CHECK(fw_loader_error() == FW_LOADER_ERR_TIMEOUT, "partial transfer must time out");
    CHECK(!fw_loader_install_slot(1), "fwswap of a torn FWCP slot must refuse");
    stage(1, img, SZ, "re-stage over a torn staging of ours");

    /* 6. A slot with a valid FWC1 manifest is accepted (same size again). */
    stage(1, img, SZ, "re-stage over FWC1");
    CHECK(fw_loader_slot_size(1) == SZ, "re-stage must leave FWC1 with the size");

    /* 7. A bigger image over FWC1: bytes past the old erase span must be 0xFF. */
    fw_loader_test_w25q[SLOT_B_IDX + erase_end + 0x10u] = 0x55;
    expect_refused(SLOT_B_IDX, 1, BIG, erase_end + 0x10u, "foreign byte past an old FWC1 image");
    fw_loader_test_w25q[SLOT_B_IDX + erase_end + 0x10u] = 0xFF;
    stage(1, img, BIG, "bigger image over FWC1, blank tail");

    /* 8. A FWCP marker with an impossible size is not ours. */
    blank_slot(SLOT_A_IDX);
    memcpy(&fw_loader_test_w25q[SLOT_A_IDX], "FWCP\x01\x00\x00\x00", 8);
    expect_refused(SLOT_A_IDX, 0, SZ, 0u, "FWCP with size 1");

    blank_slot(SLOT_A_IDX);
    blank_slot(SLOT_B_IDX);
}

int main(void)
{
    /* Same shape as the target: one 2 KB arena lent by the shell task. */
    static uint8_t scratch[2048];

    /* An erased W25Q reads 0xFF (stage-167: begin() now checks the slot). */
    memset(fw_loader_test_w25q, 0xFF, sizeof(fw_loader_test_w25q));

    test_no_buffer_gate();
    fw_loader_attach_scratch(scratch, sizeof(scratch));

    test_size_gate();
    test_slot_check();
    test_happy_path();
    test_crc_gate();
    test_vector_gate();
    test_timeout_recovers();
    test_breadcrumb_decode();

    if (failures) {
        printf("%d FAILURE(S)\n", failures);
        return 1;
    }
    printf("all fw_loader tests passed\n");
    return 0;
}
