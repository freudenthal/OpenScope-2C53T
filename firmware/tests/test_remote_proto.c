/*
 * Remote protocol over the shared CDC stream — host tests (issue #10).
 *
 * Covers what binding esp_comm to USB adds on top of the framing tests in
 * test_esp_comm.c: the §3.2 router (binary frames vs shell text on one
 * stream), the rejection paths that must answer instead of going silent,
 * the inter-byte gap timeout, the explicit STATUS v1 layout, and button
 * injection that only ACKs what was really queued.
 *
 * scripts/test_remote_proto.py builds this against mutated copies of
 * esp_comm.c and requires it to FAIL for each guard removed.
 *
 * Build:  make test-remote-proto
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_comm.h"
#include "acq_ctrl.h"

static int failures = 0, checks = 0;
#define CHECK(cond, msg) do { checks++; if (!(cond)) { failures++; \
    printf("  FAIL: %s  (%s:%d)\n", msg, __FILE__, __LINE__); } } while (0)

/* ─── captured device output ─── */
static uint8_t tx[8192];
static size_t tx_len;
static int block_calls;
static bool block_writer(const uint8_t *d, uint16_t n)
{
    block_calls++;
    if (tx_len + n <= sizeof(tx)) { memcpy(tx + tx_len, d, n); tx_len += n; }
    return true;
}
static uint8_t tx_bytes[8192];
static size_t tx_bytes_len;
static void byte_writer(uint8_t b) { if (tx_bytes_len < sizeof(tx_bytes)) tx_bytes[tx_bytes_len++] = b; }

/* ─── captured shell passthrough ─── */
static char shell[4096];
static size_t shell_len;
static int shell_calls;
static void shell_sink(const uint8_t *d, uint16_t n, void *ctx)
{
    (void)ctx;
    shell_calls++;
    if (shell_len + n < sizeof(shell)) { memcpy(shell + shell_len, d, n); shell_len += n; }
    shell[shell_len] = 0;
}

/* ─── injected device state ─── */
static esp_status_snapshot_t fake_status;
static void status_provider(esp_status_snapshot_t *out) { *out = fake_status; }
static uint8_t injected[16];
static int injected_n;
static bool inject_ok;
static bool button_injector(uint8_t id)
{
    if (!inject_ok) return false;
    injected[injected_n++] = id;
    return true;
}

static esp_meter_snapshot_t fake_meter;
static bool meter_ready;
static bool meter_wrong_mode;
static esp_meter_result_t meter_provider(esp_meter_snapshot_t *out)
{
    if (meter_wrong_mode) return ESP_METER_WRONG_MODE;
    if (!meter_ready) return ESP_METER_NOT_READY;
    *out = fake_meter;
    return ESP_METER_OK;
}

/* Waveform provider: a coherent record per channel, or a refusal. `wave_lie`
 * is a provider that answers OK while handing over a synthetic record — the
 * encoder must refuse it, not label it. */
static esp_wave_snapshot_t fake_wave;
static esp_wave_result_t wave_result;
static bool wave_lie;
static uint8_t wave_mask_seen;
static int wave_calls;
static uint8_t wave_ch1[ESP_WAVE_MAX_SAMPLES + 8];
static uint8_t wave_ch2[ESP_WAVE_MAX_SAMPLES + 8];
static esp_wave_result_t wave_provider(uint8_t mask, esp_wave_snapshot_t *out)
{
    wave_calls++;
    wave_mask_seen = mask;
    /* Filled even when refusing: a refusal must be honoured on its own,
     * not only because an empty snapshot happens to look like no record. */
    *out = fake_wave;
    if (wave_result != ESP_WAVE_OK) return wave_result;
    if (wave_lie) out->synthetic = true;
    return ESP_WAVE_OK;
}

static void wave_defaults(void)
{
    for (int i = 0; i < (int)sizeof(wave_ch1); i++) {
        wave_ch1[i] = (uint8_t)(i * 7 + 3);
        wave_ch2[i] = (uint8_t)(0xAA ^ i);              /* contains 0xAA bytes on purpose */
    }
    /* Both patterns cover every byte value 4 times over 1024 samples, so
     * their XOR is 0 and a checksum that skipped them would still pass. */
    wave_ch1[1] ^= 0x01;
    wave_ch2[1] ^= 0x01;
    memset(&fake_wave, 0, sizeof(fake_wave));
    fake_wave.frame_id = 0x01020304;
    fake_wave.timebase_idx = 0x10;
    fake_wave.timebase_tier = ESP_TIER_MEASURED;
    fake_wave.sample_rate_hz = 12490;
    fake_wave.time_ordered = true;
    fake_wave.counts_per_div = 32;
    fake_wave.head_skip = 128;
    fake_wave.ch[0].samples = wave_ch1;
    fake_wave.ch[0].count = ESP_WAVE_MAX_SAMPLES;
    fake_wave.ch[0].vdiv_idx = 6;
    fake_wave.ch[0].vdiv_tier = ESP_TIER_MEASURED;
    fake_wave.ch[0].uv_per_div = 1264486;
    fake_wave.ch[1].samples = wave_ch2;
    fake_wave.ch[1].count = ESP_WAVE_MAX_SAMPLES;
    fake_wave.ch[1].vdiv_idx = 8;
    fake_wave.ch[1].vdiv_tier = ESP_TIER_PROVISIONAL;
    fake_wave.ch[1].uv_per_div = 6586022;
    wave_result = ESP_WAVE_OK;
    wave_lie = false;
    wave_mask_seen = 0;
    wave_calls = 0;
}

/* Control provider (stage-086 additions only; the M6 sets are covered end to
 * end in tools/openscope_host/tests/test_end_to_end.py). */
static esp_description_t fake_desc;
static esp_scope_state_t fake_state;
static int probe_calls;
static uint8_t probe_ch_seen, probe_factor_seen;
static void ctl_get_state(esp_scope_state_t *o) { *o = fake_state; }
static void ctl_get_description(esp_description_t *o) { *o = fake_desc; }
static esp_ctrl_result_t ctl_set_probe(uint8_t ch, uint8_t f)
{
    probe_calls++; probe_ch_seen = ch; probe_factor_seen = f;
    return ESP_CTRL_OK;
}
static const esp_control_ops_t ctl_ops = {
    .get_state = ctl_get_state,
    .get_description = ctl_get_description,
    .set_probe = ctl_set_probe,
};

static void desc_defaults(void)
{
    memset(&fake_desc, 0, sizeof(fake_desc));
    fake_desc.samples_per_div = 32;
    for (int i = 0; i < ESP_TIMEBASE_CODES; i++) {
        fake_desc.tb_rate_hz[i] = 1000u + (uint32_t)i;      /* every code distinct */
        fake_desc.tb_tier[i] = ESP_TIER_NONE;
    }
    fake_desc.tb_rate_hz[0x10] = 12490; fake_desc.tb_tier[0x10] = ESP_TIER_MEASURED;
    fake_desc.tb_rate_hz[0x0D] = 119678; fake_desc.tb_tier[0x0D] = ESP_TIER_PROVISIONAL;
    fake_desc.tb_rate_hz[0x0F] = 0; fake_desc.tb_tier[0x0F] = ESP_TIER_MEASURED;  /* 0 vouched for */
    fake_desc.counts_per_div = 32;
    for (int c = 0; c < 2; c++)
        for (int r = 0; r < ESP_VDIV_RANGES; r++) {
            fake_desc.uv_per_div_bnc[c][r] = 100000u * (uint32_t)(c + 1) + (uint32_t)r;
            fake_desc.vdiv_tier[c][r] = ESP_TIER_MEASURED;
        }
    fake_desc.vdiv_tier[1][3] = ESP_TIER_NONE;              /* a value its tier disowns */
    fake_desc.trig_level_min = -100; fake_desc.trig_level_max = 100;
    fake_desc.trig_center_code = 128; fake_desc.trig_num = 256; fake_desc.trig_den = 206;
    fake_desc.trig_code_min = 1; fake_desc.trig_code_max = 254;
    fake_desc.trig_record_offset = -28;
    fake_desc.probe[0] = 10; fake_desc.probe[1] = 7;        /* 7: not a probe setting */
    for (int i = 0; i < (int)ESP_DESC_UID_LEN; i++)
        fake_desc.uid[i] = (uint8_t)(0xA0 + i);
    fake_desc.build_target = "guest-coldtrace";
    fake_desc.git_rev = "0a1af35-dirty";
    memset(&fake_state, 0, sizeof(fake_state));
    fake_state.ch[0].probe = 10;
    fake_state.ch[1].probe = 1;
    probe_calls = 0; probe_ch_seen = probe_factor_seen = 0;
}

static void reset(void)
{
    esp_comm_init();
    esp_comm_set_control_provider(0);
    desc_defaults();
    esp_comm_set_writer(0);
    esp_comm_set_block_writer(block_writer);
    esp_comm_set_status_provider(0);
    esp_comm_set_button_injector(0);
    esp_comm_set_meter_provider(0);
    esp_comm_set_waveform_provider(0);
    wave_defaults();
    meter_ready = false;
    meter_wrong_mode = false;
    tx_len = 0; block_calls = 0; tx_bytes_len = 0;
    shell_len = 0; shell[0] = 0; shell_calls = 0;
    injected_n = 0; inject_ok = true;
}

static size_t frame(uint8_t *out, uint8_t cmd, const uint8_t *p, uint16_t n)
{
    out[0] = 0xAA; out[1] = cmd; out[2] = (uint8_t)(n >> 8); out[3] = (uint8_t)n;
    uint8_t c = cmd ^ out[2] ^ out[3];
    for (uint16_t i = 0; i < n; i++) { out[4 + i] = p[i]; c ^= p[i]; }
    out[4 + n] = c;
    return (size_t)n + 5;
}

/* Parse one response frame from tx at *off. Returns cmd or -1. */
static int take(size_t *off, uint8_t *payload, uint16_t *plen)
{
    if (*off + 5 > tx_len || tx[*off] != 0xAA) return -1;
    uint8_t cmd = tx[*off + 1];
    uint16_t n = (uint16_t)((tx[*off + 2] << 8) | tx[*off + 3]);
    if (*off + 5 + n > tx_len) return -1;
    uint8_t c = cmd ^ tx[*off + 2] ^ tx[*off + 3];
    for (uint16_t i = 0; i < n; i++) { c ^= tx[*off + 4 + i]; if (payload) payload[i] = tx[*off + 4 + i]; }
    if (c != tx[*off + 4 + n]) return -1;
    if (plen) *plen = n;
    *off += 5 + n;
    return cmd;
}

static void route(const void *d, size_t n, uint32_t t)
{
    esp_comm_route((const uint8_t *)d, (uint16_t)n, t, shell_sink, 0);
}

/* ─────────────────────────────────────────────────────────────── */

static void test_text_passes_through_untouched(void)
{
    reset();
    route("version\r\n", 9, 0);
    CHECK(strcmp(shell, "version\r\n") == 0, "plain shell text reaches the shell verbatim");
    CHECK(tx_len == 0, "plain text produces no protocol output");
}

static void test_frame_between_text_keeps_order(void)
{
    uint8_t buf[64]; size_t n = 0, off = 0; uint8_t p[64]; uint16_t pl;
    reset();
    memcpy(buf, "ab", 2); n = 2;
    n += frame(buf + n, ESP_CMD_PING, 0, 0);
    memcpy(buf + n, "cd\r", 3); n += 3;
    route(buf, n, 0);
    CHECK(strcmp(shell, "abcd\r") == 0, "text around a frame reaches the shell, frame bytes do not");
    CHECK(shell_calls == 2, "text before and after the frame are separate, ordered runs");
    CHECK(take(&off, p, &pl) == ESP_RSP_DATA, "PING inside a text stream is answered");
}

static void test_frame_split_across_usb_packets(void)
{
    uint8_t buf[16]; size_t n = frame(buf, ESP_CMD_STATUS, 0, 0), off = 0;
    reset();
    /* 1 byte per packet, ESP_RX_GAP_MS/2 apart: the frame spans ~2.5 gaps in
     * total, so this only passes if the timeout is really INTER-byte. */
    for (size_t i = 0; i < n; i++) route(buf + i, 1, (uint32_t)(i * (ESP_RX_GAP_MS / 2)));
    CHECK(take(&off, 0, 0) == ESP_RSP_STATUS, "a slow frame (gaps < timeout, total > timeout) is reassembled");
    CHECK(shell_len == 0, "no frame byte leaked to the shell");
}

static void test_status_v1_layout(void)
{
    uint8_t buf[16], p[64]; uint16_t pl = 0; size_t off = 0;
    reset();
    fake_status.current_mode = 1;          /* MODE_MULTIMETER */
    fake_status.battery_pct = 73;
    fake_status.flags = ESP_STATUS_FLAG_CHARGING | ESP_STATUS_FLAG_CAPTURE_READY;
    fake_status.battery_mv = 3987;
    fake_status.uptime_ms = 0x01020304;
    fake_status.usb_tx_stalls = 0xA0B0C0D0;
    fake_status.usb_heals = 0x1234;
    fake_status.fw_version = "OpenScope Oct  1 2026 03:00:00";
    esp_comm_set_status_provider(status_provider);
    route(buf, frame(buf, ESP_CMD_STATUS, 0, 0), 0);
    CHECK(take(&off, p, &pl) == ESP_RSP_STATUS, "STATUS answered");
    CHECK(pl == ESP_STATUS_FIXED_LEN + strlen(fake_status.fw_version), "STATUS length = fixed + version");
    CHECK(p[0] == ESP_PROTO_VERSION, "byte 0 = protocol version");
    CHECK(p[1] == 1 && p[2] == 73 && p[3] == 0x03, "mode, battery %, flags");
    CHECK(p[4] == (3987 & 0xFF) && p[5] == (3987 >> 8), "battery_mv little-endian");
    CHECK(p[6] == 0x04 && p[7] == 0x03 && p[8] == 0x02 && p[9] == 0x01, "uptime little-endian");
    CHECK(p[10] == 0xD0 && p[13] == 0xA0, "usb_tx_stalls little-endian");
    CHECK(p[14] == 0x34 && p[15] == 0x12, "usb_heals little-endian");
    CHECK(p[16] == strlen(fake_status.fw_version) &&
          memcmp(p + 17, fake_status.fw_version, p[16]) == 0, "version string, length-prefixed");
}

static void test_status_reflects_live_state(void)
{
    uint8_t buf[16], p[64]; size_t off = 0;
    reset();
    esp_comm_set_status_provider(status_provider);
    fake_status.fw_version = "v"; fake_status.current_mode = 0;
    route(buf, frame(buf, ESP_CMD_STATUS, 0, 0), 0);
    take(&off, p, 0);
    CHECK(p[1] == 0, "mode 0 reported");
    fake_status.current_mode = 2;
    route(buf, frame(buf, ESP_CMD_STATUS, 0, 0), 0);
    take(&off, p, 0);
    CHECK(p[1] == 2, "mode change is visible on the next STATUS (not hardcoded)");
}

static void test_ping_reports_bound_version(void)
{
    uint8_t buf[16], p[64]; uint16_t pl = 0; size_t off = 0;
    reset();
    fake_status.fw_version = "OpenScope 0.4.0+rp";
    esp_comm_set_status_provider(status_provider);
    route(buf, frame(buf, ESP_CMD_PING, 0, 0), 0);
    CHECK(take(&off, p, &pl) == ESP_RSP_DATA, "PING -> DATA");
    CHECK(pl == strlen("OpenScope 0.4.0+rp") && memcmp(p, "OpenScope 0.4.0+rp", pl) == 0,
          "PING carries the firmware's own version string");
}

static void test_bad_checksum_is_answered(void)
{
    uint8_t buf[16], p[8]; size_t n = frame(buf, ESP_CMD_PING, 0, 0), off = 0;
    reset();
    buf[n - 1] ^= 0x5A;
    route(buf, n, 0);
    CHECK(take(&off, p, 0) == ESP_RSP_NAK && p[0] == ESP_ERR_BAD_CHECKSUM,
          "corrupted frame gets NAK(BAD_CHECKSUM), not silence");
    CHECK(shell_len == 0, "corrupted frame does not leak into the shell");
}

static void test_oversize_frame_is_swallowed(void)
{
    uint8_t buf[600], p[8]; size_t off = 0, n;
    reset();
    buf[0] = 0xAA; buf[1] = ESP_CMD_PING; buf[2] = 0x01; buf[3] = 0x2C;   /* 300 > 256 */
    for (int i = 0; i < 300; i++) buf[4 + i] = (uint8_t)('A' + i % 26);   /* looks like text */
    buf[304] = 0x00;                                                      /* its checksum */
    n = 305;
    memcpy(buf + n, "help\r", 5); n += 5;
    route(buf, n, 0);
    CHECK(take(&off, p, 0) == ESP_RSP_NAK && p[0] == ESP_ERR_BAD_LENGTH,
          "oversize length gets NAK(BAD_LENGTH)");
    CHECK(strcmp(shell, "help\r") == 0,
          "the oversize frame's payload is swallowed; only the text after it reaches the shell");
}

static void test_truncated_frame_times_out_and_shell_recovers(void)
{
    uint8_t buf[16];
    reset();
    size_t n = frame(buf, ESP_CMD_BUTTON, (const uint8_t *)"\x09", 1);
    route(buf, n - 2, 1000);                     /* host dies mid-frame */
    route("l", 1, 1000 + ESP_RX_GAP_MS - 1);     /* still inside the gap: it is the payload */
    CHECK(shell_len == 0 && esp_comm_rx_in_frame(), "a byte inside the gap window is still a frame byte");
    reset();
    route(buf, n - 2, 1000);
    CHECK(esp_comm_rx_poll(1000 + ESP_RX_GAP_MS) == true, "poll abandons the stale frame after the gap");
    CHECK(tx_len == 0, "abandoning is silent: no unsolicited reply to be mistaken for the next answer");
    route("version\r", 8, 1000 + ESP_RX_GAP_MS + 5);
    CHECK(strcmp(shell, "version\r") == 0, "after the timeout the shell hears the operator again");
    esp_rx_stats_t st; esp_comm_get_rx_stats(&st);
    CHECK(st.gap_timeouts == 1, "gap timeout counted");
}

static void test_route_expires_stale_frame_itself(void)
{
    uint8_t buf[16];
    reset();
    size_t n = frame(buf, ESP_CMD_PING, 0, 0);
    route(buf, n - 1, 0);                        /* no poll call in between */
    route("x\r", 2, 10 * ESP_RX_GAP_MS);
    CHECK(strcmp(shell, "x\r") == 0, "route() itself expires a stale frame before routing new bytes");
}

static void test_stubs_do_not_claim_success(void)
{
    static const uint8_t cmds[] = {
        ESP_CMD_MODULE_START, ESP_CMD_MODULE_DATA, ESP_CMD_MODULE_END,
        ESP_CMD_MODULE_LIST, ESP_CMD_MODULE_DELETE, ESP_CMD_FW_UPDATE_START,
        ESP_CMD_FW_UPDATE_DATA, ESP_CMD_FW_UPDATE_COMMIT, ESP_CMD_FRAMEBUFFER,
        ESP_CMD_SIGNAL_CONFIG };
    uint8_t buf[32], p[8], arg[5] = {0, 0, 0, 1, 0};
    for (size_t i = 0; i < sizeof(cmds); i++) {
        size_t off = 0;
        reset();
        route(buf, frame(buf, cmds[i], arg, sizeof(arg)), 0);
        char msg[80];
        snprintf(msg, sizeof(msg), "unimplemented cmd 0x%02X answers NAK(UNSUPPORTED), not ACK", cmds[i]);
        CHECK(take(&off, p, 0) == ESP_RSP_NAK && p[0] == ESP_ERR_UNSUPPORTED, msg);
    }
}

static void test_unknown_command(void)
{
    uint8_t buf[16], p[8]; size_t off = 0;
    reset();
    route(buf, frame(buf, 0x7E, 0, 0), 0);
    CHECK(take(&off, p, 0) == ESP_RSP_NAK && p[0] == ESP_ERR_UNKNOWN_CMD, "unknown cmd -> NAK(UNKNOWN_CMD)");
}

static void test_button_injection(void)
{
    uint8_t buf[16], p[8]; size_t off;

    reset(); off = 0;
    route(buf, frame(buf, ESP_CMD_BUTTON, (const uint8_t *)"\x09", 1), 0);
    CHECK(take(&off, p, 0) == ESP_RSP_NAK && p[0] == ESP_ERR_UNSUPPORTED,
          "BUTTON with no injector bound: UNSUPPORTED, not a false ACK");

    reset(); off = 0; esp_comm_set_button_injector(button_injector);
    route(buf, frame(buf, ESP_CMD_BUTTON, (const uint8_t *)"\x09", 1), 0);
    CHECK(take(&off, p, 0) == ESP_RSP_ACK && injected_n == 1 && injected[0] == 9,
          "BUTTON MENU(9) queued exactly once, then ACK");

    reset(); off = 0; esp_comm_set_button_injector(button_injector); inject_ok = false;
    route(buf, frame(buf, ESP_CMD_BUTTON, (const uint8_t *)"\x09", 1), 0);
    CHECK(take(&off, p, 0) == ESP_RSP_NAK && p[0] == ESP_ERR_NOT_READY,
          "queue full: NAK(NOT_READY)");

    reset(); off = 0; esp_comm_set_button_injector(button_injector);
    route(buf, frame(buf, ESP_CMD_BUTTON, (const uint8_t *)"\x00", 1), 0);
    CHECK(take(&off, p, 0) == ESP_RSP_NAK && p[0] == ESP_ERR_BAD_ARG && injected_n == 0,
          "button id 0 rejected, nothing injected");

    reset(); off = 0; esp_comm_set_button_injector(button_injector);
    route(buf, frame(buf, ESP_CMD_BUTTON, (const uint8_t *)"\x10", 1), 0);
    CHECK(take(&off, p, 0) == ESP_RSP_NAK && p[0] == ESP_ERR_BAD_ARG && injected_n == 0,
          "button id 16 rejected, nothing injected");

    reset(); off = 0; esp_comm_set_button_injector(button_injector);
    route(buf, frame(buf, ESP_CMD_BUTTON, (const uint8_t *)"\x09\x09", 2), 0);
    CHECK(take(&off, p, 0) == ESP_RSP_NAK && p[0] == ESP_ERR_BAD_LENGTH && injected_n == 0,
          "BUTTON with 2-byte payload rejected");
}

static void test_block_and_byte_writers_agree(void)
{
    uint8_t buf[16];
    reset();
    fake_status.fw_version = "abc";
    esp_comm_set_status_provider(status_provider);
    route(buf, frame(buf, ESP_CMD_STATUS, 0, 0), 0);
    CHECK(block_calls == 3, "one response = header, payload, checksum writes (no per-byte USB sends)");
    size_t a = tx_len; uint8_t copy[128]; memcpy(copy, tx, a);
    esp_comm_set_block_writer(0);
    esp_comm_set_writer(byte_writer);
    route(buf, frame(buf, ESP_CMD_STATUS, 0, 0), 0);
    CHECK(tx_bytes_len == a && memcmp(tx_bytes, copy, a) == 0, "byte writer emits identical bytes");
}

static void test_get_meter(void)
{
    uint8_t buf[16], p[64]; uint16_t pl = 0; size_t off;

    reset(); off = 0;
    route(buf, frame(buf, ESP_CMD_GET_METER, 0, 0), 0);
    CHECK(take(&off, p, 0) == ESP_RSP_NAK && p[0] == ESP_ERR_UNSUPPORTED,
          "GET_METER with no meter bound: UNSUPPORTED");

    reset(); off = 0; esp_comm_set_meter_provider(meter_provider);
    route(buf, frame(buf, ESP_CMD_GET_METER, 0, 0), 0);
    CHECK(take(&off, p, 0) == ESP_RSP_NAK && p[0] == ESP_ERR_NOT_READY,
          "no reading yet: NOT_READY, not a zero reading");

    reset(); off = 0; esp_comm_set_meter_provider(meter_provider); meter_ready = true;
    fake_meter.update_count = 0x01020304;
    fake_meter.value = -1.6141f;
    fake_meter.raw_bcd = 16141;
    fake_meter.decimal_pos = 3;
    fake_meter.result_class = 1;
    fake_meter.flags = ESP_METER_FLAG_NEGATIVE | ESP_METER_FLAG_AUTO;
    fake_meter.submode = 0;
    fake_meter.unit_variant = 0;
    strcpy(fake_meter.unit, "V");
    strcpy(fake_meter.display, "-1.6141");
    route(buf, frame(buf, ESP_CMD_GET_METER, 0, 0), 0);
    CHECK(take(&off, p, &pl) == ESP_RSP_METER_FRAME, "GET_METER -> METER_FRAME");
    float v; memcpy(&v, p + 4, 4);
    CHECK(p[0] == 0x04 && p[3] == 0x01, "update_count little-endian");
    CHECK(v == -1.6141f, "value is IEEE-754 LE float");
    CHECK((int16_t)(p[8] | (p[9] << 8)) == 16141, "raw BCD travels next to the scaled value");
    CHECK(p[10] == 3 && p[11] == 1 && p[12] == 0x05 && p[13] == 0 && p[14] == 0, "decimal, class, flags, submode, variant");
    CHECK(p[15] == 1 && p[16] == 'V', "unit length-prefixed");
    CHECK(p[17] == 7 && memcmp(p + 18, "-1.6141", 7) == 0, "display text length-prefixed");
    CHECK(pl == ESP_METER_FIXED_LEN + 1 + 1 + 7, "METER_FRAME length");

    reset(); off = 0; esp_comm_set_meter_provider(meter_provider);
    meter_ready = true; meter_wrong_mode = true;
    route(buf, frame(buf, ESP_CMD_GET_METER, 0, 0), 0);
    CHECK(take(&off, p, 0) == ESP_RSP_NAK && p[0] == ESP_ERR_UNSUPPORTED_IN_MODE,
          "outside meter mode: UNSUPPORTED_IN_MODE, never the frozen last reading");

    reset(); off = 0; esp_comm_set_meter_provider(meter_provider); meter_ready = true;
    route(buf, frame(buf, ESP_CMD_GET_METER, (const uint8_t *)"\x01", 1), 0);
    CHECK(take(&off, p, 0) == ESP_RSP_NAK && p[0] == ESP_ERR_BAD_LENGTH, "GET_METER takes no payload");
}

static void test_long_version_is_capped(void)
{
    uint8_t buf[16], p[128]; uint16_t pl = 0; size_t off = 0;
    static const char v60[] = "OpenScope 2C53T Oct  1 2026 04:59:12 +remote-protocol-build-x";
    reset();
    fake_status.fw_version = v60;                      /* 60 chars > ESP_FW_VERSION_MAX */
    esp_comm_set_status_provider(status_provider);
    route(buf, frame(buf, ESP_CMD_STATUS, 0, 0), 0);
    CHECK(take(&off, p, &pl) == ESP_RSP_STATUS, "STATUS with an over-long version still answers");
    CHECK(p[16] == ESP_FW_VERSION_MAX && pl == ESP_STATUS_FIXED_LEN + ESP_FW_VERSION_MAX,
          "version capped at ESP_FW_VERSION_MAX (and the stack buffer is sized to it)");
    route(buf, frame(buf, ESP_CMD_PING, 0, 0), 0);
    CHECK(take(&off, p, &pl) == ESP_RSP_DATA && pl == ESP_FW_VERSION_MAX, "PING capped the same way");

    reset(); off = 0;
    fake_status.fw_version = "OpenScope Oct  1 2026 04:59:12";   /* the firmware's real format */
    esp_comm_set_status_provider(status_provider);
    route(buf, frame(buf, ESP_CMD_PING, 0, 0), 0);
    CHECK(take(&off, p, &pl) == ESP_RSP_DATA && pl == 30 && memcmp(p + 25, "59:12", 5) == 0,
          "the production version string travels whole, seconds included");
}

static void test_oversize_then_silence_does_not_deafen_shell(void)
{
    uint8_t hdr[6] = { 0xAA, ESP_CMD_PING, 0xFF, 0xFF, 'x', 'y' };  /* announces 65535, sends 2 */
    reset();
    route(hdr, sizeof(hdr), 1000);
    CHECK(esp_comm_rx_in_frame(), "the lying frame is being discarded");
    CHECK(esp_comm_rx_poll(1000 + ESP_RX_GAP_MS) == true, "silence abandons it");
    CHECK(!esp_comm_rx_in_frame(), "…including the discard count");
    route("version\r", 8, 1000 + ESP_RX_GAP_MS + 1);
    CHECK(strcmp(shell, "version\r") == 0, "the operator is heard right after the gap, not 64 KB later");
}

static void test_touch_refreshes_open_frame_only(void)
{
    uint8_t buf[16], p[8]; size_t off = 0;
    size_t n = frame(buf, ESP_CMD_PING, 0, 0);
    reset();
    route(buf, n - 1, 0);                               /* frame open at chunk end */
    esp_comm_rx_touch(ESP_RX_GAP_MS + 10);              /* chunk took a while to process */
    CHECK(esp_comm_rx_poll(ESP_RX_GAP_MS + 20) == false, "a touched frame is not expired by work time");
    route(buf + n - 1, 1, ESP_RX_GAP_MS + 30);
    CHECK(take(&off, p, 0) == ESP_RSP_DATA, "…and completes when its last byte arrives");
    esp_comm_rx_touch(10 * ESP_RX_GAP_MS);
    CHECK(!esp_comm_rx_in_frame(), "touch never opens a frame");
}

/* ─── GET_WAVEFORM / WAVEFORM_FRAME ─── */

static uint8_t wp[ESP_WAVE_HDR_LEN + ESP_WAVE_MAX_SAMPLES + 64];

static uint32_t u32le(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
static uint16_t u16le(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

static void get_waveform(uint8_t mask)
{
    uint8_t buf[16];
    route(buf, frame(buf, ESP_CMD_GET_WAVEFORM, &mask, 1), 0);
}

/* Exactly one NAK with `code` and nothing else: no frame went out before it. */
static bool only_nak(uint8_t code)
{
    size_t off = 0; uint16_t pl = 0;
    return take(&off, wp, &pl) == ESP_RSP_NAK && pl == 1 && wp[0] == code && off == tx_len;
}

static void test_waveform_layout(void)
{
    size_t off = 0; uint16_t pl = 0;
    reset(); esp_comm_set_waveform_provider(wave_provider);
    get_waveform(ESP_WAVE_MASK_CH1);
    CHECK(take(&off, wp, &pl) == ESP_RSP_WAVEFORM_FRAME, "GET_WAVEFORM(CH1) -> WAVEFORM_FRAME");
    CHECK(off == tx_len, "CH1 only: exactly one frame");
    CHECK(wave_mask_seen == ESP_WAVE_MASK_CH1, "provider asked for the requested mask only");
    CHECK(pl == ESP_WAVE_HDR_LEN + ESP_WAVE_MAX_SAMPLES, "length = header + 1024 samples (> the 256 B receive cap: legal outbound)");
    CHECK(u32le(wp + 0) == 0x01020304, "frame_id little-endian");
    CHECK(wp[4] == 0, "channel 0 = CH1");
    CHECK(wp[5] == (ESP_WAVE_FLAG_TB_MEASURED | ESP_WAVE_FLAG_VDIV_MEASURED | ESP_WAVE_FLAG_TIME_ORDERED),
          "flags: rate + volts measured, time-ordered, not calibrated, not synthetic");
    CHECK(wp[6] == 0x10 && wp[7] == 6, "timebase code in force, vdiv index");
    CHECK(u16le(wp + 8) == ESP_WAVE_MAX_SAMPLES, "sample_count");
    CHECK(u16le(wp + 10) == ESP_WAVE_HDR_LEN, "header_len where §3.5 had 'reserved'");
    CHECK(u32le(wp + 12) == 12490, "sample_rate_hz");
    CHECK(u32le(wp + 16) == 1264486, "uv_per_div");
    CHECK(u16le(wp + 20) == 32 && u16le(wp + 22) == 128, "counts_per_div, head_skip");
    CHECK(memcmp(wp + ESP_WAVE_HDR_LEN, wave_ch1, ESP_WAVE_MAX_SAMPLES) == 0, "samples are the provider's record, byte for byte");
    CHECK(block_calls == 4, "one frame = sync/len, header, samples, checksum (the record is not re-copied)");
    CHECK(esp_comm_checksum(wave_ch1, ESP_WAVE_MAX_SAMPLES) != 0,
          "test data: the samples' XOR is non-zero, so the frame checksum really covers them");
}

static void test_waveform_both_channels_one_capture(void)
{
    size_t off = 0; uint16_t pl = 0; uint32_t id1;
    reset(); esp_comm_set_waveform_provider(wave_provider);
    get_waveform(ESP_WAVE_MASK_ALL);
    CHECK(wave_calls == 1, "one snapshot serves both channels (one capture, not two)");
    CHECK(take(&off, wp, &pl) == ESP_RSP_WAVEFORM_FRAME && wp[4] == 0, "first frame is CH1");
    id1 = u32le(wp);
    CHECK(take(&off, wp, &pl) == ESP_RSP_WAVEFORM_FRAME && wp[4] == 1, "second frame is CH2");
    CHECK(u32le(wp) == id1, "CH1 and CH2 carry the same frame_id");
    CHECK(wp[7] == 8 && u32le(wp + 16) == 6586022, "CH2 carries its own range and volts/div");
    CHECK(wp[5] == (ESP_WAVE_FLAG_TB_MEASURED | ESP_WAVE_FLAG_VDIV_PROVISIONAL | ESP_WAVE_FLAG_TIME_ORDERED),
          "a PROVISIONAL range is flagged provisional, not measured");
    CHECK(memcmp(wp + ESP_WAVE_HDR_LEN, wave_ch2, ESP_WAVE_MAX_SAMPLES) == 0, "CH2 samples (0xAA inside a payload is fine)");
    CHECK(off == tx_len, "nothing after the two frames");

    reset(); esp_comm_set_waveform_provider(wave_provider);
    get_waveform(ESP_WAVE_MASK_CH2);
    off = 0;
    CHECK(take(&off, wp, &pl) == ESP_RSP_WAVEFORM_FRAME && wp[4] == 1 && off == tx_len, "CH2 alone: one CH2 frame");
}

static void test_waveform_refusals(void)
{
    reset();
    get_waveform(ESP_WAVE_MASK_CH1);
    CHECK(only_nak(ESP_ERR_UNSUPPORTED), "no provider bound: UNSUPPORTED");

    reset(); esp_comm_set_waveform_provider(wave_provider); wave_result = ESP_WAVE_NO_DATA;
    get_waveform(ESP_WAVE_MASK_CH1);
    CHECK(only_nak(ESP_ERR_NO_CAPTURE_DATA), "fpga_data_ready() false: NO_CAPTURE_DATA, never a demo/zero frame");

    reset(); esp_comm_set_waveform_provider(wave_provider); wave_result = ESP_WAVE_WRONG_MODE;
    get_waveform(ESP_WAVE_MASK_CH1);
    CHECK(only_nak(ESP_ERR_UNSUPPORTED_IN_MODE), "outside scope mode: UNSUPPORTED_IN_MODE, never the frozen record");

    reset(); esp_comm_set_waveform_provider(wave_provider); wave_result = ESP_WAVE_BUSY;
    get_waveform(ESP_WAVE_MASK_CH1);
    CHECK(only_nak(ESP_ERR_NOT_READY), "no tear-free copy: NOT_READY");

    reset(); esp_comm_set_waveform_provider(wave_provider); fake_wave.frame_id = 0;
    get_waveform(ESP_WAVE_MASK_CH1);
    CHECK(only_nak(ESP_ERR_NO_CAPTURE_DATA), "frame_id 0 (no committed record) is refused");
}

static void test_waveform_lying_provider_is_refused(void)
{
    reset(); esp_comm_set_waveform_provider(wave_provider); wave_lie = true;
    get_waveform(ESP_WAVE_MASK_ALL);
    CHECK(only_nak(ESP_ERR_NO_CAPTURE_DATA),
          "provider says OK but the record is synthetic: refused whole, no frame with a flag the host might ignore");
}

static void test_waveform_args(void)
{
    uint8_t buf[16], two[2] = { 1, 1 };
    reset(); esp_comm_set_waveform_provider(wave_provider);
    get_waveform(0);
    CHECK(only_nak(ESP_ERR_BAD_ARG) && wave_calls == 0, "mask 0 asks for nothing: BAD_ARG");
    reset(); esp_comm_set_waveform_provider(wave_provider);
    get_waveform(0x04);
    CHECK(only_nak(ESP_ERR_BAD_ARG) && wave_calls == 0, "mask bit 2 (no such channel): BAD_ARG");
    reset(); esp_comm_set_waveform_provider(wave_provider);
    get_waveform(0x83);
    CHECK(only_nak(ESP_ERR_BAD_ARG) && wave_calls == 0, "stray high bit next to valid ones: BAD_ARG");
    reset(); esp_comm_set_waveform_provider(wave_provider);
    route(buf, frame(buf, ESP_CMD_GET_WAVEFORM, 0, 0), 0);
    CHECK(only_nak(ESP_ERR_BAD_LENGTH), "no mask byte: BAD_LENGTH");
    reset(); esp_comm_set_waveform_provider(wave_provider);
    route(buf, frame(buf, ESP_CMD_GET_WAVEFORM, two, 2), 0);
    CHECK(only_nak(ESP_ERR_BAD_LENGTH), "two payload bytes: BAD_LENGTH");
}

static void test_waveform_all_or_nothing(void)
{
    reset(); esp_comm_set_waveform_provider(wave_provider); fake_wave.ch[1].samples = 0;
    get_waveform(ESP_WAVE_MASK_ALL);
    CHECK(only_nak(ESP_ERR_NO_CAPTURE_DATA), "CH2 has no record: one NAK, and CH1 was NOT sent first");

    reset(); esp_comm_set_waveform_provider(wave_provider); fake_wave.ch[1].samples = 0;
    get_waveform(ESP_WAVE_MASK_CH1);
    CHECK(tx_len > 0 && tx[1] == ESP_RSP_WAVEFORM_FRAME, "an unrequested channel is not checked");

    reset(); esp_comm_set_waveform_provider(wave_provider); fake_wave.ch[0].count = 0;
    get_waveform(ESP_WAVE_MASK_CH1);
    CHECK(only_nak(ESP_ERR_NO_CAPTURE_DATA), "zero samples is not a record");

    reset(); esp_comm_set_waveform_provider(wave_provider); fake_wave.ch[0].count = ESP_WAVE_MAX_SAMPLES + 1;
    get_waveform(ESP_WAVE_MASK_CH1);
    CHECK(only_nak(ESP_ERR_NO_CAPTURE_DATA), "more samples than one FPGA record: refused (bound holds)");
}

static void test_waveform_flags_are_derived(void)
{
    size_t off; uint16_t pl;

    /* A tier that disowns its number: the number does not travel. */
    reset(); esp_comm_set_waveform_provider(wave_provider);
    fake_wave.timebase_tier = ESP_TIER_NONE;            /* e.g. incoherent code 0x08 */
    fake_wave.ch[0].vdiv_tier = ESP_TIER_NONE;          /* e.g. railed range 2 */
    get_waveform(ESP_WAVE_MASK_CH1); off = 0;
    CHECK(take(&off, wp, &pl) == ESP_RSP_WAVEFORM_FRAME, "frame sent");
    CHECK(u32le(wp + 12) == 0 && u32le(wp + 16) == 0, "tier NONE: rate and volts zeroed on the wire");
    CHECK((wp[5] & (ESP_WAVE_FLAG_TB_MEASURED | ESP_WAVE_FLAG_TB_PROVISIONAL |
                    ESP_WAVE_FLAG_VDIV_MEASURED | ESP_WAVE_FLAG_VDIV_PROVISIONAL)) == 0,
          "tier NONE: neither measured nor provisional");

    /* A tier that claims a number that is not there. */
    reset(); esp_comm_set_waveform_provider(wave_provider);
    fake_wave.sample_rate_hz = 0; fake_wave.ch[0].uv_per_div = 0;
    get_waveform(ESP_WAVE_MASK_CH1); off = 0;
    take(&off, wp, &pl);
    CHECK((wp[5] & (ESP_WAVE_FLAG_TB_MEASURED | ESP_WAVE_FLAG_VDIV_MEASURED)) == 0,
          "MEASURED tier with a 0 value is not reported as measured");

    /* Provisional rate (code 0x0D). */
    reset(); esp_comm_set_waveform_provider(wave_provider);
    fake_wave.timebase_tier = ESP_TIER_PROVISIONAL; fake_wave.sample_rate_hz = 123663;
    get_waveform(ESP_WAVE_MASK_CH1); off = 0;
    take(&off, wp, &pl);
    CHECK((wp[5] & ESP_WAVE_FLAG_TB_PROVISIONAL) && !(wp[5] & ESP_WAVE_FLAG_TB_MEASURED) &&
          u32le(wp + 12) == 123663, "PROVISIONAL rate travels flagged provisional, not measured");

    /* Display and hardware disagree on the timebase: the rate is withheld. */
    reset(); esp_comm_set_waveform_provider(wave_provider);
    fake_wave.timebase_disagrees = true;
    get_waveform(ESP_WAVE_MASK_CH1); off = 0;
    take(&off, wp, &pl);
    CHECK(u32le(wp + 12) == 0, "display != hardware timebase: rate withheld");
    CHECK((wp[5] & ESP_WAVE_FLAG_TB_DISAGREES) && !(wp[5] & ESP_WAVE_FLAG_TB_MEASURED),
          "…and flagged, so a 0 rate on a measured code is explained");

    /* Not time-ordered. */
    reset(); esp_comm_set_waveform_provider(wave_provider);
    fake_wave.time_ordered = false;
    get_waveform(ESP_WAVE_MASK_CH1); off = 0;
    take(&off, wp, &pl);
    CHECK(!(wp[5] & ESP_WAVE_FLAG_TIME_ORDERED), "time_ordered follows the record");

    /* No snapshot can make the frame claim calibration or synthesis. */
    reset(); esp_comm_set_waveform_provider(wave_provider);
    get_waveform(ESP_WAVE_MASK_ALL); off = 0;
    take(&off, wp, &pl);
    CHECK(!(wp[5] & (ESP_WAVE_FLAG_CALIBRATED | ESP_WAVE_FLAG_SYNTHETIC)), "CH1: calibrated and synthetic clear");
    take(&off, wp, &pl);
    CHECK(!(wp[5] & (ESP_WAVE_FLAG_CALIBRATED | ESP_WAVE_FLAG_SYNTHETIC)), "CH2: calibrated and synthetic clear");
}

static void test_waveform_byte_writer_agrees(void)
{
    static uint8_t copy[2 * (ESP_WAVE_HDR_LEN + ESP_WAVE_MAX_SAMPLES + 5)];
    size_t a;
    reset(); esp_comm_set_waveform_provider(wave_provider);
    get_waveform(ESP_WAVE_MASK_ALL);
    a = tx_len; memcpy(copy, tx, a);
    esp_comm_set_block_writer(0);
    esp_comm_set_writer(byte_writer);
    get_waveform(ESP_WAVE_MASK_ALL);
    CHECK(tx_bytes_len == a && memcmp(tx_bytes, copy, a) == 0, "byte writer emits the identical two-part frames");
}

/* --- stage-086: DESCRIPTION, SET_PROBE, SCOPE_STATE append, host badge --- */

static uint32_t rd_u32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
static uint16_t rd_u16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

static int request(uint8_t cmd, const uint8_t *p, uint16_t n, uint8_t *out, uint16_t *outn)
{
    uint8_t f[64];
    size_t off = 0;
    tx_len = 0;
    route(f, frame(f, cmd, p, n), 1000);
    return take(&off, out, outn);
}

static void test_description_layout(void)
{
    uint8_t pl[512]; uint16_t n = 0;
    reset();
    esp_comm_set_control_provider(&ctl_ops);
    CHECK(request(ESP_CMD_GET_DESCRIPTION, 0, 0, pl, &n) == ESP_RSP_DESCRIPTION, "DESCRIPTION reply");
    /* 239 B of sections 1-4, + 3 + 12 + 2 + (1 + 15) + (1 + 13) of section 7 */
    CHECK(n == 239 + 3 + 12 + 2 + 16 + 14, "286 bytes");
    CHECK(ESP_DESCRIPTION_LEN == 320, "the largest reply is 320 bytes");
    CHECK(pl[0] == ESP_DESCRIPTION_VERSION, "version byte");

    /* walk the sections the way a host does */
    const uint8_t *p = pl + 1, *end = pl + n;
    int seen = 0;
    while (p + 3 <= end) {
        uint8_t id = p[0]; uint16_t len = rd_u16(p + 1);
        const uint8_t *d = p + 3;
        CHECK(d + len <= end, "section inside the payload");
        if (id == ESP_DESC_SEC_TIMEBASE) {
            seen |= 1;
            CHECK(len == ESP_DESC_TB_LEN, "timebase len");
            CHECK(rd_u16(d) == 32 && d[2] == ESP_TIMEBASE_CODES, "samples/div, n");
            const uint8_t *e = d + 3;
            CHECK(rd_u32(e + 5 * 0x10) == 12490 && e[5 * 0x10 + 4] == ESP_TIER_MEASURED, "0x10 measured");
            CHECK(rd_u32(e + 5 * 0x0D) == 119678 && e[5 * 0x0D + 4] == ESP_TIER_PROVISIONAL, "0x0D provisional");
            CHECK(rd_u32(e + 5 * 0x08) == 0 && e[5 * 0x08 + 4] == ESP_TIER_NONE, "tier NONE rate zeroed");
            CHECK(rd_u32(e + 5 * 0x0F) == 0 && e[5 * 0x0F + 4] == ESP_TIER_NONE, "zero rate never vouched for");
            CHECK(rd_u32(e + 5 * 20) == 0, "last code present");
        } else if (id == ESP_DESC_SEC_RANGES) {
            seen |= 2;
            CHECK(len == ESP_DESC_RANGES_LEN, "ranges len");
            CHECK(rd_u16(d) == 32 && d[2] == ESP_VDIV_RANGES && d[3] == 2, "counts/div, n, channels");
            const uint8_t *e = d + 4;
            CHECK(rd_u32(e + 5 * 6) == 100006 && e[5 * 6 + 4] == ESP_TIER_MEASURED, "CH1 range 6");
            CHECK(rd_u32(e + 5 * (10 + 9)) == 200009, "CH2 range 9 (CH1 block first)");
            CHECK(rd_u32(e + 5 * (10 + 3)) == 0 && e[5 * 13 + 4] == ESP_TIER_NONE, "tier NONE volts zeroed");
        } else if (id == ESP_DESC_SEC_TRIGGER) {
            seen |= 4;
            CHECK(len == ESP_DESC_TRIGGER_LEN, "trigger len");
            CHECK((int16_t)rd_u16(d) == -100 && (int16_t)rd_u16(d + 2) == 100, "level limits");
            CHECK(d[4] == 128 && rd_u16(d + 5) == 256 && rd_u16(d + 7) == 206, "center, num, den");
            CHECK(d[9] == 1 && d[10] == 254 && (int8_t)d[11] == -28, "code limits, record offset");
        } else if (id == ESP_DESC_SEC_PROBE) {
            seen |= 8;
            CHECK(len == 2 && d[0] == 10 && d[1] == 1, "probe 10 / unknown 7 sent as 1");
        } else if (id == ESP_DESC_SEC_IDENTITY) {
            seen |= 16;
            CHECK(len == 12 + 2 + 16 + 14, "identity len = bytes written");
            CHECK(d[0] == 0xA0 && d[11] == 0xAB, "uid bytes in order");
            CHECK(rd_u16(d + 12) == ESP_API_LEVEL && ESP_API_LEVEL == 1, "api level 1");
            CHECK(d[14] == 15 && memcmp(d + 15, "guest-coldtrace", 15) == 0, "target");
            CHECK(d[30] == 13 && memcmp(d + 31, "0a1af35-dirty", 13) == 0, "git rev");
        }
        p = d + len;
    }
    CHECK(p == end, "sections end exactly at the payload end");
    CHECK(seen == 31, "all five sections");
}

/* stage-190: section 7 texts are cut to ESP_DESC_TEXT_MAX; NULL is empty. */
static void test_description_identity_texts(void)
{
    static const char long_target[] = "guest-coldtrace-hwspi-usbtask-and-more-40-chars";
    uint8_t pl[512]; uint16_t n = 0;
    reset();
    esp_comm_set_control_provider(&ctl_ops);
    fake_desc.build_target = long_target;
    fake_desc.git_rev = 0;
    CHECK(request(ESP_CMD_GET_DESCRIPTION, 0, 0, pl, &n) == ESP_RSP_DESCRIPTION, "DESCRIPTION reply");
    const uint8_t *d = pl + 239;                         /* section 7 header */
    CHECK(d[0] == ESP_DESC_SEC_IDENTITY, "section 7 after the probe section");
    CHECK(rd_u16(d + 1) == 12 + 2 + 1 + 31 + 1, "len: target cut to 31, empty rev");
    CHECK(d[3 + 14] == 31 && memcmp(d + 3 + 15, long_target, 31) == 0, "target cut to 31");
    CHECK(d[3 + 46] == 0, "NULL rev sent empty");
    CHECK(n == 239 + 3 + 47, "reply ends after the empty rev");

    fake_desc.build_target = "x";
    fake_desc.git_rev = "0123456789abcdef0123456789abcdef0123";   /* 36 */
    CHECK(request(ESP_CMD_GET_DESCRIPTION, 0, 0, pl, &n) == ESP_RSP_DESCRIPTION, "DESCRIPTION reply 2");
    CHECK(d[3 + 14] == 1 && d[3 + 15] == 'x', "short target");
    CHECK(d[3 + 16] == 31 && memcmp(d + 3 + 17, fake_desc.git_rev, 31) == 0, "rev cut to 31");
    CHECK(n == 239 + 3 + 12 + 2 + 2 + 32, "reply length");
    desc_defaults();
}

static void test_description_refusals(void)
{
    uint8_t pl[16]; uint16_t n = 0; uint8_t junk = 0;
    reset();
    CHECK(request(ESP_CMD_GET_DESCRIPTION, 0, 0, pl, &n) == ESP_RSP_NAK && pl[0] == ESP_ERR_UNSUPPORTED,
          "no provider: UNSUPPORTED");
    esp_comm_set_control_provider(&ctl_ops);
    CHECK(request(ESP_CMD_GET_DESCRIPTION, &junk, 1, pl, &n) == ESP_RSP_NAK && pl[0] == ESP_ERR_BAD_LENGTH,
          "payload: BAD_LENGTH");
}

static void test_set_probe(void)
{
    uint8_t pl[16]; uint16_t n = 0;
    static const uint8_t good[][2] = { {1, 1}, {1, 10}, {2, 1}, {2, 10} };
    static const uint8_t bad[][2] = { {0, 1}, {3, 1}, {1, 0}, {1, 2}, {2, 100}, {1, 11} };
    reset();
    esp_comm_set_control_provider(&ctl_ops);
    for (unsigned i = 0; i < sizeof(good) / sizeof(good[0]); i++) {
        CHECK(request(ESP_CMD_SET_PROBE, good[i], 2, pl, &n) == ESP_RSP_ACK, "SET_PROBE ACK");
        CHECK(probe_ch_seen == good[i][0] && probe_factor_seen == good[i][1], "provider got the args");
    }
    CHECK(probe_calls == 4, "4 calls");
    for (unsigned i = 0; i < sizeof(bad) / sizeof(bad[0]); i++)
        CHECK(request(ESP_CMD_SET_PROBE, bad[i], 2, pl, &n) == ESP_RSP_NAK && pl[0] == ESP_ERR_BAD_ARG,
              "SET_PROBE BAD_ARG");
    CHECK(request(ESP_CMD_SET_PROBE, good[0], 1, pl, &n) == ESP_RSP_NAK && pl[0] == ESP_ERR_BAD_LENGTH,
          "SET_PROBE BAD_LENGTH");
    CHECK(probe_calls == 4, "a refused request never reaches the provider");
}

static void test_scope_state_probe_append(void)
{
    uint8_t pl[64]; uint16_t n = 0;
    reset();
    esp_comm_set_control_provider(&ctl_ops);
    CHECK(request(ESP_CMD_GET_SCOPE_STATE, 0, 0, pl, &n) == ESP_RSP_SCOPE_STATE, "SCOPE_STATE reply");
    CHECK(n == 38 && pl[0] == 1 && pl[1] == 38, "still v1, length byte 38");
    CHECK(pl[36] == 10 && pl[37] == 1, "probe CH1 10, CH2 1");
    fake_state.ch[1].probe = 0;
    request(ESP_CMD_GET_SCOPE_STATE, 0, 0, pl, &n);
    CHECK(pl[37] == 1, "a provider 0 is sent as 1");
}

static void test_host_active_clock(void)
{
    uint8_t f[16];
    reset();
    CHECK(!esp_comm_host_active(0) && !esp_comm_host_active(5000), "no frame yet: not active");
    route("help\r\n", 6, 100);
    CHECK(!esp_comm_host_active(200), "shell text does not count");
    route(f, frame(f, ESP_CMD_PING, 0, 0), 10000);
    CHECK(esp_comm_host_active(10000) && esp_comm_host_active(12999), "active < 3 s after a frame");
    CHECK(!esp_comm_host_active(13000), "inactive at 3 s");
    f[frame(f, ESP_CMD_PING, 0, 0) - 1] ^= 0xFF;            /* bad checksum */
    route(f, 5, 20000);
    CHECK(!esp_comm_host_active(20001), "a corrupt frame does not count");
}

/* ─── stage-230: park owner, scope sets in scope mode, trigger write ─── */

/* A fake unit for the M6 sets: each set answers as usb_debug.c does, through
 * the same acq_ctrl.h helpers, so the mode gate and the write decision are
 * the ones that run on the unit. `set_result` forces NOT_READY. */
static uint8_t fake_mode;                   /* current_mode */
static esp_ctrl_result_t set_result;
static int set_calls, hw_writes;
static uint8_t trig_code_now, trig_src_now;
static int16_t level_seen;
static uint8_t args_seen[4];

static uint8_t code_from_level(int16_t level)    /* the fpga.c relation */
{
    int c = 128 + (level * 256) / 206;
    return (uint8_t)(c < 1 ? 1 : (c > 254 ? 254 : c));
}
static esp_ctrl_result_t set_mode_fn(uint8_t m)
{
    set_calls++; args_seen[0] = m;
    if (set_result != ESP_CTRL_OK) return set_result;
    fake_mode = m;
    return ESP_CTRL_OK;
}
static esp_ctrl_result_t set_timebase_fn(uint8_t code)
{
    set_calls++; args_seen[0] = code;
    if (!acq_scope_set_allowed(fake_mode)) return ESP_CTRL_WRONG_MODE;
    if (set_result != ESP_CTRL_OK) return set_result;
    hw_writes++;
    return ESP_CTRL_OK;
}
static esp_ctrl_result_t set_vdiv_fn(uint8_t ch, uint8_t r)
{
    set_calls++; args_seen[0] = ch; args_seen[1] = r;
    if (!acq_scope_set_allowed(fake_mode)) return ESP_CTRL_WRONG_MODE;
    if (set_result != ESP_CTRL_OK) return set_result;
    hw_writes++;
    return ESP_CTRL_OK;
}
static esp_ctrl_result_t set_trigger_fn(uint8_t mode, uint8_t edge, uint8_t src, int16_t level)
{
    set_calls++; args_seen[0] = mode; args_seen[1] = edge; args_seen[2] = src; level_seen = level;
    if (!acq_scope_set_allowed(fake_mode)) return ESP_CTRL_WRONG_MODE;
    if (acq_trigger_needs_write(trig_code_now, trig_src_now, code_from_level(level), src)) {
        if (set_result != ESP_CTRL_OK) return set_result;
        hw_writes++;
        trig_code_now = code_from_level(level);
        trig_src_now = src;
    }
    return ESP_CTRL_OK;
}
static esp_ctrl_result_t run_stop_fn(bool running)
{
    set_calls++; args_seen[0] = running ? 1 : 0;
    return set_result;
}
static const esp_control_ops_t set_ops = {
    .get_state = ctl_get_state,
    .set_mode = set_mode_fn,
    .set_timebase = set_timebase_fn,
    .set_vdiv = set_vdiv_fn,
    .set_trigger = set_trigger_fn,
    .run_stop = run_stop_fn,
};

static void sets_reset(void)
{
    reset();
    esp_comm_set_control_provider(&set_ops);
    fake_mode = 0; set_result = ESP_CTRL_OK; set_calls = 0; hw_writes = 0;
    trig_code_now = 128; trig_src_now = 0; level_seen = 0;
    memset(args_seen, 0, sizeof(args_seen));
}

/* reply = ACK, or NAK with this error */
static bool nak_is(int rsp, const uint8_t *pl, uint8_t err) { return rsp == ESP_RSP_NAK && pl[0] == err; }

static void test_acq_owner_two_requesters(void)
{
    acq_owner_t o = { 0 };
    int a = 0, b = 0;                       /* two task handles */
    CHECK(!acq_owner_nested(&o, &a) && !acq_owner_nested(&o, &b), "no holder: nobody nested");
    CHECK(acq_owner_release(&o, &a) == ACQ_RELEASE_REFUSE, "resume with no park: refused");
    acq_owner_took(&o, &a);
    CHECK(acq_owner_nested(&o, &a), "A holds");
    CHECK(!acq_owner_nested(&o, &b), "B is not nested in A's park (it must wait for the mutex)");
    CHECK(!acq_owner_enter_nested(&o, &b) && o.depth == 1, "B cannot count itself into A's park");
    CHECK(acq_owner_release(&o, &b) == ACQ_RELEASE_REFUSE, "B's resume does not end A's park");
    CHECK(o.holder == &a && o.depth == 1, "A's park unchanged after B's resume");
    CHECK(acq_owner_release(&o, NULL) == ACQ_RELEASE_REFUSE, "NULL is never a holder");
    /* A nested pause in A (a shell command that pauses inside a parked command) */
    CHECK(acq_owner_enter_nested(&o, &a) && o.depth == 2, "A nested: depth 2");
    CHECK(acq_owner_release(&o, &a) == ACQ_RELEASE_KEEP, "inner resume keeps the park");
    CHECK(acq_owner_nested(&o, &a), "A still holds after the inner resume");
    CHECK(acq_owner_release(&o, &a) == ACQ_RELEASE_GIVE, "outer resume gives the mutex");
    CHECK(o.holder == NULL && o.depth == 0, "no holder after the last resume");
    CHECK(acq_owner_release(&o, &a) == ACQ_RELEASE_REFUSE, "a second resume is refused");
    /* interleaved: A, then B (after A gave the mutex), A's late resume */
    acq_owner_took(&o, &a);
    CHECK(acq_owner_release(&o, &a) == ACQ_RELEASE_GIVE, "A gives");
    acq_owner_took(&o, &b);
    CHECK(acq_owner_release(&o, &a) == ACQ_RELEASE_REFUSE, "A's late resume does not end B's park");
    CHECK(acq_owner_nested(&o, &b) && o.depth == 1, "B keeps its park");
    CHECK(acq_owner_release(&o, &b) == ACQ_RELEASE_GIVE, "B gives");
    /* the count does not wrap */
    acq_owner_took(&o, &a);
    o.depth = UINT8_MAX;
    CHECK(!acq_owner_enter_nested(&o, &a) && o.depth == UINT8_MAX, "full count: refused, no wrap");
}

static void test_acq_mode_gate_and_write_decision(void)
{
    CHECK(acq_scope_set_allowed(0), "scope mode: scope sets allowed");
    for (uint8_t m = 1; m < 8; m++)
        CHECK(!acq_scope_set_allowed(m), "meter, siggen, settings, other: refused");
    CHECK(!acq_trigger_needs_write(150, 0, 150, 0), "same code and source: no write");
    CHECK(acq_trigger_needs_write(150, 0, 151, 0), "code change: write");
    CHECK(acq_trigger_needs_write(150, 0, 150, 1), "source change: write");
    CHECK(acq_trigger_needs_write(150, 1, 150, 0), "source back to CH1: write");
}

static void test_sets_length_and_args(void)
{
    uint8_t pl[16]; uint16_t n = 0;
    static const uint8_t z[6] = { 0 };
    static const struct { uint8_t cmd; uint16_t len; } lens[] = {
        { ESP_CMD_SET_MODE, 1 }, { ESP_CMD_SET_TIMEBASE, 1 }, { ESP_CMD_SET_VDIV, 2 },
        { ESP_CMD_SET_TRIGGER, 5 }, { ESP_CMD_RUN_STOP, 1 },
    };
    sets_reset();
    for (unsigned i = 0; i < sizeof(lens) / sizeof(lens[0]); i++) {
        CHECK(nak_is(request(lens[i].cmd, z, (uint16_t)(lens[i].len - 1), pl, &n), pl, ESP_ERR_BAD_LENGTH),
              "short payload: BAD_LENGTH");
        CHECK(nak_is(request(lens[i].cmd, z, (uint16_t)(lens[i].len + 1), pl, &n), pl, ESP_ERR_BAD_LENGTH),
              "long payload: BAD_LENGTH");
    }
    {   uint8_t m = 4;  CHECK(nak_is(request(ESP_CMD_SET_MODE, &m, 1, pl, &n), pl, ESP_ERR_BAD_ARG), "mode 4: BAD_ARG"); }
    {   uint8_t c = ESP_TIMEBASE_CODES;
        CHECK(nak_is(request(ESP_CMD_SET_TIMEBASE, &c, 1, pl, &n), pl, ESP_ERR_BAD_ARG), "timebase past the table: BAD_ARG"); }
    {   static const uint8_t bad[][2] = { {0, 0}, {3, 0}, {1, ESP_VDIV_RANGES}, {2, 255} };
        for (unsigned i = 0; i < 4; i++)
            CHECK(nak_is(request(ESP_CMD_SET_VDIV, bad[i], 2, pl, &n), pl, ESP_ERR_BAD_ARG), "vdiv channel/range: BAD_ARG"); }
    {   uint8_t r = 2;  CHECK(nak_is(request(ESP_CMD_RUN_STOP, &r, 1, pl, &n), pl, ESP_ERR_BAD_ARG), "run 2: BAD_ARG"); }
    {   /* mode, edge, source, level LE */
        static const uint8_t bad[][5] = {
            {3, 0, 0, 0, 0}, {0, 2, 0, 0, 0}, {0, 0, 2, 0, 0},
            {0, 0, 0, 101, 0}, {0, 0, 0, 0x9B, 0xFF},          /* +101, -101 */
            {0, 0, 0, 0x00, 0x80},                             /* -32768 */
        };
        for (unsigned i = 0; i < sizeof(bad) / sizeof(bad[0]); i++)
            CHECK(nak_is(request(ESP_CMD_SET_TRIGGER, bad[i], 5, pl, &n), pl, ESP_ERR_BAD_ARG), "trigger field: BAD_ARG"); }
    CHECK(set_calls == 0, "no refused request reached the provider");
    {   uint8_t p[5] = {1, 1, 1, 100, 0};
        CHECK(request(ESP_CMD_SET_TRIGGER, p, 5, pl, &n) == ESP_RSP_ACK && level_seen == 100, "level +100 ACK");
        p[3] = 0x9C; p[4] = 0xFF;                                /* -100 */
        CHECK(request(ESP_CMD_SET_TRIGGER, p, 5, pl, &n) == ESP_RSP_ACK && level_seen == -100, "level -100 ACK, sign kept");
        p[3] = 0xCE;                                             /* -50 */
        CHECK(request(ESP_CMD_SET_TRIGGER, p, 5, pl, &n) == ESP_RSP_ACK && level_seen == -50, "level -50 reaches the provider");
        CHECK(args_seen[0] == 1 && args_seen[1] == 1 && args_seen[2] == 1, "mode, edge, source reach the provider"); }
    {   uint8_t v[2] = {2, 9};
        CHECK(request(ESP_CMD_SET_VDIV, v, 2, pl, &n) == ESP_RSP_ACK && args_seen[0] == 2 && args_seen[1] == 9, "vdiv CH2 range 9 ACK"); }
    {   uint8_t c = ESP_TIMEBASE_CODES - 1;
        CHECK(request(ESP_CMD_SET_TIMEBASE, &c, 1, pl, &n) == ESP_RSP_ACK && args_seen[0] == c, "last timebase code ACK"); }
}

static void test_sets_reply_mapping(void)
{
    uint8_t pl[16]; uint16_t n = 0;
    uint8_t one = 1, tb = 0x10, v[2] = {1, 5}, tr[5] = {0, 0, 0, 20, 0};
    sets_reset();
    set_result = ESP_CTRL_NOT_READY;
    CHECK(nak_is(request(ESP_CMD_SET_TIMEBASE, &tb, 1, pl, &n), pl, ESP_ERR_NOT_READY), "NOT_READY -> NAK NOT_READY");
    CHECK(nak_is(request(ESP_CMD_RUN_STOP, &one, 1, pl, &n), pl, ESP_ERR_NOT_READY), "run_stop NOT_READY -> NAK");
    CHECK(nak_is(request(ESP_CMD_SET_MODE, &one, 1, pl, &n), pl, ESP_ERR_NOT_READY), "set_mode NOT_READY -> NAK");
    set_result = ESP_CTRL_WRONG_MODE;
    CHECK(nak_is(request(ESP_CMD_RUN_STOP, &one, 1, pl, &n), pl, ESP_ERR_UNSUPPORTED_IN_MODE),
          "WRONG_MODE -> NAK 0x0C");
    set_result = ESP_CTRL_OK;
    CHECK(request(ESP_CMD_SET_VDIV, v, 2, pl, &n) == ESP_RSP_ACK && n == 0, "OK -> ACK, no payload");
    CHECK(request(ESP_CMD_SET_TRIGGER, tr, 5, pl, &n) == ESP_RSP_ACK, "trigger OK -> ACK");
    esp_comm_set_control_provider(0);
    CHECK(nak_is(request(ESP_CMD_SET_TIMEBASE, &tb, 1, pl, &n), pl, ESP_ERR_UNSUPPORTED), "no provider: UNSUPPORTED");
}

static void test_scope_sets_need_scope_mode(void)
{
    uint8_t pl[16]; uint16_t n = 0;
    uint8_t meter = 1, scope = 0, tb = 0x14, v[2] = {1, 7}, tr[5] = {0, 0, 0, 30, 0}, run = 1;
    sets_reset();
    CHECK(request(ESP_CMD_SET_MODE, &meter, 1, pl, &n) == ESP_RSP_ACK && fake_mode == 1, "to meter mode");
    CHECK(nak_is(request(ESP_CMD_SET_TIMEBASE, &tb, 1, pl, &n), pl, ESP_ERR_UNSUPPORTED_IN_MODE), "meter: SET_TIMEBASE NAK 0x0C");
    CHECK(nak_is(request(ESP_CMD_SET_VDIV, v, 2, pl, &n), pl, ESP_ERR_UNSUPPORTED_IN_MODE), "meter: SET_VDIV NAK 0x0C");
    CHECK(nak_is(request(ESP_CMD_SET_TRIGGER, tr, 5, pl, &n), pl, ESP_ERR_UNSUPPORTED_IN_MODE), "meter: SET_TRIGGER NAK 0x0C");
    CHECK(hw_writes == 0 && trig_code_now == 128, "meter: no relay or register write");
    CHECK(request(ESP_CMD_RUN_STOP, &run, 1, pl, &n) == ESP_RSP_ACK, "meter: RUN_STOP stays open");
    for (uint8_t m = 2; m <= 3; m++) {
        fake_mode = m;
        CHECK(nak_is(request(ESP_CMD_SET_VDIV, v, 2, pl, &n), pl, ESP_ERR_UNSUPPORTED_IN_MODE), "siggen/settings: SET_VDIV NAK 0x0C");
    }
    CHECK(request(ESP_CMD_SET_MODE, &scope, 1, pl, &n) == ESP_RSP_ACK, "back to scope mode");
    CHECK(request(ESP_CMD_SET_VDIV, v, 2, pl, &n) == ESP_RSP_ACK && hw_writes == 1, "scope: SET_VDIV writes");
}

static void test_trigger_mode_change_does_not_write(void)
{
    uint8_t pl[16]; uint16_t n = 0;
    uint8_t tr[5] = {0, 0, 0, 0, 0};       /* level 0 = code 128 = in force */
    sets_reset();
    CHECK(request(ESP_CMD_SET_TRIGGER, tr, 5, pl, &n) == ESP_RSP_ACK && hw_writes == 0, "same code: ACK, no write");
    tr[0] = 1; tr[1] = 1;                  /* NORMAL, falling */
    CHECK(request(ESP_CMD_SET_TRIGGER, tr, 5, pl, &n) == ESP_RSP_ACK && hw_writes == 0, "mode/edge only: no write");
    set_result = ESP_CTRL_NOT_READY;       /* a park would fail now ... */
    tr[0] = 2;
    CHECK(request(ESP_CMD_SET_TRIGGER, tr, 5, pl, &n) == ESP_RSP_ACK, "... but no park is needed: ACK");
    tr[3] = 10;                            /* new code: needs the park */
    CHECK(nak_is(request(ESP_CMD_SET_TRIGGER, tr, 5, pl, &n), pl, ESP_ERR_NOT_READY), "new code, park fails: NAK NOT_READY");
    CHECK(trig_code_now == 128, "nothing written");
    set_result = ESP_CTRL_OK;
    CHECK(request(ESP_CMD_SET_TRIGGER, tr, 5, pl, &n) == ESP_RSP_ACK && hw_writes == 1 &&
          trig_code_now == code_from_level(10), "new code: one write");
    tr[2] = 1;                             /* CH2, same level */
    CHECK(request(ESP_CMD_SET_TRIGGER, tr, 5, pl, &n) == ESP_RSP_ACK && hw_writes == 2 && trig_src_now == 1,
          "source change: write");
}

static void test_scope_state_full_layout(void)
{
    uint8_t pl[64]; uint16_t n = 0; uint8_t junk = 0;
    reset();
    esp_comm_set_control_provider(&ctl_ops);
    fake_state.mode = 0; fake_state.running = true;
    fake_state.timebase_idx = 0x12; fake_state.timebase_code = 0x10; fake_state.timebase_tier = ESP_TIER_MEASURED;
    fake_state.sample_rate_hz = 12490;
    fake_state.trig_mode = 1; fake_state.trig_edge = 1; fake_state.trig_source = 1;
    fake_state.trig_level = -37; fake_state.trig_code = 82; fake_state.counts_per_div = 32;
    fake_state.ch[0].enabled = true; fake_state.ch[0].vdiv_idx = 6;
    fake_state.ch[0].vdiv_tier = ESP_TIER_MEASURED; fake_state.ch[0].uv_per_div = 0x01020304;
    fake_state.ch[1].enabled = false; fake_state.ch[1].vdiv_idx = 2;
    fake_state.ch[1].vdiv_tier = ESP_TIER_NONE; fake_state.ch[1].uv_per_div = 999;
    fake_state.ch[0].probe = 10; fake_state.ch[1].probe = 1;
    CHECK(request(ESP_CMD_GET_SCOPE_STATE, 0, 0, pl, &n) == ESP_RSP_SCOPE_STATE && n == 38, "SCOPE_STATE 38 B");
    CHECK(pl[2] == 0 && pl[3] == 1 && pl[4] == 0x12 && pl[5] == 0x10 && pl[6] == ESP_TIER_MEASURED,
          "mode, running, display timebase, code in force, tier");
    CHECK(pl[7] == 1 && pl[8] == 1 && pl[9] == 1, "trigger mode, edge, source");
    CHECK(pl[10] == 0xDB && pl[11] == 0xFF, "level -37 as i16 LE");
    CHECK(pl[12] == 0xCA && pl[13] == 0x30 && pl[14] == 0 && pl[15] == 0, "rate 12490 LE");
    CHECK(pl[16] == 82 && pl[17] == 0 && pl[18] == 32 && pl[19] == 0, "trig code, pad, counts/div");
    CHECK(pl[20] == 1 && pl[21] == 6 && pl[22] == ESP_TIER_MEASURED && pl[23] == 0 &&
          pl[24] == 4 && pl[25] == 3 && pl[26] == 2 && pl[27] == 1, "CH1 block");
    CHECK(pl[28] == 0 && pl[29] == 2 && pl[30] == ESP_TIER_NONE &&
          pl[32] == 0 && pl[33] == 0 && pl[34] == 0 && pl[35] == 0, "CH2 block: disowned gain zeroed");
    CHECK(pl[36] == 10 && pl[37] == 1, "probe bytes after the channel blocks");
    CHECK(nak_is(request(ESP_CMD_GET_SCOPE_STATE, &junk, 1, pl, &n), pl, ESP_ERR_BAD_LENGTH), "payload: BAD_LENGTH");
}

/* ─── stage-155: parser gaps (code-review-2026-11-23 §90) ─── */

static void test_payload_cap_256_accepted_257_refused(void)
{
    static uint8_t buf[600];
    uint8_t p[300], payload[257]; uint16_t pl = 0; size_t off = 0, n;
    for (int i = 0; i < 257; i++) payload[i] = (uint8_t)('a' + i % 26);
    reset();
    n = frame(buf, ESP_CMD_PING, payload, ESP_MAX_PAYLOAD);
    CHECK(ESP_MAX_PAYLOAD == 256, "host-to-device cap is 256 (protocol doc; O-8 is a user decision)");
    route(buf, n, 0);
    CHECK(take(&off, p, &pl) == ESP_RSP_DATA, "a 256-byte payload (the cap) is accepted and answered");
    CHECK(shell_len == 0, "no byte of the 256-byte frame reaches the shell");
    reset(); off = 0;
    n = frame(buf, ESP_CMD_PING, payload, 257);
    memcpy(buf + n, "ls\r", 3); n += 3;
    route(buf, n, 0);
    CHECK(take(&off, p, &pl) == ESP_RSP_NAK && p[0] == ESP_ERR_BAD_LENGTH,
          "a 257-byte payload (one over the cap) gets NAK(BAD_LENGTH)");
    CHECK(take(&off, p, &pl) == -1, "and nothing else: the 257-byte frame is not also answered");
    CHECK(strcmp(shell, "ls\r") == 0, "its payload is swallowed; the text after it reaches the shell");
}

static void test_sync_byte_inside_frame(void)
{
    static uint8_t buf[400];
    uint8_t p[300], payload[0xAA]; uint16_t pl = 0; size_t off = 0, n;
    const uint8_t aa[] = { 0xAA, 0xAA, 0x01, 0xAA };
    reset();
    route(buf, frame(buf, ESP_CMD_PING, aa, sizeof aa), 0);
    CHECK(take(&off, p, &pl) == ESP_RSP_DATA, "0xAA bytes inside a payload are data, not a new frame");
    CHECK(take(&off, p, &pl) == -1, "exactly one reply");
    CHECK(shell_len == 0, "nothing leaks to the shell");
    /* Length low byte 0xAA (170) and a payload of 0xAA only. */
    memset(payload, 0xAA, sizeof payload);
    reset(); off = 0;
    n = frame(buf, ESP_CMD_PING, payload, sizeof payload);
    CHECK(buf[3] == 0xAA, "the length byte itself is 0xAA");
    route(buf, n, 0);
    CHECK(take(&off, p, &pl) == ESP_RSP_DATA, "length 0xAA with an all-0xAA payload is one frame");
    CHECK(take(&off, p, &pl) == -1 && shell_len == 0, "one reply, no leak");
    esp_rx_stats_t st; esp_comm_get_rx_stats(&st);
    CHECK(st.frames_ok == 1 && st.bad_checksum == 0, "counted as one good frame, no checksum error");
}

static void test_two_frames_in_one_chunk(void)
{
    uint8_t buf[32], p[128]; uint16_t pl = 0; size_t off = 0, n;
    reset();
    n = frame(buf, ESP_CMD_STATUS, 0, 0);
    n += frame(buf + n, ESP_CMD_PING, 0, 0);
    route(buf, n, 0);
    CHECK(take(&off, p, &pl) == ESP_RSP_STATUS, "first frame of the chunk answered first");
    CHECK(take(&off, p, &pl) == ESP_RSP_DATA, "second frame of the same chunk answered second");
    CHECK(shell_len == 0, "no leak");
}

static void test_bad_checksum_then_good_frame(void)
{
    uint8_t buf[32], p[128]; uint16_t pl = 0; size_t off = 0, n;
    reset();
    n = frame(buf, ESP_CMD_STATUS, 0, 0);
    buf[n - 1] ^= 0x33;                          /* corrupt the first frame's checksum */
    n += frame(buf + n, ESP_CMD_PING, 0, 0);     /* a good frame right behind it */
    route(buf, n, 0);
    CHECK(take(&off, p, &pl) == ESP_RSP_NAK && p[0] == ESP_ERR_BAD_CHECKSUM,
          "the corrupt frame gets NAK(BAD_CHECKSUM)");
    CHECK(take(&off, p, &pl) == ESP_RSP_DATA, "the parser resyncs: the next frame is answered");
    CHECK(shell_len == 0, "no leak");
}

static void test_gap_timeout_across_tick_wrap(void)
{
    uint8_t buf[16], p[64]; size_t off = 0, n = frame(buf, ESP_CMD_PING, 0, 0);
    const uint32_t t0 = 0xFFFFFFF0u;
    reset();
    route(buf, 2, t0);
    CHECK(!esp_comm_rx_poll(t0 + 5), "5 ms later, before the wrap: frame still open");
    CHECK(!esp_comm_rx_poll(0x00000010u), "32 ms later, across the 32-bit wrap: frame still open");
    route(buf + 2, n - 2, 0x00000015u);
    CHECK(take(&off, p, 0) == ESP_RSP_DATA, "the frame completed across the wrap is answered");
    reset();
    route(buf, 2, t0);
    CHECK(!esp_comm_rx_poll(t0 + ESP_RX_GAP_MS - 1), "one ms short of the gap across the wrap: open");
    CHECK(esp_comm_rx_poll(t0 + ESP_RX_GAP_MS), "the full gap across the wrap: abandoned");
}

static int fail_on_call, fail_calls;
static bool failing_writer(const uint8_t *d, uint16_t n)
{
    fail_calls++;
    if (fail_calls == fail_on_call) return false;
    return block_writer(d, n);
}

static void test_write_failure_ends_the_frame(void)
{
    uint8_t buf[16];
    reset();
    fail_calls = 0; fail_on_call = 2;
    esp_comm_set_block_writer(failing_writer);
    route(buf, frame(buf, ESP_CMD_PING, 0, 0), 0);      /* reply = header, version, checksum */
    CHECK(fail_calls == 2, "a write that fails mid-frame ends it: no later write (stage-150 U-3)");
    CHECK(tx_len == ESP_HEADER_SIZE, "only the header went out");
    reset();
    fail_calls = 0; fail_on_call = 99;
    esp_comm_set_block_writer(failing_writer);
    route(buf, frame(buf, ESP_CMD_PING, 0, 0), 0);
    CHECK(fail_calls == 3, "control: a reply with a payload is three writes when none fails");
}

int main(void)
{
    printf("test_remote_proto\n");
    test_text_passes_through_untouched();
    test_frame_between_text_keeps_order();
    test_frame_split_across_usb_packets();
    test_status_v1_layout();
    test_status_reflects_live_state();
    test_ping_reports_bound_version();
    test_bad_checksum_is_answered();
    test_oversize_frame_is_swallowed();
    test_truncated_frame_times_out_and_shell_recovers();
    test_route_expires_stale_frame_itself();
    test_stubs_do_not_claim_success();
    test_unknown_command();
    test_button_injection();
    test_block_and_byte_writers_agree();
    test_get_meter();
    test_long_version_is_capped();
    test_oversize_then_silence_does_not_deafen_shell();
    test_touch_refreshes_open_frame_only();
    test_waveform_layout();
    test_waveform_both_channels_one_capture();
    test_waveform_refusals();
    test_waveform_lying_provider_is_refused();
    test_waveform_args();
    test_waveform_all_or_nothing();
    test_waveform_flags_are_derived();
    test_waveform_byte_writer_agrees();
    test_description_layout();
    test_description_identity_texts();
    test_description_refusals();
    test_set_probe();
    test_scope_state_probe_append();
    test_host_active_clock();
    test_acq_owner_two_requesters();
    test_acq_mode_gate_and_write_decision();
    test_sets_length_and_args();
    test_sets_reply_mapping();
    test_scope_sets_need_scope_mode();
    test_trigger_mode_change_does_not_write();
    test_scope_state_full_layout();
    test_payload_cap_256_accepted_257_refused();
    test_sync_byte_inside_frame();
    test_two_frames_in_one_chunk();
    test_bad_checksum_then_good_frame();
    test_gap_timeout_across_tick_wrap();
    test_write_failure_ends_the_frame();
    printf("%d/%d checks passed\n", checks - failures, checks);
    return failures ? 1 : 0;
}
