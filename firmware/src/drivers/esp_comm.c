/*
 * ESP32 Communication Protocol — GD32F307 side
 *
 * Receives commands from ESP32 over UART, dispatches to handlers,
 * sends responses. Handles module transfers and firmware staging.
 */

#include "esp_comm.h"
#include <string.h>

#ifndef ESP_COMM_TRANSFER_STUBS
#define ESP_COMM_TRANSFER_STUBS 0
#endif

/* ─── Receiver state machine ─── */

typedef enum {
    RX_WAIT_SYNC = 0,
    RX_WAIT_CMD,
    RX_WAIT_LEN_HI,
    RX_WAIT_LEN_LO,
    RX_WAIT_PAYLOAD,
    RX_WAIT_CHECKSUM,
} rx_state_t;

static rx_state_t rx_state = RX_WAIT_SYNC;
static esp_packet_t rx_packet;
static uint16_t rx_payload_idx = 0;
static uint8_t rx_checksum_acc = 0;

/* Oversize frame: swallow the bytes it announced (plus checksum) instead of
 * letting them fall through to whatever shares the stream. Bounded by the
 * inter-byte gap timeout, so a lying length cannot deafen the shell. */
static uint32_t rx_discard = 0;
static uint32_t rx_last_ms = 0;
static bool host_seen = false;         /* a valid frame arrived since init */
static uint32_t host_last_ms = 0;      /* ... at this esp_comm_route() time */
static esp_rx_stats_t rx_stats;

typedef enum {
    RX_NONE = 0,        /* byte consumed, nothing complete */
    RX_PACKET,          /* complete frame, checksum good */
    RX_BAD_CHECKSUM,    /* complete frame, checksum bad */
    RX_BAD_LENGTH,      /* header announced > ESP_MAX_PAYLOAD */
} rx_result_t;

#if ESP_COMM_TRANSFER_STUBS
/* ─── Transfer state ─── */

typedef struct {
    bool     active;
    bool     is_firmware;       /* true=firmware, false=module */
    uint8_t  slot;              /* module slot (0-3) */
    uint32_t total_size;
    uint32_t received;
    uint32_t flash_offset;      /* current write position in SPI flash */
} transfer_state_t;

static transfer_state_t transfer = {0};

/* Module slot metadata */
static module_slot_info_t modules[ESP_MODULE_SLOT_COUNT];
#endif /* ESP_COMM_TRANSFER_STUBS */

/* UART write function (set by caller) */
static esp_write_fn uart_write = 0;
static esp_write_block_fn block_write = 0;
static esp_status_fn status_provider = 0;
static esp_button_fn button_injector = 0;
static esp_meter_fn meter_provider = 0;
static esp_wave_fn wave_provider = 0;
static const esp_control_ops_t *control_ops = 0;

/* Reported only when no status provider is bound (host tests, bare ESP32
 * bring-up). The firmware binds the real build string. */
static const char fw_version[] = "0.0.0-unbound";

/* ─── Checksum ─── */

uint8_t esp_comm_checksum(const uint8_t *data, uint16_t len)
{
    uint8_t chk = 0;
    uint16_t i;
    if (!data) return 0;
    for (i = 0; i < len; i++)
        chk ^= data[i];
    return chk;
}

/* ─── Init ─── */

void esp_comm_init(void)
{
    rx_state = RX_WAIT_SYNC;
    rx_discard = 0;
    host_seen = false;
    memset(&rx_stats, 0, sizeof(rx_stats));
    memset(&rx_packet, 0, sizeof(rx_packet));
#if ESP_COMM_TRANSFER_STUBS
    memset(&transfer, 0, sizeof(transfer));
    memset(modules, 0, sizeof(modules));
#endif
}

void esp_comm_set_writer(esp_write_fn fn)
{
    uart_write = fn;
}

void esp_comm_set_block_writer(esp_write_block_fn fn)
{
    block_write = fn;
}

void esp_comm_set_status_provider(esp_status_fn fn)
{
    status_provider = fn;
}

void esp_comm_set_button_injector(esp_button_fn fn)
{
    button_injector = fn;
}

void esp_comm_set_meter_provider(esp_meter_fn fn)
{
    meter_provider = fn;
}

void esp_comm_set_waveform_provider(esp_wave_fn fn)
{
    wave_provider = fn;
}

void esp_comm_set_control_provider(const esp_control_ops_t *ops)
{
    control_ops = ops;
}

void esp_comm_get_rx_stats(esp_rx_stats_t *out)
{
    *out = rx_stats;
}

/* ─── Packet receiver (byte-at-a-time state machine) ─── */

static rx_result_t rx_step(uint8_t byte)
{
    if (rx_discard > 0) {
        rx_discard--;
        return RX_NONE;
    }

    switch (rx_state) {
    case RX_WAIT_SYNC:
        if (byte == ESP_SYNC_BYTE) {
            rx_state = RX_WAIT_CMD;
            rx_checksum_acc = 0;
        }
        return RX_NONE;

    case RX_WAIT_CMD:
        rx_packet.cmd = byte;
        rx_checksum_acc ^= byte;
        rx_state = RX_WAIT_LEN_HI;
        return RX_NONE;

    case RX_WAIT_LEN_HI:
        rx_packet.payload_len = (uint16_t)(byte << 8);
        rx_checksum_acc ^= byte;
        rx_state = RX_WAIT_LEN_LO;
        return RX_NONE;

    case RX_WAIT_LEN_LO:
        rx_packet.payload_len |= byte;
        rx_checksum_acc ^= byte;
        rx_payload_idx = 0;

        if (rx_packet.payload_len > ESP_MAX_PAYLOAD) {
            /* Receive-only cap (§2.2/§3.5): responses may be larger. Eat
             * the rest of this frame so it cannot leak into the shell. */
            rx_discard = (uint32_t)rx_packet.payload_len + ESP_CHECKSUM_SIZE;
            rx_state = RX_WAIT_SYNC;
            rx_stats.bad_length++;
            return RX_BAD_LENGTH;
        }
        rx_state = (rx_packet.payload_len == 0) ? RX_WAIT_CHECKSUM
                                                : RX_WAIT_PAYLOAD;
        return RX_NONE;

    case RX_WAIT_PAYLOAD:
        rx_packet.payload[rx_payload_idx++] = byte;
        rx_checksum_acc ^= byte;
        if (rx_payload_idx >= rx_packet.payload_len)
            rx_state = RX_WAIT_CHECKSUM;
        return RX_NONE;

    case RX_WAIT_CHECKSUM:
        rx_state = RX_WAIT_SYNC;
        rx_packet.valid = (byte == rx_checksum_acc);
        if (rx_packet.valid) {
            rx_stats.frames_ok++;
            return RX_PACKET;
        }
        rx_stats.bad_checksum++;
        return RX_BAD_CHECKSUM;
    }

    rx_state = RX_WAIT_SYNC;
    return RX_NONE;
}

bool esp_comm_receive_byte(uint8_t byte)
{
    return rx_step(byte) == RX_PACKET;
}

bool esp_comm_rx_in_frame(void)
{
    return rx_state != RX_WAIT_SYNC || rx_discard > 0;
}

void esp_comm_rx_touch(uint32_t now_ms)
{
    if (esp_comm_rx_in_frame())
        rx_last_ms = now_ms;
}

bool esp_comm_rx_poll(uint32_t now_ms)
{
    if (!esp_comm_rx_in_frame())
        return false;
    if ((uint32_t)(now_ms - rx_last_ms) < ESP_RX_GAP_MS)
        return false;
    rx_state = RX_WAIT_SYNC;
    rx_discard = 0;
    rx_stats.gap_timeouts++;
    /* Deliberately silent. The protocol has no request ids, so a reply
     * nobody is waiting for would be taken by the host as the answer to its
     * NEXT request. The host that abandoned this frame has already timed
     * out; the counter (usbstat) is the record. */
    return true;
}

void esp_comm_route(const uint8_t *data, uint16_t len, uint32_t now_ms,
                    esp_passthrough_fn passthrough, void *ctx)
{
    uint16_t run_start = 0;
    uint16_t i;

    (void)esp_comm_rx_poll(now_ms);

    for (i = 0; i < len; i++) {
        uint8_t b = data[i];

        if (!esp_comm_rx_in_frame() && b != ESP_SYNC_BYTE)
            continue;                       /* belongs to the passthrough run */

        /* Flush the text that preceded this frame byte, in order. */
        if (i > run_start && passthrough)
            passthrough(data + run_start, (uint16_t)(i - run_start), ctx);
        run_start = (uint16_t)(i + 1);

        rx_last_ms = now_ms;
        switch (rx_step(b)) {
        case RX_PACKET:
            host_seen = true;               /* the remote badge clock (stage-086) */
            host_last_ms = now_ms;
            esp_comm_process(&rx_packet);
            break;
        case RX_BAD_CHECKSUM: esp_comm_send_nak(ESP_ERR_BAD_CHECKSUM); break;
        case RX_BAD_LENGTH:   esp_comm_send_nak(ESP_ERR_BAD_LENGTH); break;
        case RX_NONE:         break;
        }
    }
    if (len > run_start && passthrough)
        passthrough(data + run_start, (uint16_t)(len - run_start), ctx);
}

bool esp_comm_host_active(uint32_t now_ms)
{
    return host_seen && (uint32_t)(now_ms - host_last_ms) < ESP_HOST_ACTIVE_MS;
}

const esp_packet_t *esp_comm_get_packet(void)
{
    return &rx_packet;
}

/* ─── Packet sender ─── */

/* One frame whose payload is two parts sent back to back (a header built on
 * the stack and samples that live elsewhere), so a 1 KB record is never
 * copied a second time just to sit behind its header. RAM is ~1.5 KB from
 * full: there is no room for a contiguous 1048-byte response buffer. */
static void send_frame2(uint8_t cmd, const uint8_t *a, uint16_t alen,
                        const uint8_t *b, uint16_t blen)
{
    uint16_t len = (uint16_t)(alen + blen);
    uint8_t hdr[ESP_HEADER_SIZE] = {
        ESP_SYNC_BYTE, cmd, (uint8_t)(len >> 8), (uint8_t)(len & 0xFF)
    };
    uint8_t chk = (uint8_t)(cmd ^ hdr[2] ^ hdr[3]) ^ esp_comm_checksum(a, alen)
                                                  ^ esp_comm_checksum(b, blen);

    if (block_write) {
        /* One task writes the whole frame: nothing else can interleave.
         * Stop at the first part that did not go out: the host is not
         * reading, and the rest of the frame would only wait again
         * (stage-150, U-3). The host drops a truncated frame. */
        if (!block_write(hdr, ESP_HEADER_SIZE))
            return;
        if (alen && !block_write(a, alen))
            return;
        if (blen && !block_write(b, blen))
            return;
        (void)block_write(&chk, ESP_CHECKSUM_SIZE);
        return;
    }
    if (!uart_write) return;

    uint16_t i;
    for (i = 0; i < ESP_HEADER_SIZE; i++)
        uart_write(hdr[i]);
    for (i = 0; i < alen; i++)
        uart_write(a[i]);
    for (i = 0; i < blen; i++)
        uart_write(b[i]);
    uart_write(chk);
}

void esp_comm_send_response(uint8_t cmd, const uint8_t *payload, uint16_t len)
{
    send_frame2(cmd, payload, len, 0, 0);
}

void esp_comm_send_ack(void)
{
    esp_comm_send_response(ESP_RSP_ACK, 0, 0);
}

void esp_comm_send_nak(uint8_t error_code)
{
    esp_comm_send_response(ESP_RSP_NAK, &error_code, 1);
}

bool esp_comm_transfer_active(void)
{
#if ESP_COMM_TRANSFER_STUBS
    return transfer.active;
#else
    return false;   /* staging is not compiled in: RAM is ~1.5 KB from full */
#endif
}

/* ─── Command handlers ─── */

static void snapshot(esp_status_snapshot_t *st)
{
    memset(st, 0, sizeof(*st));
    st->fw_version = fw_version;
    if (status_provider)
        status_provider(st);
    if (!st->fw_version)
        st->fw_version = fw_version;
}

static uint8_t fw_version_len(const char *v)
{
    size_t n = strlen(v);
    return (uint8_t)(n > ESP_FW_VERSION_MAX ? ESP_FW_VERSION_MAX : n);
}

static void put_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

static void handle_ping(const esp_packet_t *pkt)
{
    esp_status_snapshot_t st;
    (void)pkt;
    snapshot(&st);
    esp_comm_send_response(ESP_RSP_DATA, (const uint8_t *)st.fw_version,
                           fw_version_len(st.fw_version));
}

/* STATUS v1 — layout documented in esp_comm.h (ESP_STATUS_FIXED_LEN). */
static void handle_status(const esp_packet_t *pkt)
{
    esp_status_snapshot_t st;
    uint8_t out[ESP_STATUS_FIXED_LEN + ESP_FW_VERSION_MAX];
    uint8_t n;
    (void)pkt;

    snapshot(&st);
    n = fw_version_len(st.fw_version);
    out[0] = ESP_PROTO_VERSION;
    out[1] = st.current_mode;
    out[2] = st.battery_pct;
    out[3] = st.flags;
    put_u16(&out[4], st.battery_mv);
    put_u32(&out[6], st.uptime_ms);
    put_u32(&out[10], st.usb_tx_stalls);
    put_u16(&out[14], st.usb_heals);
    out[16] = n;
    memcpy(&out[ESP_STATUS_FIXED_LEN], st.fw_version, n);
    esp_comm_send_response(ESP_RSP_STATUS, out, (uint16_t)(ESP_STATUS_FIXED_LEN + n));
}

static void handle_button(const esp_packet_t *pkt)
{
    if (pkt->payload_len != 1) {
        esp_comm_send_nak(ESP_ERR_BAD_LENGTH);
        return;
    }
    if (pkt->payload[0] < ESP_BTN_CH1 || pkt->payload[0] > ESP_BTN_POWER) {
        esp_comm_send_nak(ESP_ERR_BAD_ARG);
        return;
    }
    if (!button_injector) {
        esp_comm_send_nak(ESP_ERR_UNSUPPORTED);
        return;
    }
    /* ACK only once the press is really queued (§3.7: never report an
     * action that did not happen). */
    if (!button_injector(pkt->payload[0])) {
        esp_comm_send_nak(ESP_ERR_NOT_READY);
        return;
    }
    esp_comm_send_ack();
}

static uint8_t put_str(uint8_t *p, const char *str)
{
    size_t n = str ? strlen(str) : 0;
    if (n > 15) n = 15;
    p[0] = (uint8_t)n;
    if (n) memcpy(p + 1, str, n);
    return (uint8_t)(n + 1);
}

/* METER_FRAME v1 — layout in esp_comm.h (ESP_METER_FIXED_LEN). */
static void handle_get_meter(const esp_packet_t *pkt)
{
    esp_meter_snapshot_t m;
    uint8_t out[ESP_METER_FIXED_LEN + 32];
    uint16_t n = ESP_METER_FIXED_LEN - 1;   /* index of unit_len */
    uint32_t vbits;

    if (pkt->payload_len != 0) {
        esp_comm_send_nak(ESP_ERR_BAD_LENGTH);
        return;
    }
    if (!meter_provider) {
        esp_comm_send_nak(ESP_ERR_UNSUPPORTED);
        return;
    }
    memset(&m, 0, sizeof(m));
    switch (meter_provider(&m)) {
    case ESP_METER_OK:
        break;
    case ESP_METER_WRONG_MODE:
        esp_comm_send_nak(ESP_ERR_UNSUPPORTED_IN_MODE);   /* frozen, not live */
        return;
    default:
        esp_comm_send_nak(ESP_ERR_NOT_READY);             /* no reading yet: say so */
        return;
    }
    memcpy(&vbits, &m.value, sizeof(vbits));
    put_u32(&out[0], m.update_count);
    put_u32(&out[4], vbits);
    put_u16(&out[8], (uint16_t)m.raw_bcd);
    out[10] = m.decimal_pos;
    out[11] = m.result_class;
    out[12] = m.flags;
    out[13] = m.submode;
    out[14] = m.unit_variant;
    n = (uint16_t)(n + put_str(&out[n], m.unit));
    n = (uint16_t)(n + put_str(&out[n], m.display));
    esp_comm_send_response(ESP_RSP_METER_FRAME, out, n);
}

/* A measured number goes on the wire only with the confidence that makes it
 * one: MEASURED or PROVISIONAL, and non-zero. A value the tier disowns (NONE)
 * is zeroed rather than sent bare — derive the guard, do not trust the
 * provider to keep the number and its tier in agreement (scope_cal.c). */
static uint8_t tier_flag(uint8_t tier, uint32_t value, uint8_t measured, uint8_t provisional)
{
    if (value == 0)
        return 0;
    if (tier == ESP_TIER_MEASURED)
        return measured;
    if (tier == ESP_TIER_PROVISIONAL)
        return provisional;
    return 0;
}

static bool wave_channel_ok(const esp_wave_channel_t *ch)
{
    return ch->samples != 0 && ch->count != 0 && ch->count <= ESP_WAVE_MAX_SAMPLES;
}

static void send_wave_channel(const esp_wave_snapshot_t *w, uint8_t c)
{
    const esp_wave_channel_t *ch = &w->ch[c];
    uint8_t hdr[ESP_WAVE_HDR_LEN];
    uint8_t f = 0;      /* bit0 calibrated stays clear: no per-unit cal exists (§3.5) */
    uint8_t tb, vd;

    tb = w->timebase_disagrees ? 0      /* the rate belongs to a code not in force */
                               : tier_flag(w->timebase_tier, w->sample_rate_hz,
                                           ESP_WAVE_FLAG_TB_MEASURED,
                                           ESP_WAVE_FLAG_TB_PROVISIONAL);
    vd = tier_flag(ch->vdiv_tier, ch->uv_per_div,
                   ESP_WAVE_FLAG_VDIV_MEASURED, ESP_WAVE_FLAG_VDIV_PROVISIONAL);
    f |= tb | vd;
    if (w->time_ordered)       f |= ESP_WAVE_FLAG_TIME_ORDERED;
    if (w->timebase_disagrees) f |= ESP_WAVE_FLAG_TB_DISAGREES;

    put_u32(&hdr[0], w->frame_id);
    hdr[4] = c;
    hdr[5] = f;
    hdr[6] = w->timebase_idx;
    hdr[7] = ch->vdiv_idx;
    put_u16(&hdr[8], ch->count);
    put_u16(&hdr[10], ESP_WAVE_HDR_LEN);
    put_u32(&hdr[12], tb ? w->sample_rate_hz : 0);
    put_u32(&hdr[16], vd ? ch->uv_per_div : 0);
    put_u16(&hdr[20], w->counts_per_div);
    put_u16(&hdr[22], w->head_skip);
    send_frame2(ESP_RSP_WAVEFORM_FRAME, hdr, ESP_WAVE_HDR_LEN, ch->samples, ch->count);
}

/* WAVEFORM_FRAME v1 — layout in esp_comm.h (ESP_WAVE_HDR_LEN). */
static void handle_get_waveform(const esp_packet_t *pkt)
{
    esp_wave_snapshot_t w;
    uint8_t mask, c;

    if (pkt->payload_len != 1) {
        esp_comm_send_nak(ESP_ERR_BAD_LENGTH);
        return;
    }
    mask = pkt->payload[0];
    if (mask == 0 || (mask & (uint8_t)~ESP_WAVE_MASK_ALL) != 0) {
        esp_comm_send_nak(ESP_ERR_BAD_ARG);
        return;
    }
    if (!wave_provider) {
        esp_comm_send_nak(ESP_ERR_UNSUPPORTED);
        return;
    }
    memset(&w, 0, sizeof(w));
    switch (wave_provider(mask, &w)) {
    case ESP_WAVE_OK:
        break;
    case ESP_WAVE_WRONG_MODE:
        esp_comm_send_nak(ESP_ERR_UNSUPPORTED_IN_MODE);   /* buffers not live outside scope mode */
        return;
    case ESP_WAVE_BUSY:
        esp_comm_send_nak(ESP_ERR_NOT_READY);             /* no tear-free copy: retry */
        return;
    default:
        esp_comm_send_nak(ESP_ERR_NO_CAPTURE_DATA);       /* no capture yet: never the demo trace */
        return;
    }
    /* §2.3: the demo trace (or anything else that is not a capture) is
     * refused, not sent with a flag the host might ignore. A provider that
     * says OK and hands over a synthetic record is caught here. */
    if (w.synthetic || w.frame_id == 0) {
        esp_comm_send_nak(ESP_ERR_NO_CAPTURE_DATA);       /* refused whole: not a real record */
        return;
    }
    /* All or nothing: check every requested channel before the first byte
     * goes out, so a host never receives CH1 followed by a refusal. */
    for (c = 0; c < 2; c++) {
        if ((mask & (1u << c)) && !wave_channel_ok(&w.ch[c])) {
            esp_comm_send_nak(ESP_ERR_NO_CAPTURE_DATA);   /* a channel without a record */
            return;
        }
    }
    for (c = 0; c < 2; c++) {
        if (mask & (1u << c))
            send_wave_channel(&w, c);
    }
}

/* ─── Control surface (M6) — layouts and reply rules in esp_comm.h ─── */

static void put_i16(uint8_t *p, int16_t v) { put_u16(p, (uint16_t)v); }
static int16_t get_i16(const uint8_t *p) { return (int16_t)(uint16_t)(p[0] | (p[1] << 8)); }

/* Shared tail of every set: map the provider's answer onto the wire. */
static void ctrl_reply(esp_ctrl_result_t r)
{
    switch (r) {
    case ESP_CTRL_OK:         esp_comm_send_ack(); break;
    case ESP_CTRL_WRONG_MODE: esp_comm_send_nak(ESP_ERR_UNSUPPORTED_IN_MODE); break;
    default:                  esp_comm_send_nak(ESP_ERR_NOT_READY); break;   /* nothing changed */
    }
}

/* Length first, then the provider's presence, then every field: a refusal
 * names the first thing wrong with the request itself. */
static bool ctrl_len_ok(const esp_packet_t *pkt, uint16_t want, bool bound)
{
    if (pkt->payload_len != want) {
        esp_comm_send_nak(ESP_ERR_BAD_LENGTH);
        return false;
    }
    if (!bound) {
        esp_comm_send_nak(ESP_ERR_UNSUPPORTED);
        return false;
    }
    return true;
}

static void put_scope_channel(uint8_t *p, const esp_scope_channel_t *c)
{
    /* Same guard as the waveform header: a gain the tier disowns is zeroed. */
    const bool usable = tier_flag(c->vdiv_tier, c->uv_per_div, 1, 1) != 0;
    p[0] = c->enabled ? 1 : 0;
    p[1] = c->vdiv_idx;
    p[2] = usable ? c->vdiv_tier : ESP_TIER_NONE;
    p[3] = 0;
    put_u32(&p[4], usable ? c->uv_per_div : 0);
}

/* 1 or 10 on the wire; anything else from a provider is not a probe setting
 * this link knows, and 1 is the factor that leaves uv_per_div unscaled. */
static uint8_t wire_probe(uint8_t factor)
{
    return (factor == 10u) ? 10u : 1u;
}

static void handle_get_scope_state(const esp_packet_t *pkt)
{
    esp_scope_state_t s;
    uint8_t out[ESP_SCOPE_STATE_LEN];
    bool rate_ok;

    if (!ctrl_len_ok(pkt, 0, control_ops && control_ops->get_state))
        return;
    memset(&s, 0, sizeof(s));
    control_ops->get_state(&s);
    rate_ok = tier_flag(s.timebase_tier, s.sample_rate_hz, 1, 1) != 0;
    memset(out, 0, sizeof(out));
    out[0] = ESP_SCOPE_STATE_VERSION;
    out[1] = ESP_SCOPE_STATE_LEN;
    out[2] = s.mode;
    out[3] = s.running ? 1 : 0;
    out[4] = s.timebase_idx;
    out[5] = s.timebase_code;
    out[6] = rate_ok ? s.timebase_tier : ESP_TIER_NONE;
    out[7] = s.trig_mode;
    out[8] = s.trig_edge;
    out[9] = s.trig_source;
    put_i16(&out[10], s.trig_level);
    put_u32(&out[12], rate_ok ? s.sample_rate_hz : 0);
    out[16] = s.trig_code;
    put_u16(&out[18], s.counts_per_div);
    put_scope_channel(&out[20], &s.ch[0]);
    put_scope_channel(&out[28], &s.ch[1]);
    out[36] = wire_probe(s.ch[0].probe);        /* stage-086 append */
    out[37] = wire_probe(s.ch[1].probe);
    esp_comm_send_response(ESP_RSP_SCOPE_STATE, out, ESP_SCOPE_STATE_LEN);
}

/* ─── Device description (stage-086) — layout in esp_comm.h ─── */

/* One (value, tier) pair with the link's guard: a value its tier disowns is
 * zeroed, and a zero value is never sent with a tier that vouches for it. */
static uint8_t *put_tiered(uint8_t *p, uint32_t value, uint8_t tier)
{
    const bool usable = tier_flag(tier, value, 1, 1) != 0;
    put_u32(p, usable ? value : 0);
    p[4] = usable ? tier : ESP_TIER_NONE;
    return p + 5;
}

static uint8_t *put_section(uint8_t *p, uint8_t id, uint16_t len)
{
    p[0] = id;
    put_u16(&p[1], len);
    return p + 3;
}

/* u8 len, then the text without its NUL, cut to ESP_DESC_TEXT_MAX. */
static uint8_t *put_text(uint8_t *p, const char *s)
{
    uint8_t n = 0;
    if (s)
        while (n < ESP_DESC_TEXT_MAX && s[n] != '\0')
            n++;
    *p++ = n;
    if (n)
        memcpy(p, s, n);
    return p + n;
}

static void handle_get_description(const esp_packet_t *pkt)
{
    esp_description_t d;
    /* static: 320 B off the usb_dbg task stack; only that task dispatches
     * frames, and the reply is sent before the next one is parsed. */
    static uint8_t out[ESP_DESCRIPTION_LEN];
    uint8_t *p = out;
    uint8_t *ident;
    unsigned i, c;

    if (!ctrl_len_ok(pkt, 0, control_ops && control_ops->get_description))
        return;
    memset(&d, 0, sizeof(d));
    control_ops->get_description(&d);

    *p++ = ESP_DESCRIPTION_VERSION;

    p = put_section(p, ESP_DESC_SEC_TIMEBASE, ESP_DESC_TB_LEN);
    put_u16(p, d.samples_per_div); p += 2;
    *p++ = ESP_TIMEBASE_CODES;
    for (i = 0; i < ESP_TIMEBASE_CODES; i++)
        p = put_tiered(p, d.tb_rate_hz[i], d.tb_tier[i]);

    p = put_section(p, ESP_DESC_SEC_RANGES, ESP_DESC_RANGES_LEN);
    put_u16(p, d.counts_per_div); p += 2;
    *p++ = ESP_VDIV_RANGES;
    *p++ = 2;
    for (c = 0; c < 2; c++)
        for (i = 0; i < ESP_VDIV_RANGES; i++)
            p = put_tiered(p, d.uv_per_div_bnc[c][i], d.vdiv_tier[c][i]);

    p = put_section(p, ESP_DESC_SEC_TRIGGER, ESP_DESC_TRIGGER_LEN);
    put_i16(p, d.trig_level_min); p += 2;
    put_i16(p, d.trig_level_max); p += 2;
    *p++ = d.trig_center_code;
    put_u16(p, d.trig_num); p += 2;
    put_u16(p, d.trig_den); p += 2;
    *p++ = d.trig_code_min;
    *p++ = d.trig_code_max;
    *p++ = (uint8_t)d.trig_record_offset;

    p = put_section(p, ESP_DESC_SEC_PROBE, ESP_DESC_PROBE_LEN);
    *p++ = wire_probe(d.probe[0]);
    *p++ = wire_probe(d.probe[1]);

    /* stage-190: identity. The length goes in after the texts are cut. */
    ident = p;
    p = put_section(p, ESP_DESC_SEC_IDENTITY, 0);
    memcpy(p, d.uid, ESP_DESC_UID_LEN); p += ESP_DESC_UID_LEN;
    put_u16(p, ESP_API_LEVEL); p += 2;
    p = put_text(p, d.build_target);
    p = put_text(p, d.git_rev);
    put_u16(&ident[1], (uint16_t)(p - ident - 3));

    esp_comm_send_response(ESP_RSP_DESCRIPTION, out, (uint16_t)(p - out));
}

static void handle_set_probe(const esp_packet_t *pkt)
{
    if (!ctrl_len_ok(pkt, 2, control_ops && control_ops->set_probe))
        return;
    if ((pkt->payload[0] != 1 && pkt->payload[0] != 2) ||
        (pkt->payload[1] != 1 && pkt->payload[1] != 10)) {
        esp_comm_send_nak(ESP_ERR_BAD_ARG);
        return;
    }
    ctrl_reply(control_ops->set_probe(pkt->payload[0], pkt->payload[1]));
}

static void handle_set_mode(const esp_packet_t *pkt)
{
    if (!ctrl_len_ok(pkt, 1, control_ops && control_ops->set_mode))
        return;
    if (pkt->payload[0] > 3) {
        esp_comm_send_nak(ESP_ERR_BAD_ARG);
        return;
    }
    ctrl_reply(control_ops->set_mode(pkt->payload[0]));
}

static void handle_set_timebase(const esp_packet_t *pkt)
{
    if (!ctrl_len_ok(pkt, 1, control_ops && control_ops->set_timebase))
        return;
    if (pkt->payload[0] >= ESP_TIMEBASE_CODES) {
        esp_comm_send_nak(ESP_ERR_BAD_ARG);
        return;
    }
    ctrl_reply(control_ops->set_timebase(pkt->payload[0]));
}

static void handle_set_vdiv(const esp_packet_t *pkt)
{
    if (!ctrl_len_ok(pkt, 2, control_ops && control_ops->set_vdiv))
        return;
    if ((pkt->payload[0] != 1 && pkt->payload[0] != 2) || pkt->payload[1] >= ESP_VDIV_RANGES) {
        esp_comm_send_nak(ESP_ERR_BAD_ARG);
        return;
    }
    ctrl_reply(control_ops->set_vdiv(pkt->payload[0], pkt->payload[1]));
}

static void handle_set_trigger(const esp_packet_t *pkt)
{
    int16_t level;
    if (!ctrl_len_ok(pkt, 5, control_ops && control_ops->set_trigger))
        return;
    level = get_i16(&pkt->payload[3]);
    if (pkt->payload[0] > 2 || pkt->payload[1] > 1 || pkt->payload[2] > 1 ||
        level < -ESP_TRIG_LEVEL_MAX || level > ESP_TRIG_LEVEL_MAX) {
        esp_comm_send_nak(ESP_ERR_BAD_ARG);       /* one bad field: nothing changes */
        return;
    }
    ctrl_reply(control_ops->set_trigger(pkt->payload[0], pkt->payload[1], pkt->payload[2], level));
}

static void handle_run_stop(const esp_packet_t *pkt)
{
    if (!ctrl_len_ok(pkt, 1, control_ops && control_ops->run_stop))
        return;
    if (pkt->payload[0] > 1) {
        esp_comm_send_nak(ESP_ERR_BAD_ARG);
        return;
    }
    ctrl_reply(control_ops->run_stop(pkt->payload[0] != 0));
}

#if ESP_COMM_TRANSFER_STUBS
static void handle_module_start(const esp_packet_t *pkt)
{
    if (transfer.active) {
        esp_comm_send_nak(ESP_ERR_TRANSFER_ACTIVE);
        return;
    }
    if (pkt->payload_len < 5) {
        esp_comm_send_nak(ESP_ERR_BAD_LENGTH);
        return;
    }

    uint8_t slot = pkt->payload[0];
    uint32_t size = ((uint32_t)pkt->payload[1] << 24) |
                    ((uint32_t)pkt->payload[2] << 16) |
                    ((uint32_t)pkt->payload[3] << 8) |
                    (uint32_t)pkt->payload[4];

    if (slot >= ESP_MODULE_SLOT_COUNT) {
        esp_comm_send_nak(ESP_ERR_INVALID_SLOT);
        return;
    }
    if (size > ESP_MODULE_MAX_SIZE) {
        esp_comm_send_nak(ESP_ERR_FLASH_FULL);
        return;
    }

    transfer.active = true;
    transfer.is_firmware = false;
    transfer.slot = slot;
    transfer.total_size = size;
    transfer.received = 0;
    /* SPI flash offset: slot 0 at 0x100000, slot 1 at 0x200000, etc. */
    transfer.flash_offset = (uint32_t)(slot + 1) * 0x100000;

    /* TODO: erase SPI flash sector for this slot */

    esp_comm_send_ack();
}

static void handle_module_data(const esp_packet_t *pkt)
{
    if (!transfer.active || transfer.is_firmware) {
        esp_comm_send_nak(ESP_ERR_NOT_READY);
        return;
    }

    uint32_t remaining = transfer.total_size - transfer.received;
    uint16_t chunk = pkt->payload_len;
    if (chunk > remaining)
        chunk = (uint16_t)remaining;

    /* TODO: write pkt->payload[0..chunk-1] to SPI flash at
     * transfer.flash_offset + transfer.received */

    transfer.received += chunk;
    esp_comm_send_ack();
}

static void handle_module_end(const esp_packet_t *pkt)
{
    (void)pkt;
    if (!transfer.active || transfer.is_firmware) {
        esp_comm_send_nak(ESP_ERR_NOT_READY);
        return;
    }

    /* Mark module as installed */
    modules[transfer.slot].installed = true;
    modules[transfer.slot].size = transfer.received;

    /* Copy name from first bytes of module if available */
    strncpy(modules[transfer.slot].name, "Module",
            sizeof(modules[transfer.slot].name) - 1);
    strncpy(modules[transfer.slot].version, "1.0",
            sizeof(modules[transfer.slot].version) - 1);

    transfer.active = false;

    /* TODO: write module metadata to SPI flash index */

    esp_comm_send_ack();
}

static void handle_module_list(const esp_packet_t *pkt)
{
    (void)pkt;
    /* Send module slot info for all slots */
    esp_comm_send_response(ESP_RSP_MODULE_LIST,
                           (const uint8_t *)modules,
                           sizeof(modules));
}

static void handle_module_delete(const esp_packet_t *pkt)
{
    if (pkt->payload_len < 1) {
        esp_comm_send_nak(ESP_ERR_BAD_LENGTH);
        return;
    }
    uint8_t slot = pkt->payload[0];
    if (slot >= ESP_MODULE_SLOT_COUNT) {
        esp_comm_send_nak(ESP_ERR_INVALID_SLOT);
        return;
    }

    /* TODO: erase SPI flash for this slot */
    memset(&modules[slot], 0, sizeof(module_slot_info_t));
    esp_comm_send_ack();
}

static void handle_fw_update_start(const esp_packet_t *pkt)
{
    if (transfer.active) {
        esp_comm_send_nak(ESP_ERR_TRANSFER_ACTIVE);
        return;
    }
    if (pkt->payload_len < 4) {
        esp_comm_send_nak(ESP_ERR_BAD_LENGTH);
        return;
    }

    uint32_t size = ((uint32_t)pkt->payload[0] << 24) |
                    ((uint32_t)pkt->payload[1] << 16) |
                    ((uint32_t)pkt->payload[2] << 8) |
                    (uint32_t)pkt->payload[3];

    transfer.active = true;
    transfer.is_firmware = true;
    transfer.total_size = size;
    transfer.received = 0;
    /* Stage firmware in upper half of flash: 0x08080000 */
    transfer.flash_offset = 0x08080000;

    /* TODO: erase staging flash area */

    esp_comm_send_ack();
}

static void handle_fw_update_data(const esp_packet_t *pkt)
{
    if (!transfer.active || !transfer.is_firmware) {
        esp_comm_send_nak(ESP_ERR_NOT_READY);
        return;
    }

    uint32_t remaining = transfer.total_size - transfer.received;
    uint16_t chunk = pkt->payload_len;
    if (chunk > remaining)
        chunk = (uint16_t)remaining;

    /* TODO: write to staging flash at transfer.flash_offset + transfer.received */

    transfer.received += chunk;
    esp_comm_send_ack();
}

static void handle_fw_update_commit(const esp_packet_t *pkt)
{
    (void)pkt;
    if (!transfer.active || !transfer.is_firmware) {
        esp_comm_send_nak(ESP_ERR_NOT_READY);
        return;
    }

    if (transfer.received < transfer.total_size) {
        esp_comm_send_nak(ESP_ERR_BAD_LENGTH);
        return;
    }

    /* TODO: Write "update pending" flag to a known flash location.
     * The bootloader checks this flag on boot and copies
     * staged firmware to the active slot. */

    transfer.active = false;
    esp_comm_send_ack();

    /* TODO: NVIC_SystemReset() to reboot into bootloader */
}

#endif /* ESP_COMM_TRANSFER_STUBS */

/* ─── Command dispatcher ─── */

void esp_comm_process(const esp_packet_t *pkt)
{
    if (!pkt->valid) {
        esp_comm_send_nak(ESP_ERR_BAD_CHECKSUM);
        return;
    }

    switch (pkt->cmd) {
    case ESP_CMD_PING:              handle_ping(pkt); break;
    case ESP_CMD_STATUS:            handle_status(pkt); break;
    case ESP_CMD_BUTTON:            handle_button(pkt); break;
    case ESP_CMD_GET_METER:         handle_get_meter(pkt); break;
    case ESP_CMD_GET_WAVEFORM:      handle_get_waveform(pkt); break;
    case ESP_CMD_GET_SCOPE_STATE:   handle_get_scope_state(pkt); break;
    case ESP_CMD_SET_MODE:          handle_set_mode(pkt); break;
    case ESP_CMD_SET_TIMEBASE:      handle_set_timebase(pkt); break;
    case ESP_CMD_SET_VDIV:          handle_set_vdiv(pkt); break;
    case ESP_CMD_SET_TRIGGER:       handle_set_trigger(pkt); break;
    case ESP_CMD_RUN_STOP:          handle_run_stop(pkt); break;
    case ESP_CMD_GET_DESCRIPTION:   handle_get_description(pkt); break;
    case ESP_CMD_SET_PROBE:         handle_set_probe(pkt); break;
#if ESP_COMM_TRANSFER_STUBS
    /* ESP32 co-processor staging. The flash writes are still TODO, so these
     * are compiled only where that is understood (the legacy flow tests). */
    case ESP_CMD_MODULE_START:      handle_module_start(pkt); break;
    case ESP_CMD_MODULE_DATA:       handle_module_data(pkt); break;
    case ESP_CMD_MODULE_END:        handle_module_end(pkt); break;
    case ESP_CMD_MODULE_LIST:       handle_module_list(pkt); break;
    case ESP_CMD_MODULE_DELETE:     handle_module_delete(pkt); break;
    case ESP_CMD_FW_UPDATE_START:   handle_fw_update_start(pkt); break;
    case ESP_CMD_FW_UPDATE_DATA:    handle_fw_update_data(pkt); break;
    case ESP_CMD_FW_UPDATE_COMMIT:  handle_fw_update_commit(pkt); break;
#else
    /* Known commands whose handlers would have ACKed data they then dropped
     * (flash writes are TODO). A success report for an action that never
     * happened is the failure mode this project keeps paying for, so they
     * answer UNSUPPORTED until they are real. */
    case ESP_CMD_MODULE_START:
    case ESP_CMD_MODULE_DATA:
    case ESP_CMD_MODULE_END:
    case ESP_CMD_MODULE_LIST:
    case ESP_CMD_MODULE_DELETE:
    case ESP_CMD_FW_UPDATE_START:
    case ESP_CMD_FW_UPDATE_DATA:
    case ESP_CMD_FW_UPDATE_COMMIT:
#endif
    case ESP_CMD_FRAMEBUFFER:
    case ESP_CMD_SIGNAL_CONFIG:
        esp_comm_send_nak(ESP_ERR_UNSUPPORTED);
        break;

    default:
        esp_comm_send_nak(ESP_ERR_UNKNOWN_CMD);
        break;
    }
}
