/* Test shim: the real firmware esp_comm.c behind a C ABI that Python's
 * ctypes can drive, so the host tool is tested against the device's own
 * parser/dispatcher instead of a Python re-implementation of it. */
#include <stdint.h>
#include <string.h>
#include "esp_comm.h"

static uint8_t tx[65536]; static uint32_t tx_n;
static uint8_t sh[65536]; static uint32_t sh_n;
static esp_status_snapshot_t st;
static int inject_ok = 1, last_button = -1, presses = 0;
static esp_meter_snapshot_t mt; static int meter_ok = 0, meter_wrong = 0;

static void wr(const uint8_t *d, uint16_t n) { if (tx_n + n <= sizeof tx) { memcpy(tx + tx_n, d, n); tx_n += n; } }
static void to_shell(const uint8_t *d, uint16_t n, void *c) { (void)c; if (sh_n + n <= sizeof sh) { memcpy(sh + sh_n, d, n); sh_n += n; } }
static void status(esp_status_snapshot_t *o) { *o = st; }
static esp_meter_result_t meter(esp_meter_snapshot_t *o) { if (meter_wrong) return ESP_METER_WRONG_MODE; if (!meter_ok) return ESP_METER_NOT_READY; *o = mt; return ESP_METER_OK; }
void shim_set_meter_wrong_mode(int w) { meter_wrong = w; }
static bool inject(uint8_t id) { if (!inject_ok) return false; last_button = id; presses++; return true; }

/* Waveform provider. wave_state: 0 ok, 1 no capture data, 2 wrong mode,
 * 3 busy, 4 = LIES: answers OK with a synthetic record (must be refused).
 * wave_step > 0 advances frame_id by that much per request, like a live
 * acquisition; 0 holds it (a stopped scope). */
static esp_wave_snapshot_t wv;
static uint8_t wv_s[2][ESP_WAVE_MAX_SAMPLES];
static int wave_state = 1, wave_step = 0;
static esp_wave_result_t wave(uint8_t mask, esp_wave_snapshot_t *o)
{
    (void)mask;
    if (wave_state == 1) return ESP_WAVE_NO_DATA;
    if (wave_state == 2) return ESP_WAVE_WRONG_MODE;
    if (wave_state == 3) return ESP_WAVE_BUSY;
    *o = wv;
    o->synthetic = (wave_state == 4);
    wv.frame_id += (uint32_t)wave_step;
    return ESP_WAVE_OK;
}
void shim_set_wave_state(int state) { wave_state = state; }
void shim_set_waveform(uint32_t frame_id, int step, int tb_idx, int tb_tier, uint32_t rate_hz,
                       int time_ordered, int tb_disagrees, int head_skip)
{
    wv.frame_id = frame_id; wave_step = step; wv.timebase_idx = (uint8_t)tb_idx;
    wv.timebase_tier = (uint8_t)tb_tier; wv.sample_rate_hz = rate_hz;
    wv.time_ordered = time_ordered != 0; wv.timebase_disagrees = tb_disagrees != 0;
    wv.counts_per_div = 32; wv.head_skip = (uint16_t)head_skip;
    wave_state = 0;
}
void shim_set_wave_channel(int ch, const uint8_t *samples, int n, int vdiv_idx, int vdiv_tier, uint32_t uv_per_div)
{
    esp_wave_channel_t *c = &wv.ch[ch ? 1 : 0];
    if (n > ESP_WAVE_MAX_SAMPLES) n = ESP_WAVE_MAX_SAMPLES;
    memcpy(wv_s[ch ? 1 : 0], samples, (size_t)n);
    c->samples = wv_s[ch ? 1 : 0]; c->count = (uint16_t)n;
    c->vdiv_idx = (uint8_t)vdiv_idx; c->vdiv_tier = (uint8_t)vdiv_tier; c->uv_per_div = uv_per_div;
}

/* Control surface (M6). Stubs record the last call; ctrl_result is what every
 * set returns (0 OK, 1 NOT_READY, 2 WRONG_MODE). A set that returns OK is
 * reflected in the state GET_SCOPE_STATE reports, like the firmware's. */
static esp_scope_state_t cs;
static int ctrl_result = 0, ctrl_calls = 0;
static int last_ctrl[6];            /* [0] which (1 mode .. 5 run), then args */
static void rec(int which, int a, int b, int c, int d) { ctrl_calls++; last_ctrl[0] = which; last_ctrl[1] = a; last_ctrl[2] = b; last_ctrl[3] = c; last_ctrl[4] = d; }
static void c_get(esp_scope_state_t *o) { *o = cs; }
static esp_ctrl_result_t c_mode(uint8_t m) { rec(1, m, 0, 0, 0); if (!ctrl_result) cs.mode = m; return (esp_ctrl_result_t)ctrl_result; }
static esp_ctrl_result_t c_tb(uint8_t code) { rec(2, code, 0, 0, 0); if (!ctrl_result) cs.timebase_idx = cs.timebase_code = code; return (esp_ctrl_result_t)ctrl_result; }
static esp_ctrl_result_t c_vdiv(uint8_t ch, uint8_t r) { rec(3, ch, r, 0, 0); if (!ctrl_result) cs.ch[ch - 1].vdiv_idx = r; return (esp_ctrl_result_t)ctrl_result; }
static esp_ctrl_result_t c_trig(uint8_t m, uint8_t e, uint8_t s, int16_t l) { rec(4, m, e, s, l); if (!ctrl_result) { cs.trig_mode = m; cs.trig_edge = e; cs.trig_source = s; cs.trig_level = l; } return (esp_ctrl_result_t)ctrl_result; }
static esp_ctrl_result_t c_run(bool r) { rec(5, r, 0, 0, 0); if (!ctrl_result) cs.running = r; return (esp_ctrl_result_t)ctrl_result; }
/* stage-086: description (fixed tables + the live probe of cs) and SET_PROBE
 * (which = 6). The probe lives in cs so SCOPE_STATE and DESCRIPTION agree. */
static esp_description_t ds;
static void c_desc(esp_description_t *o) { *o = ds; o->probe[0] = cs.ch[0].probe; o->probe[1] = cs.ch[1].probe; }
static esp_ctrl_result_t c_probe(uint8_t ch, uint8_t f) { rec(6, ch, f, 0, 0); if (!ctrl_result) cs.ch[ch - 1].probe = f; return (esp_ctrl_result_t)ctrl_result; }
static const esp_control_ops_t ctrl_ops = { c_get, c_mode, c_tb, c_vdiv, c_trig, c_run, c_desc, c_probe };
void shim_set_desc_rate(int code, uint32_t rate, int tier) { ds.tb_rate_hz[code] = rate; ds.tb_tier[code] = (uint8_t)tier; }
void shim_set_desc_range(int ch, int r, uint32_t uv, int tier) { ds.uv_per_div_bnc[ch ? 1 : 0][r] = uv; ds.vdiv_tier[ch ? 1 : 0][r] = (uint8_t)tier; }
void shim_set_scope_probe(int ch, int f) { cs.ch[ch ? 1 : 0].probe = (uint8_t)f; }
static void desc_init(void)
{
    memset(&ds, 0, sizeof ds);
    ds.samples_per_div = 32; ds.counts_per_div = 32;
    ds.trig_level_min = -100; ds.trig_level_max = 100; ds.trig_center_code = 128;
    ds.trig_num = 256; ds.trig_den = 206; ds.trig_code_min = 1; ds.trig_code_max = 254;
    ds.trig_record_offset = -28;
}
void shim_set_ctrl_result(int r) { ctrl_result = r; }
void shim_set_ctrl_bound(int b) { esp_comm_set_control_provider(b ? &ctrl_ops : 0); }
int  shim_ctrl_calls(void) { return ctrl_calls; }
int  shim_last_ctrl(int i) { return (i >= 0 && i < 6) ? last_ctrl[i] : -999; }
void shim_set_scope_state(int mode, int running, int tb_idx, int tb_code, int tb_tier, uint32_t rate,
                          int tmode, int tedge, int tsrc, int tlevel, int tcode)
{
    cs.mode = (uint8_t)mode; cs.running = running != 0; cs.timebase_idx = (uint8_t)tb_idx;
    cs.timebase_code = (uint8_t)tb_code; cs.timebase_tier = (uint8_t)tb_tier; cs.sample_rate_hz = rate;
    cs.trig_mode = (uint8_t)tmode; cs.trig_edge = (uint8_t)tedge; cs.trig_source = (uint8_t)tsrc;
    cs.trig_level = (int16_t)tlevel; cs.trig_code = (uint8_t)tcode; cs.counts_per_div = 32;
}
void shim_set_scope_channel(int ch, int enabled, int vdiv, int tier, uint32_t uv)
{
    esp_scope_channel_t *c = &cs.ch[ch ? 1 : 0];
    c->enabled = enabled != 0; c->vdiv_idx = (uint8_t)vdiv; c->vdiv_tier = (uint8_t)tier; c->uv_per_div = uv;
}

void shim_init(void)
{
    esp_comm_init();
    esp_comm_set_block_writer(wr);
    esp_comm_set_status_provider(status);
    esp_comm_set_button_injector(inject);
    esp_comm_set_meter_provider(meter);
    esp_comm_set_waveform_provider(wave);
    esp_comm_set_control_provider(&ctrl_ops);
    memset(&cs, 0, sizeof cs); memset(last_ctrl, 0, sizeof last_ctrl);
    cs.ch[0].probe = cs.ch[1].probe = 1; desc_init();
    ctrl_result = 0; ctrl_calls = 0;
    memset(&wv, 0, sizeof wv); wave_state = 1; wave_step = 0;
    meter_ok = 0; meter_wrong = 0; memset(&mt, 0, sizeof mt);
    tx_n = sh_n = 0; presses = 0; last_button = -1; inject_ok = 1;
    memset(&st, 0, sizeof st);
    st.fw_version = "OpenScope 2C53T shim";
}
void shim_set_status(int mode, int pct, int flags, int mv, uint32_t up, uint32_t stalls, int heals)
{
    st.current_mode = (uint8_t)mode; st.battery_pct = (uint8_t)pct; st.flags = (uint8_t)flags;
    st.battery_mv = (uint16_t)mv; st.uptime_ms = up; st.usb_tx_stalls = stalls; st.usb_heals = (uint16_t)heals;
}
void shim_set_inject_ok(int ok) { inject_ok = ok; }
int  shim_last_button(void) { return last_button; }
int  shim_presses(void) { return presses; }
void shim_feed(const uint8_t *d, uint16_t n, uint32_t now) { esp_comm_route(d, n, now, to_shell, 0); }
int  shim_poll(uint32_t now) { return esp_comm_rx_poll(now); }
uint32_t shim_take_tx(uint8_t *out, uint32_t max) { uint32_t n = tx_n < max ? tx_n : max; memcpy(out, tx, n); memmove(tx, tx + n, tx_n - n); tx_n -= n; return n; }
uint32_t shim_take_shell(uint8_t *out, uint32_t max) { uint32_t n = sh_n < max ? sh_n : max; memcpy(out, sh, n); memmove(sh, sh + n, sh_n - n); sh_n -= n; return n; }
void shim_set_meter(uint32_t count, float value, int bcd, int dp, int cls, int flags, const char *unit, const char *disp)
{
    meter_ok = 1; mt.update_count = count; mt.value = value; mt.raw_bcd = (int16_t)bcd;
    mt.decimal_pos = (uint8_t)dp; mt.result_class = (uint8_t)cls; mt.flags = (uint8_t)flags;
    strncpy(mt.unit, unit, 15); strncpy(mt.display, disp, 15);
}
