#!/usr/bin/env python3
"""Build and run the remote-protocol host tests, then prove they can fail.

firmware/tests/test_remote_proto.c exercises esp_comm.c the way the USB
binding (issue #10) uses it. Every guard in that path exists to stop one
specific lie or deafness: a false ACK, a silently dropped frame, an oversize
frame leaking into the shell, a truncated frame leaving the shell deaf.

Same method as test_cal_backup.py: for each guard, copy the source, remove
the guard, rebuild, and REQUIRE the C suite to go red. A mutation that still
passes is a failure here. If a substitution string is no longer found (the
code was refactored), this suite fails loudly instead of skipping.

Three C suites run here (stage-155), each base build with -Werror:
  * test_remote_proto.c    against esp_comm.c        (MUTATIONS)
  * test_usb_link_policy.c against usb_link_policy.c (POLICY_MUTATIONS): the
    CDC link decisions usb_debug.c makes (stall, heal, TX gates, RX watchdog,
    DTR)
  * test_esp_comm.c        against esp_comm.c with ESP_COMM_TRANSFER_STUBS=1
    (base run only; the framing guards are mutated through test_remote_proto)
"""

from __future__ import annotations

import shutil
import subprocess
import sys
import tempfile
import unittest
from dataclasses import dataclass
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
DRIVERS = REPO / "firmware" / "src" / "drivers"
TESTS = REPO / "firmware" / "tests"
SRC = DRIVERS / "esp_comm.c"
HDR = DRIVERS / "esp_comm.h"
TEST_C = TESTS / "test_remote_proto.c"
POLICY_SRC = DRIVERS / "usb_link_policy.c"
POLICY_HDR = DRIVERS / "usb_link_policy.h"
POLICY_TEST_C = TESTS / "test_usb_link_policy.c"
ESP_TEST_C = TESTS / "test_esp_comm.c"
ACQ_HDR = DRIVERS / "acq_ctrl.h"            # stage-230: park owner, mode gate, trigger write

CFLAGS = ["-std=gnu11", "-Wall", "-Wextra", "-O1"]   # no -Werror: a mutant may leave an unused variable
BASE_CFLAGS = [*CFLAGS, "-Werror"]                    # the unmutated suites build warning-free


class BuildFailed(Exception):
    """A mutant that does not compile proves nothing about the guard."""


@dataclass
class Mutation:
    name: str
    old: str
    new: str
    in_header: bool = False     # mutate the suite's header instead of its source
    in_file: str = ""           # or mutate this extra file of the suite (by name)


MUTATIONS: tuple[Mutation, ...] = (
    Mutation(
        name="bad checksum dropped silently instead of NAKed",
        old="case RX_BAD_CHECKSUM: esp_comm_send_nak(ESP_ERR_BAD_CHECKSUM); break;",
        new="case RX_BAD_CHECKSUM: break;",
    ),
    Mutation(
        name="oversize length dropped silently instead of NAKed",
        old="case RX_BAD_LENGTH:   esp_comm_send_nak(ESP_ERR_BAD_LENGTH); break;",
        new="case RX_BAD_LENGTH:   break;",
    ),
    Mutation(
        name="oversize frame payload no longer swallowed (leaks into shell)",
        old="rx_discard = (uint32_t)rx_packet.payload_len + ESP_CHECKSUM_SIZE;",
        new="rx_discard = 0;",
    ),
    Mutation(
        name="gap timeout never fires (truncated frame deafens the shell)",
        old="if ((uint32_t)(now_ms - rx_last_ms) < ESP_RX_GAP_MS)\n        return false;",
        new="if (1)\n        return false;",
    ),
    Mutation(
        name="abandoned frame answered with an unsolicited NAK",
        old="    rx_stats.gap_timeouts++;\n",
        new="    rx_stats.gap_timeouts++;\n    esp_comm_send_nak(ESP_ERR_BAD_LENGTH);\n",
    ),
    Mutation(
        name="button ACKed without an injector (false success)",
        old="    if (!button_injector) {\n        esp_comm_send_nak(ESP_ERR_UNSUPPORTED);\n        return;\n    }",
        new="    if (!button_injector) {\n        esp_comm_send_ack();\n        return;\n    }",
    ),
    Mutation(
        name="button ACKed even when the queue refused it",
        old="    if (!button_injector(pkt->payload[0])) {\n        esp_comm_send_nak(ESP_ERR_NOT_READY);\n        return;\n    }",
        new="    (void)button_injector(pkt->payload[0]);",
    ),
    Mutation(
        name="button id lower bound unchecked (id 0 queued)",
        old="if (pkt->payload[0] < ESP_BTN_CH1 || pkt->payload[0] > ESP_BTN_POWER) {",
        new="if (pkt->payload[0] > ESP_BTN_POWER) {",
    ),
    Mutation(
        name="button id upper bound unchecked",
        old="if (pkt->payload[0] < ESP_BTN_CH1 || pkt->payload[0] > ESP_BTN_POWER) {",
        new="if (pkt->payload[0] < ESP_BTN_CH1) {",
    ),
    Mutation(
        name="fw_version cap removed (stack overflow in handle_status)",
        old="return (uint8_t)(n > ESP_FW_VERSION_MAX ? ESP_FW_VERSION_MAX : n);",
        new="return (uint8_t)n;",
    ),
    Mutation(
        name="gap poll leaves an oversize discard running (lying length deafens the shell)",
        old="    rx_state = RX_WAIT_SYNC;\n    rx_discard = 0;\n    rx_stats.gap_timeouts++;",
        new="    rx_state = RX_WAIT_SYNC;\n    rx_stats.gap_timeouts++;",
    ),
    Mutation(
        name="gap clock refreshed only at the sync byte (timeout from frame start)",
        old="        rx_last_ms = now_ms;\n        switch (rx_step(b)) {",
        new="        if (b == ESP_SYNC_BYTE) rx_last_ms = now_ms;\n        switch (rx_step(b)) {",
    ),
    Mutation(
        name="rx_touch does nothing",
        old="    if (esp_comm_rx_in_frame())\n        rx_last_ms = now_ms;",
        new="    (void)now_ms;",
    ),
    Mutation(
        name="stub commands ACK again",
        old="    case ESP_CMD_SIGNAL_CONFIG:\n        esp_comm_send_nak(ESP_ERR_UNSUPPORTED);",
        new="    case ESP_CMD_SIGNAL_CONFIG:\n        esp_comm_send_ack();",
    ),
    Mutation(
        name="STATUS mode hardcoded again",
        old="    out[1] = st.current_mode;",
        new="    out[1] = 0;",
    ),
    Mutation(
        name="meter with no reading answers a zero frame instead of NOT_READY",
        old="    default:\n        esp_comm_send_nak(ESP_ERR_NOT_READY);             /* no reading yet: say so */\n        return;",
        new="    default:\n        break;",
    ),
    Mutation(
        name="frozen reading sent outside meter mode",
        old="    case ESP_METER_WRONG_MODE:\n        esp_comm_send_nak(ESP_ERR_UNSUPPORTED_IN_MODE);   /* frozen, not live */\n        return;",
        new="    case ESP_METER_WRONG_MODE:\n        break;",
    ),
    Mutation(
        name="raw BCD dropped from METER_FRAME",
        old="    put_u16(&out[8], (uint16_t)m.raw_bcd);",
        new="    put_u16(&out[8], 0);",
    ),
    # ── GET_WAVEFORM (M5): every guard below stops one specific lie ──
    Mutation(
        name="waveform: synthetic record sent (flagged) instead of refused",
        old="    if (w.synthetic || w.frame_id == 0) {",
        new="    if (w.frame_id == 0) {",
    ),
    Mutation(
        name="waveform: frame_id 0 (no committed record) sent as a capture",
        old="    if (w.synthetic || w.frame_id == 0) {",
        new="    if (w.synthetic) {",
    ),
    Mutation(
        name="waveform: NO_DATA answered with a frame instead of NO_CAPTURE_DATA",
        old="    default:\n        esp_comm_send_nak(ESP_ERR_NO_CAPTURE_DATA);       /* no capture yet: never the demo trace */\n        return;",
        new="    default:\n        break;",
    ),
    Mutation(
        name="waveform: frozen record sent outside scope mode",
        old="    case ESP_WAVE_WRONG_MODE:\n        esp_comm_send_nak(ESP_ERR_UNSUPPORTED_IN_MODE);   /* buffers not live outside scope mode */\n        return;",
        new="    case ESP_WAVE_WRONG_MODE:\n        break;",
    ),
    Mutation(
        name="waveform: torn copy (BUSY) sent anyway",
        old="    case ESP_WAVE_BUSY:\n        esp_comm_send_nak(ESP_ERR_NOT_READY);             /* no tear-free copy: retry */\n        return;",
        new="    case ESP_WAVE_BUSY:\n        break;",
    ),
    Mutation(
        name="waveform: no provider answers with an empty success",
        old="    if (!wave_provider) {\n        esp_comm_send_nak(ESP_ERR_UNSUPPORTED);\n        return;\n    }",
        new="    if (!wave_provider) {\n        return;\n    }",
    ),
    Mutation(
        name="waveform: mask 0 accepted",
        old="    if (mask == 0 || (mask & (uint8_t)~ESP_WAVE_MASK_ALL) != 0) {",
        new="    if ((mask & (uint8_t)~ESP_WAVE_MASK_ALL) != 0) {",
    ),
    Mutation(
        name="waveform: unknown channel bits accepted",
        old="    if (mask == 0 || (mask & (uint8_t)~ESP_WAVE_MASK_ALL) != 0) {",
        new="    if (mask == 0) {",
    ),
    Mutation(
        name="waveform: payload length unchecked",
        old="    if (pkt->payload_len != 1) {\n        esp_comm_send_nak(ESP_ERR_BAD_LENGTH);\n        return;\n    }\n    mask = pkt->payload[0];",
        new="    mask = pkt->payload[0];",
    ),
    Mutation(
        name="waveform: sample count bound removed",
        old="ch->count != 0 && ch->count <= ESP_WAVE_MAX_SAMPLES;",
        new="ch->count != 0;",
    ),
    Mutation(
        name="waveform: empty record accepted",
        old="ch->samples != 0 && ch->count != 0 && ",
        new="ch->samples != 0 && ",
    ),
    Mutation(
        name="waveform: CH1 sent before CH2 is validated (not all-or-nothing)",
        old="        if ((mask & (1u << c)) && !wave_channel_ok(&w.ch[c])) {",
        new="        if ((mask & (1u << c)) && c == 0 && !wave_channel_ok(&w.ch[c])) {",
    ),
    Mutation(
        name="waveform: frame claims calibration",
        old="    uint8_t f = 0;      /* bit0 calibrated stays clear: no per-unit cal exists (§3.5) */",
        new="    uint8_t f = ESP_WAVE_FLAG_CALIBRATED;",
    ),
    Mutation(
        name="waveform: tier NONE rate still sent",
        old="    put_u32(&hdr[12], tb ? w->sample_rate_hz : 0);",
        new="    put_u32(&hdr[12], w->timebase_disagrees ? 0 : w->sample_rate_hz);",
    ),
    Mutation(
        name="waveform: tier NONE volts/div still sent",
        old="    put_u32(&hdr[16], vd ? ch->uv_per_div : 0);",
        new="    put_u32(&hdr[16], ch->uv_per_div);",
    ),
    Mutation(
        name="waveform: rate claimed while display and hardware disagree",
        old="    tb = w->timebase_disagrees ? 0      /* the rate belongs to a code not in force */\n                               : tier_flag(",
        new="    tb = 0 ? 0\n                               : tier_flag(",
    ),
    Mutation(
        name="waveform: PROVISIONAL reported as MEASURED",
        old="    if (tier == ESP_TIER_PROVISIONAL)\n        return provisional;",
        new="    if (tier == ESP_TIER_PROVISIONAL)\n        return measured;",
    ),
    Mutation(
        name="waveform: zero value flagged measured",
        old="    if (value == 0)\n        return 0;\n    if (tier == ESP_TIER_MEASURED)",
        new="    if (tier == ESP_TIER_MEASURED)",
    ),
    Mutation(
        name="waveform: every frame labelled CH1",
        old="    hdr[4] = c;",
        new="    hdr[4] = 0;",
    ),
    Mutation(
        name="waveform: checksum ignores the samples part",
        old="                                                  ^ esp_comm_checksum(b, blen);",
        new="                                                  ;",
    ),
    Mutation(
        name="waveform: byte writer drops the samples part",
        old="    for (i = 0; i < blen; i++)\n        uart_write(b[i]);\n",
        new="",
    ),
    Mutation(
        name="router passes frame bytes to the shell",
        old="        if (!esp_comm_rx_in_frame() && b != ESP_SYNC_BYTE)\n            continue;",
        new="        if (b != ESP_SYNC_BYTE && !esp_comm_rx_in_frame())\n            continue;\n        if (passthrough) passthrough(&b, 1, ctx);",
    ),
    # ── stage-086: DESCRIPTION, SET_PROBE, SCOPE_STATE probe append, host badge ──
    Mutation(
        name="description: tier NONE value sent bare",
        old="    const bool usable = tier_flag(tier, value, 1, 1) != 0;\n    put_u32(p, usable ? value : 0);",
        new="    const bool usable = tier_flag(tier, value, 1, 1) != 0;\n    put_u32(p, value);",
    ),
    Mutation(
        name="description: zero value keeps a tier that vouches for it",
        old="    p[4] = usable ? tier : ESP_TIER_NONE;",
        new="    p[4] = tier;",
    ),
    Mutation(
        name="description: probe section dropped",
        old="    p = put_section(p, ESP_DESC_SEC_PROBE, ESP_DESC_PROBE_LEN);\n"
            "    *p++ = wire_probe(d.probe[0]);\n    *p++ = wire_probe(d.probe[1]);\n",
        new="",
    ),
    # ── stage-190: DESCRIPTION section 7 (identity) ──
    Mutation(
        name="identity: text not cut to ESP_DESC_TEXT_MAX",
        old="    p = put_text(p, d.build_target, ESP_DESC_TEXT_MAX);",
        new="    p = put_text(p, d.build_target, 255);",
    ),
    Mutation(
        name="description texts never cut (put_text ignores max)",
        old="        while (n < max && s[n] != '\\0')",
        new="        while (s[n] != '\\0')",
    ),
    Mutation(
        name="identity: section length left at 0",
        old="    put_u16(&ident[1], (uint16_t)(p - ident - 3));\n",
        new="",
    ),
    Mutation(
        name="identity: api level not sent",
        old="    put_u16(p, ESP_API_LEVEL); p += 2;\n",
        new="    put_u16(p, 0); p += 2;\n",
    ),
    Mutation(
        name="SET_PROBE: factor other than 1/10 accepted",
        old="        (pkt->payload[1] != 1 && pkt->payload[1] != 10)) {",
        new="        (pkt->payload[1] == 0)) {",
    ),
    Mutation(
        name="SET_PROBE: channel 0 accepted",
        old="    if ((pkt->payload[0] != 1 && pkt->payload[0] != 2) ||\n        (pkt->payload[1] != 1",
        new="    if ((pkt->payload[0] > 2) ||\n        (pkt->payload[1] != 1",
    ),
    # ── stage-272: SET_METER_MODE, DESCRIPTION section 5 ──
    Mutation(
        name="SET_METER_MODE: function 11 accepted",
        old="    if (pkt->payload[0] >= ESP_METER_FUNCTIONS) {",
        new="    if (pkt->payload[0] > ESP_METER_FUNCTIONS) {",
    ),
    Mutation(
        name="SET_METER_MODE: provider BAD_ARG sent as NOT_READY",
        old="    case ESP_CTRL_BAD_ARG:    esp_comm_send_nak(ESP_ERR_BAD_ARG); break;\n",
        new="",
    ),
    Mutation(
        name="SET_METER_MODE: not dispatched",
        old="    case ESP_CMD_SET_METER_MODE:    handle_set_meter_mode(pkt); break;\n",
        new="",
    ),
    Mutation(
        name="meter section: usable flag sent raw",
        old="            *p++ = d.meter_usable[i] ? 1u : 0u;",
        new="            *p++ = d.meter_usable[i];",
    ),
    Mutation(
        name="meter section: unit not cut to ESP_DESC_METER_UNIT_MAX",
        old="            p = put_text(p, d.meter_unit[i], ESP_DESC_METER_UNIT_MAX);",
        new="            p = put_text(p, d.meter_unit[i], ESP_DESC_METER_NAME_MAX);",
    ),
    Mutation(
        name="meter section: length left at 0",
        old="        put_u16(&meter[1], (uint16_t)(p - meter - 3));\n",
        new="",
    ),
    Mutation(
        name="meter section: sent with no functions",
        old="    if (d.meter_n) {",
        new="    if (1) {",
    ),
    Mutation(
        name="SCOPE_STATE: provider probe sent raw",
        old="    return (factor == 10u) ? 10u : 1u;",
        new="    return factor;",
    ),
    Mutation(
        name="SCOPE_STATE: probe bytes not appended",
        old="    out[36] = wire_probe(s.ch[0].probe);        /* stage-086 append */\n",
        new="",
    ),
    Mutation(
        name="host badge: a valid frame does not start the clock",
        old="            host_seen = true;               /* the remote badge clock (stage-086) */\n",
        new="",
    ),
    Mutation(
        name="host badge: never times out",
        old="    return host_seen && (uint32_t)(now_ms - host_last_ms) < ESP_HOST_ACTIVE_MS;",
        new="    return host_seen;",
    ),
    # ── stage-155: parser gaps ──
    Mutation(
        name="payload cap off by one low (256 refused)",
        old="if (rx_packet.payload_len > ESP_MAX_PAYLOAD) {",
        new="if (rx_packet.payload_len >= ESP_MAX_PAYLOAD) {",
    ),
    Mutation(
        name="payload cap off by one high (257 accepted)",
        old="if (rx_packet.payload_len > ESP_MAX_PAYLOAD) {",
        new="if (rx_packet.payload_len > ESP_MAX_PAYLOAD + 1) {",
    ),
    Mutation(
        name="resync on a 0xAA inside the payload",
        old="    case RX_WAIT_PAYLOAD:\n        rx_packet.payload[rx_payload_idx++] = byte;",
        new="    case RX_WAIT_PAYLOAD:\n"
            "        if (byte == ESP_SYNC_BYTE) { rx_state = RX_WAIT_CMD; rx_checksum_acc = 0; return RX_NONE; }\n"
            "        rx_packet.payload[rx_payload_idx++] = byte;",
    ),
    Mutation(
        name="bad checksum eats the next byte (no resync on the following frame)",
        old="        rx_stats.bad_checksum++;\n        return RX_BAD_CHECKSUM;",
        new="        rx_stats.bad_checksum++;\n        rx_discard = 1;\n        return RX_BAD_CHECKSUM;",
    ),
    Mutation(
        name="gap timeout breaks at the 32-bit tick wrap",
        old="if ((uint32_t)(now_ms - rx_last_ms) < ESP_RX_GAP_MS)\n        return false;",
        new="if (now_ms < rx_last_ms + ESP_RX_GAP_MS)\n        return false;",
    ),
    Mutation(
        name="send_frame2 keeps writing after a failed part",
        old="        if (alen && !block_write(a, alen))\n            return;",
        new="        if (alen) (void)block_write(a, alen);",
    ),
    # ── stage-230: provider WRONG_MODE on the wire, SET argument guards ──
    Mutation(
        name="set: WRONG_MODE answered as NOT_READY",
        old="    case ESP_CTRL_WRONG_MODE: esp_comm_send_nak(ESP_ERR_UNSUPPORTED_IN_MODE); break;\n",
        new="",
    ),
    Mutation(
        name="SET_TRIGGER: level lower bound unchecked",
        old="level < -ESP_TRIG_LEVEL_MAX || level > ESP_TRIG_LEVEL_MAX",
        new="level > ESP_TRIG_LEVEL_MAX",
    ),
    Mutation(
        name="SET_TRIGGER: level read as unsigned",
        old="static int16_t get_i16(const uint8_t *p) { return (int16_t)(uint16_t)(p[0] | (p[1] << 8)); }",
        new="static int16_t get_i16(const uint8_t *p) { return (int16_t)(p[0] & 0x7F); }",
    ),
    Mutation(
        name="SET_VDIV: range past the table accepted",
        old="pkt->payload[1] >= ESP_VDIV_RANGES",
        new="pkt->payload[1] > ESP_VDIV_RANGES",
    ),
    Mutation(
        name="RUN_STOP: argument 2 accepted",
        old="    if (pkt->payload[0] > 1) {\n        esp_comm_send_nak(ESP_ERR_BAD_ARG);\n        return;\n    }\n"
            "    ctrl_reply(control_ops->run_stop",
        new="    ctrl_reply(control_ops->run_stop",
    ),
    # ── stage-230: acq_ctrl.h, the park owner and the scope-set decisions ──
    Mutation(
        name="park: another task counts itself into the holder's park",
        old="return me != NULL && o->depth > 0u && o->holder == me;",
        new="return me != NULL && o->depth > 0u;",
        in_file="acq_ctrl.h",
    ),
    Mutation(
        name="park: a resume by a non-holder ends the park",
        old="if (me == NULL || o->depth == 0u || o->holder != me)",
        new="if (me == NULL || o->depth == 0u)",
        in_file="acq_ctrl.h",
    ),
    Mutation(
        name="park: a resume with no park is accepted",
        old="if (me == NULL || o->depth == 0u || o->holder != me)",
        new="if (me == NULL || (o->depth != 0u && o->holder != me))",
        in_file="acq_ctrl.h",
    ),
    Mutation(
        name="park: the inner resume of a nested pause ends the park",
        old="    if (o->depth > 1u) {",
        new="    if (0) {",
        in_file="acq_ctrl.h",
    ),
    Mutation(
        name="park: a nested pause is not counted",
        old="    o->depth++;\n    return true;",
        new="    return true;",
        in_file="acq_ctrl.h",
    ),
    Mutation(
        name="park: the nested count wraps",
        old="if (!acq_owner_nested(o, me) || o->depth == UINT8_MAX)",
        new="if (!acq_owner_nested(o, me))",
        in_file="acq_ctrl.h",
    ),
    Mutation(
        name="park: the last resume keeps the holder",
        old="    o->depth = 0u;\n    o->holder = NULL;\n    return ACQ_RELEASE_GIVE;",
        new="    o->depth = 0u;\n    return ACQ_RELEASE_GIVE;",
        in_file="acq_ctrl.h",
    ),
    Mutation(
        name="U-4: scope sets allowed in every mode",
        old="    return mode == ACQ_CTRL_SCOPE_MODE;",
        new="    (void)mode;\n    return true;",
        in_file="acq_ctrl.h",
    ),
    Mutation(
        name="U-4: scope sets refused in scope mode",
        old="    return mode == ACQ_CTRL_SCOPE_MODE;",
        new="    return mode != ACQ_CTRL_SCOPE_MODE && mode < 2u;",
        in_file="acq_ctrl.h",
    ),
    Mutation(
        name="A-8: every SET_TRIGGER writes (and parks)",
        old="    return code_new != code_now || src_new != src_now;",
        new="    (void)code_now; (void)src_now; (void)code_new; (void)src_new;\n    return true;",
        in_file="acq_ctrl.h",
    ),
    Mutation(
        name="A-8: a source change alone is not written",
        old="    return code_new != code_now || src_new != src_now;",
        new="    (void)src_now; (void)src_new;\n    return code_new != code_now;",
        in_file="acq_ctrl.h",
    ),
)

# ── stage-155: usb_link_policy.c, the CDC link decisions usb_debug.c makes ──
POLICY_MUTATIONS: tuple[Mutation, ...] = (
    Mutation("stall: completion flag wins over a VALID packet",
             "    if (ept_tx_valid)\n        return USB_LINK_STALL_HOST_SLOW;",
             "    if (ept_tx_valid && !tx_completed)\n        return USB_LINK_STALL_HOST_SLOW;"),
    Mutation("stall: late completion counted as lost",
             "    if (tx_completed)\n        return USB_LINK_STALL_LATE;\n", ""),
    Mutation("stall: host-slow counted as lost",
             "    if (ept_tx_valid)\n        return USB_LINK_STALL_HOST_SLOW;\n", ""),
    Mutation("stall counter wraps at 0xFF",
             "return (consecutive < 0xFFu) ? (uint8_t)(consecutive + 1u) : consecutive;",
             "return (uint8_t)(consecutive + 1u);"),
    Mutation("heal ignores the enable switch",
             "    return heal_enabled && dtr_seen && dtr &&",
             "    (void)heal_enabled;\n    return dtr_seen && dtr &&"),
    Mutation("heal without a DTR seen since enumeration",
             "    return heal_enabled && dtr_seen && dtr &&",
             "    (void)dtr_seen;\n    return heal_enabled && dtr &&"),
    Mutation("heal with the port closed",
             "    return heal_enabled && dtr_seen && dtr &&",
             "    (void)dtr;\n    return heal_enabled && dtr_seen &&"),
    Mutation("heal one stall late",
             "consecutive >= USB_LINK_HEAL_AFTER_STALLS &&",
             "consecutive > USB_LINK_HEAL_AFTER_STALLS &&"),
    Mutation("heal before the scheduler runs",
             "consecutive >= USB_LINK_HEAL_AFTER_STALLS &&\n           scheduler_running;",
             "consecutive >= USB_LINK_HEAL_AFTER_STALLS;\n    (void)scheduler_running;"),
    Mutation("heal threshold changed",
             "#define USB_LINK_HEAL_AFTER_STALLS  2", "#define USB_LINK_HEAL_AFTER_STALLS  3",
             in_header=True),
    Mutation("closed port still written",
             "    if (dtr_opened && !dtr)\n        return USB_LINK_TX_DROP_CLOSED;",
             "    (void)dtr_opened; (void)dtr;"),
    Mutation("host that never raised DTR is dropped",
             "    if (dtr_opened && !dtr)\n        return USB_LINK_TX_DROP_CLOSED;",
             "    (void)dtr_opened;\n    if (!dtr)\n        return USB_LINK_TX_DROP_CLOSED;"),
    Mutation("latch gate acts when not latched",
             "    if (!latched)\n        return USB_LINK_TX_SEND;\n", "    (void)latched;\n"),
    Mutation("latch not cleared by a completion",
             "    if (tx_completed)\n        return USB_LINK_TX_CLEAR_LATCH;\n", ""),
    Mutation("latched write waits instead of dropping",
             "    if (ept_tx_valid)\n        return USB_LINK_TX_DROP_STALLED;\n", ""),
    Mutation("latch kept on a lost completion (heal unreachable)",
             "        return USB_LINK_TX_DROP_STALLED;\n    return USB_LINK_TX_CLEAR_LATCH;",
             "        return USB_LINK_TX_DROP_STALLED;\n    return USB_LINK_TX_DROP_STALLED;"),
    Mutation("RX lost: NAK not required",
             "return rx_nak && !rxtc_pending && !rx_completed;",
             "(void)rx_nak;\n    return !rxtc_pending && !rx_completed;"),
    Mutation("RX lost: pending RXTC ignored",
             "return rx_nak && !rxtc_pending && !rx_completed;",
             "(void)rxtc_pending;\n    return rx_nak && !rx_completed;"),
    Mutation("RX lost: unconsumed packet ignored",
             "return rx_nak && !rxtc_pending && !rx_completed;",
             "(void)rx_completed;\n    return rx_nak && !rxtc_pending;"),
    Mutation("RX watchdog keeps suspicion after the state clears",
             "    if (!lost_now) {\n        wd->suspect = false;\n        return false;",
             "    if (!lost_now) {\n        return false;"),
    Mutation("RX watchdog recovers with no confirm window",
             "    if ((uint32_t)(now_tick - wd->suspect_tick) < confirm_ticks)\n        return false;\n",
             "    (void)confirm_ticks;\n"),
    Mutation("RX watchdog restarts the window on every poll",
             "    if (!wd->suspect) {", "    if (1) {"),
    Mutation("RX watchdog breaks at the tick wrap",
             "if ((uint32_t)(now_tick - wd->suspect_tick) < confirm_ticks)",
             "if (now_tick < wd->suspect_tick + confirm_ticks)"),
    Mutation("RX watchdog keeps suspicion after a recovery",
             "    wd->suspect = false;\n    return true;", "    return true;"),
    Mutation("banner on every DTR-high setup",
             "    if (dtr && !s.dtr)\n        s.banner_pending = 1;",
             "    if (dtr)\n        s.banner_pending = 1;"),
    Mutation("DTR setup not recorded as seen",
             "    s.dtr_seen = 1;\n    if (dtr)", "    if (dtr)"),
    Mutation("DTR rise not recorded as opened",
             "    if (dtr)\n        s.dtr_opened = 1;", "    if (0)\n        s.dtr_opened = 1;"),
    Mutation("RTS read as DTR",
             "uint8_t dtr = (uint8_t)(w_value & 0x0001u);",
             "uint8_t dtr = (uint8_t)(w_value & 0x0003u);"),
    Mutation("reopen on input with DTR high",
             "return rx_len > 0 && !s.dtr && s.dtr_opened;", "return rx_len > 0 && s.dtr_opened;"),
    Mutation("reopen when never opened",
             "return rx_len > 0 && !s.dtr && s.dtr_opened;", "return rx_len > 0 && !s.dtr;"),
    Mutation("reopen with no input",
             "return rx_len > 0 && !s.dtr && s.dtr_opened;",
             "(void)rx_len;\n    return !s.dtr && s.dtr_opened;"),
    Mutation("forget keeps DTR high",
             "    s.dtr = 0;                      /* the next open", "    /* the next open"),
    Mutation("forget keeps dtr_seen",
             "    s.dtr_seen = 0;\n    s.dtr_opened = 0;\n    return s;",
             "    s.dtr_opened = 0;\n    return s;"),
    Mutation("forget keeps dtr_opened",
             "    s.dtr_seen = 0;\n    s.dtr_opened = 0;\n    return s;",
             "    s.dtr_seen = 0;\n    return s;"),
    Mutation("forget clears the pending banner",
             "    s.dtr_opened = 0;\n    return s;\n}\n\nbool usb_link_forget_on_unconfigured",
             "    s.dtr_opened = 0;\n    s.banner_pending = 0;\n    return s;\n}\n\n"
             "bool usb_link_forget_on_unconfigured"),
    Mutation("DTR forgotten on a suspend",
             "    return !suspended;", "    (void)suspended;\n    return true;"),
    Mutation("DTR fall detected on any change",
             "return dtr_prev && !dtr_now;", "return dtr_prev != dtr_now;"),
    Mutation("banner for a binary host",
             "return !banner_sent && first_byte != sync_byte && !loader_busy;",
             "(void)first_byte; (void)sync_byte;\n    return !banner_sent && !loader_busy;"),
    Mutation("banner during a firmware transfer",
             "return !banner_sent && first_byte != sync_byte && !loader_busy;",
             "(void)loader_busy;\n    return !banner_sent && first_byte != sync_byte;"),
    Mutation("second banner",
             "return !banner_sent && first_byte != sync_byte && !loader_busy;",
             "(void)banner_sent;\n    return first_byte != sync_byte && !loader_busy;"),
)


@dataclass
class Suite:
    src: Path
    hdr: Path
    test_c: Path
    defines: tuple[str, ...] = ()
    extras: tuple[Path, ...] = ()       # headers the test includes besides hdr


REMOTE_PROTO = Suite(SRC, HDR, TEST_C, extras=(ACQ_HDR,))
LINK_POLICY = Suite(POLICY_SRC, POLICY_HDR, POLICY_TEST_C)
ESP_COMM = Suite(SRC, HDR, ESP_TEST_C, ("-DESP_COMM_TRANSFER_STUBS=1",))


def compile_and_run(suite: Suite, workdir: Path, source_text: str | None = None,
                    header_text: str | None = None,
                    cflags: list[str] = CFLAGS,
                    extra_texts: dict[str, str] | None = None) -> subprocess.CompletedProcess:
    src = workdir / suite.src.name
    src.write_text(suite.src.read_text(encoding="utf-8") if source_text is None else source_text,
                   encoding="utf-8")
    (workdir / suite.hdr.name).write_text(
        suite.hdr.read_text(encoding="utf-8") if header_text is None else header_text,
        encoding="utf-8")
    shutil.copy(suite.test_c, workdir / suite.test_c.name)
    for extra in suite.extras:
        text = (extra_texts or {}).get(extra.name)
        (workdir / extra.name).write_text(
            extra.read_text(encoding="utf-8") if text is None else text, encoding="utf-8")
    binary = workdir / "t"
    build = subprocess.run(
        ["cc", *cflags, *suite.defines, "-o", str(binary), str(workdir / suite.test_c.name),
         str(src), "-I", str(workdir)],
        capture_output=True, text=True,
    )
    if build.returncode != 0:
        raise BuildFailed(build.stderr)
    return subprocess.run([str(binary)], capture_output=True, text=True)


class _SuiteChecks:
    """Base build (-Werror) passes; each mutation turns it red."""
    suite: Suite
    mutations: tuple[Mutation, ...] = ()

    def test_source_files_present(self):
        for path in (self.suite.src, self.suite.hdr, self.suite.test_c, *self.suite.extras):
            self.assertTrue(path.is_file(), f"missing {path}")

    def test_base_suite_passes(self):
        with tempfile.TemporaryDirectory() as tmp:
            proc = compile_and_run(self.suite, Path(tmp), cflags=BASE_CFLAGS)   # BuildFailed = error
            self.assertEqual(proc.returncode, 0,
                             f"base suite failed:\n{proc.stdout}\n{proc.stderr}")

    def run_mutations(self):
        src = self.suite.src.read_text(encoding="utf-8")
        hdr = self.suite.hdr.read_text(encoding="utf-8")
        extras = {p.name: p.read_text(encoding="utf-8") for p in self.suite.extras}
        for m in self.mutations:
            with self.subTest(mutation=m.name):
                if m.in_file:
                    self.assertIn(m.in_file, extras, f"{m.in_file} is not a file of this suite")
                    base = extras[m.in_file]
                else:
                    base = hdr if m.in_header else src
                self.assertIn(m.old, base, f"mutation string not found (refactor?): {m.name}")
                mutated = base.replace(m.old, m.new, 1)
                self.assertNotEqual(mutated, base, "replacement was a no-op")
                in_src = not m.in_header and not m.in_file
                with tempfile.TemporaryDirectory() as tmp:
                    try:
                        proc = compile_and_run(
                            self.suite, Path(tmp),
                            source_text=mutated if in_src else None,
                            header_text=mutated if m.in_header else None,
                            extra_texts={m.in_file: mutated} if m.in_file else None)
                    except BuildFailed as e:
                        self.fail(f"mutant '{m.name}' did not build, so it proves nothing:\n{e}")
                self.assertNotEqual(
                    proc.returncode, 0,
                    f"mutation '{m.name}' did NOT make the suite fail — the guard is not tested.\n"
                    f"{proc.stdout}{proc.stderr}")


class TestRemoteProto(_SuiteChecks, unittest.TestCase):
    suite = REMOTE_PROTO
    mutations = MUTATIONS

    def test_mutations_are_caught(self):
        self.run_mutations()

    def test_acq_ctrl_has_no_rtos_or_vendor_header(self):
        # stage-230: the park and scope-set decisions compile on the host as is.
        text = ACQ_HDR.read_text(encoding="utf-8")
        for banned in ("FreeRTOS.h", "task.h", "semphr.h", "at32f403a_407.h"):
            self.assertNotIn(banned, text, f"acq_ctrl.h includes {banned}")

    def test_firmware_calls_every_acq_decision(self):
        # The decisions tested here are the ones that run on the unit.
        fpga = (DRIVERS / "fpga.c").read_text(encoding="utf-8")
        for fn in ("acq_owner_nested", "acq_owner_enter_nested", "acq_owner_took",
                   "acq_owner_release", "xSemaphoreTake(acq_park_mutex", "xSemaphoreGive(acq_park_mutex"):
            self.assertIn(fn + ("" if "(" in fn else "("), fpga, f"fpga.c does not call {fn}")
        usb = (DRIVERS / "usb_debug.c").read_text(encoding="utf-8")
        for fn in ("acq_scope_set_allowed", "acq_trigger_needs_write"):
            self.assertIn(fn + "(", usb, f"usb_debug.c does not call {fn}")
        for setter in ("remote_set_timebase", "remote_set_vdiv", "remote_set_trigger"):
            body = usb.split(f"static esp_ctrl_result_t {setter}(", 1)[1].split("\n}\n", 1)[0]
            self.assertIn("remote_scope_set_allowed()", body, f"{setter} has no mode gate")
            self.assertIn("ESP_CTRL_WRONG_MODE", body, f"{setter} does not return WRONG_MODE")


class TestUsbLinkPolicy(_SuiteChecks, unittest.TestCase):
    suite = LINK_POLICY
    mutations = POLICY_MUTATIONS

    def test_mutations_are_caught(self):
        self.run_mutations()

    def test_policy_has_no_vendor_or_rtos_header(self):
        # The point of the file: decisions testable without the USB stack.
        for path in (POLICY_SRC, POLICY_HDR):
            text = path.read_text(encoding="utf-8")
            for banned in ("FreeRTOS.h", "task.h", "at32f403a_407.h", "usbd_", "cdc_class.h"):
                self.assertNotIn(banned, text, f"{path.name} includes {banned}")

    def test_usb_debug_calls_every_decision(self):
        # Each decision tested here is the one that runs on the unit.
        text = (DRIVERS / "usb_debug.c").read_text(encoding="utf-8")
        for fn in ("usb_link_classify_stall", "usb_link_count_lost", "usb_link_heal_due",
                   "usb_link_tx_open_gate", "usb_link_tx_latch_gate",
                   "usb_link_rx_completion_lost", "usb_link_rx_watchdog_step",
                   "usb_link_dtr_on_setup", "usb_link_dtr_reopen_on_input",
                   "usb_link_dtr_forget", "usb_link_forget_on_unconfigured",
                   "usb_link_dtr_fell", "usb_link_banner_on_first_input"):
            self.assertIn(fn + "(", text, f"usb_debug.c does not call {fn}")


class TestEspComm(_SuiteChecks, unittest.TestCase):
    suite = ESP_COMM


def setUpModule():
    # A skip, not a silent exit 0: run_tests.py counts skips, and --strict
    # must not report "all tests ran" on a machine that ran none.
    if shutil.which("cc") is None:
        raise unittest.SkipTest("cc not found: the remote-protocol host tests cannot run")


if __name__ == "__main__":
    unittest.main(verbosity=2)
