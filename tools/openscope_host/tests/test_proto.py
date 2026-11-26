"""proto.py: framing and payload codecs, no hardware (remote_protocol.md §6 step 5).

The rejection paths are asserted, not just the happy path: a corrupted
checksum must be rejected, a truncated frame must never be accepted, and the
decoder must resync after garbage. test_guards_are_load_bearing removes the
checksum comparison and requires a corrupted frame to slip through.
"""
from __future__ import annotations

import os
import struct
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from openscope import proto  # noqa: E402


def status_payload(ver=1, mode=1, pct=73, flags=3, mv=3987, up=123456, stalls=2, heals=1, fw=b"OpenScope x"):
    return struct.pack("<BBBBHIIHB", ver, mode, pct, flags, mv, up, stalls, heals, len(fw)) + fw


class TestEncode(unittest.TestCase):
    def test_ping_bytes(self):
        self.assertEqual(proto.encode(proto.CMD_PING), bytes([0xAA, 0x01, 0x00, 0x00, 0x01]))

    def test_length_is_big_endian_and_checksum_xor(self):
        f = proto.encode(0x0A, bytes([0x09]))
        self.assertEqual(f, bytes([0xAA, 0x0A, 0x00, 0x01, 0x09, 0x0A ^ 0x01 ^ 0x09]))

    def test_refuses_payload_the_device_would_drop(self):
        proto.encode(0x20, bytes(256))
        with self.assertRaises(ValueError):
            proto.encode(0x20, bytes(257))


class TestDecoder(unittest.TestCase):
    def test_roundtrip(self):
        d = proto.Decoder()
        for n in (0, 1, 255, 256, 1036, 4000):
            payload = bytes((i * 31 + 7) & 0xFF for i in range(n))
            raw = bytes([0xAA, 0x91, n >> 8, n & 0xFF]) + payload
            raw += bytes([proto.checksum(raw[1:])])
            frames = d.feed(raw)
            self.assertEqual(frames, [proto.Frame(0x91, payload)], n)

    def test_corrupted_checksum_rejected(self):
        d = proto.Decoder()
        raw = bytearray(proto.encode(0x83, b"hello"))
        raw[-1] ^= 0xFF
        self.assertEqual(d.feed(bytes(raw)), [])
        self.assertEqual(d.bad_checksum, 1)

    def test_corrupted_payload_rejected(self):
        d = proto.Decoder()
        raw = bytearray(proto.encode(0x83, b"hello"))
        raw[5] ^= 0x01
        self.assertEqual(d.feed(bytes(raw)), [])

    def test_truncated_frame_never_accepted(self):
        full = proto.encode(0x83, b"payload")
        for cut in range(1, len(full)):
            d = proto.Decoder()
            self.assertEqual(d.feed(full[:cut]), [], f"accepted a frame cut at {cut}")

    def test_split_feeding_reassembles(self):
        full = proto.encode(0x85, b"abcdefgh")
        d = proto.Decoder()
        got = []
        for b in full:
            got += d.feed(bytes([b]))
        self.assertEqual(got, [proto.Frame(0x85, b"abcdefgh")])

    def test_resync_after_garbage(self):
        d = proto.Decoder()
        garbage = bytes([0xAA, 0x83, 0x00, 0x05, 1, 2]) + bytes([0xAA, 0x13, 0x37])   # two false starts
        good = proto.encode(0x81)
        frames = d.feed(garbage + good)
        # the false start announcing 5 bytes swallows the next header bytes;
        # the decoder must still find the real frame once those fail the checksum
        frames += d.feed(proto.encode(0x81))
        self.assertIn(proto.Frame(0x81, b""), frames)
        self.assertGreaterEqual(d.resyncs, 1)

    def test_resync_does_not_lose_following_frame(self):
        d = proto.Decoder()
        bad = bytearray(proto.encode(0x83, b"xx"))
        bad[-1] ^= 1
        frames = d.feed(bytes(bad) + proto.encode(0x85, b"ok"))
        self.assertEqual(frames, [proto.Frame(0x85, b"ok")])

    def test_shell_text_is_kept_separately(self):
        d = proto.Decoder()
        frames = d.feed(b"\r\n> " + proto.encode(0x81) + b"version\r\n")
        self.assertEqual(frames, [proto.Frame(0x81, b"")])
        self.assertEqual(d.take_text(), b"\r\n> version\r\n")

    def test_oversize_length_is_noise(self):
        d = proto.Decoder(max_payload=1024)
        self.assertEqual(d.feed(bytes([0xAA, 0x83, 0xFF, 0xFF]) + proto.encode(0x81)),
                         [proto.Frame(0x81, b"")])

    def test_text_keeps_only_the_last_4096_bytes(self):
        # stage-180 (H-5): nothing takes the text on a long poll loop
        d = proto.Decoder()
        for i in range(100):
            d.feed(b"%04d" % i * 25 + proto.encode(0x81))     # 100 B of text, then a frame
        self.assertEqual(len(d.text), proto.Decoder.TEXT_CAP)
        self.assertTrue(d.take_text().endswith(b"0099" * 25), "the newest text must stay")
        d.feed(b"x" * 10000)                                    # one big chunk with no frame
        self.assertEqual(bytes(d.text), b"x" * proto.Decoder.TEXT_CAP)
        d = proto.Decoder()
        d.feed(b"y" * 5000 + bytes([0xAA, 0x81]))               # text, then a header still arriving
        self.assertEqual(len(d.text), proto.Decoder.TEXT_CAP)
        self.assertEqual(d.feed(proto.encode(0x81)[2:]), [proto.Frame(0x81, b"")])


class TestStatus(unittest.TestCase):
    def test_parse(self):
        s = proto.parse_status(status_payload())
        self.assertEqual((s.mode_name, s.battery_pct, s.battery_mv), ("meter", 73, 3987))
        self.assertTrue(s.charging and s.capture_ready and not s.battery_critical)
        self.assertEqual((s.uptime_ms, s.usb_tx_stalls, s.usb_heals), (123456, 2, 1))
        self.assertEqual(s.fw_version, "OpenScope x")

    def test_unknown_major_refused(self):
        with self.assertRaises(proto.ProtocolError):
            proto.parse_status(status_payload(ver=2))

    def test_length_mismatch_refused(self):
        with self.assertRaises(proto.ProtocolError):
            proto.parse_status(status_payload()[:-1])
        with self.assertRaises(proto.ProtocolError):
            proto.parse_status(status_payload()[:10])


class TestMeter(unittest.TestCase):
    def payload(self, unit=b"kOhm", disp=b"OL"):
        return (struct.pack("<IfhBBBBBB", 9, 0.0, 0, 0, 5, 0x04, 2, 1, len(unit)) + unit
                + bytes([len(disp)]) + disp)

    def test_parse(self):
        m = proto.parse_meter(self.payload())
        self.assertEqual((m.unit, m.display, m.result, m.submode), ("kOhm", "OL", "overload", 2))

    def test_string_lengths_must_add_up(self):
        p = self.payload()
        for bad in (p[:-1], p + b"x", p[:15]):
            with self.assertRaises(proto.ProtocolError):
                proto.parse_meter(bad)


def wave_payload(fid=42, ch=0, flags=0x06, tb=0x10, vdiv=6, samples=bytes(range(16)), hlen=24,
                 rate=12490, uv=1264486, cpd=32, skip=4, extra=b""):
    hdr = struct.pack("<IBBBBHHIIHH", fid, ch, flags, tb, vdiv, len(samples), hlen, rate, uv, cpd, skip)
    return hdr + extra + samples


class TestWaveform(unittest.TestCase):
    def test_parse(self):
        w = proto.parse_waveform(wave_payload())
        self.assertEqual((w.frame_id, w.channel_name, w.sample_rate_hz, w.uv_per_div), (42, "CH1", 12490, 1264486))
        self.assertTrue(w.timebase_measured and w.vdiv_measured and not w.calibrated)
        self.assertEqual(w.samples, bytes(range(16)))
        self.assertEqual(w.body, bytes(range(4, 16)), "body starts after head_skip")
        self.assertAlmostEqual(w.volts_per_count, 1.264486 / 32)

    def test_longer_header_is_skipped_not_misread(self):
        """header_len lets the firmware append fields without a major bump."""
        w = proto.parse_waveform(wave_payload(hlen=28, extra=b"\xEE" * 4))
        self.assertEqual(w.samples, bytes(range(16)))

    def test_rejections(self):
        good = wave_payload()
        for bad, why in ((good[:-1], "short"), (good + b"x", "long"), (good[:20], "no header"),
                         (wave_payload(hlen=20), "header_len below the fixed part"),
                         (wave_payload(ch=2), "no CH3"),
                         (wave_payload(fid=0), "frame_id 0 = no record"),
                         (wave_payload(flags=proto.WAVE_FLAG_SYNTHETIC), "synthetic is refused, not returned")):
            with self.assertRaises(proto.ProtocolError, msg=why):
                proto.parse_waveform(bad)

    def test_unmeasured_numbers_are_not_invented(self):
        w = proto.parse_waveform(wave_payload(flags=0, rate=0, uv=0))
        self.assertIsNone(w.volts_per_count)
        self.assertIsNone(w.seconds_per_sample)
        self.assertEqual((w.timebase_tier, w.vdiv_tier), ("none", "none"))
        h = w.header()
        self.assertIsNone(h["sample_rate_hz"])
        self.assertIsNone(h["uv_per_div"])

    def test_channel_mask(self):
        self.assertEqual(proto.channel_mask([1]), 1)
        self.assertEqual(proto.channel_mask("1,2"), 3)
        self.assertEqual(proto.channel_mask(["CH2"]), 2)
        for bad in ([], [3], "0", [1, 3]):
            with self.assertRaises(ValueError):
                proto.channel_mask(bad)


class TestPeriodEstimate(unittest.TestCase):
    def test_sine_and_square(self):
        import math
        sine = [int(round(128 + 60 * math.sin(2 * math.pi * i / 37.3))) for i in range(900)]
        self.assertAlmostEqual(proto.estimate_period(sine), 37.3, delta=0.2)
        square = [200 if (i // 25) % 2 else 50 for i in range(900)]
        self.assertAlmostEqual(proto.estimate_period(square), 50.0, delta=0.01)

    def test_noise_at_the_level_does_not_add_crossings(self):
        import math
        noisy = [int(round(128 + 60 * math.sin(2 * math.pi * i / 50) + (3 if i % 2 else -3)))
                 for i in range(900)]
        self.assertAlmostEqual(proto.estimate_period(noisy), 50.0, delta=0.5)

    def test_refuses_what_it_cannot_know(self):
        self.assertIsNone(proto.estimate_period([128] * 900), "flat")
        self.assertIsNone(proto.estimate_period([127, 128, 129, 128] * 225), "span below the floor")
        one = [200 if 300 <= i < 600 else 50 for i in range(900)]
        self.assertIsNone(proto.estimate_period(one), "one edge is not a period")
        two = [200 if (300 <= i < 400 or 500 <= i < 600) else 50 for i in range(900)]
        self.assertIsNone(proto.estimate_period(two), "two rising crossings: one period")

    # stage-255 (G-240 host side)
    @staticmethod
    def seamed_square(cut=10, at=405, n=1024):
        sq = [200 if (i // 25) % 2 else 50 for i in range(n + cut)]
        return sq[:at] + sq[at + cut:]          # a seam inside a low phase: one period loses `cut` samples

    def test_seam_cut_period_does_not_bias(self):
        x = self.seamed_square()
        self.assertEqual(proto.estimate_period(x), 50.0)
        p = proto.single_periods(x)
        self.assertEqual(sum(1 for v in p if abs(v - 50) > 1), 1, "exactly one cut period")
        # negative control: the stage-250 estimate (last - first) / (n - 1) is biased
        xs = [i for i in range(1, len(x)) if x[i - 1] < 125 <= x[i]]
        self.assertGreater(abs((xs[-1] - xs[0]) / (len(xs) - 1) / 50.0 - 1), 0.005)    # 1 %

    def test_small_noisy_sine_with_hysteresis(self):
        import math
        import random
        rnd = random.Random(7)
        for counts in (10, 20):
            x = [int(round(128 + counts / 2 * math.sin(2 * math.pi * i / 37.3) + rnd.randint(-1, 1)))
                 for i in range(900)]
            self.assertAlmostEqual(proto.estimate_period(x) / 37.3, 1.0, delta=0.01, msg=f"{counts} counts")

    def test_summary_names_the_period_method(self):
        x = bytes(self.seamed_square())
        for flags, method in ((proto.WAVE_FLAG_TB_MEASURED, "median_filtered"),
                              (proto.WAVE_FLAG_TB_MEASURED | proto.WAVE_FLAG_TIME_ORDERED, "time_ordered")):
            w = proto.Waveform(1, 0, flags, 0x10, 6, len(x), 24, 12500, 0, 32, 0, x)
            sm = w.summary()
            self.assertEqual(sm["period_method"], method)
            self.assertEqual(sm["period_samples"], 50.0)
            self.assertEqual(sm["frequency_hz"], 250.0)
            if method == "median_filtered":
                self.assertIn("not time-ordered", sm["period_note"])
                self.assertIn("(19 of 20 kept)", sm["period_note"])
            else:
                self.assertNotIn("period_note", sm)
            from openscope import cli
            self.assertEqual("(seam-filtered)" in cli._wave_line(w), method == "median_filtered")
        flat = proto.Waveform(1, 0, 0, 0x10, 6, 900, 24, 12500, 0, 32, 0, bytes([128] * 900))
        sm = flat.summary()
        self.assertNotIn("period_method", sm)
        self.assertIn("no repeating crossings", sm["period_note"])


class TestButtons(unittest.TestCase):
    def test_names_and_ids(self):
        self.assertEqual(proto.button_id("menu"), 9)
        self.assertEqual(proto.button_id("15"), 15)
        for bad in ("FOO", "0", "16", 99):
            with self.assertRaises(ValueError):
                proto.button_id(bad)


class TestGuardsAreLoadBearing(unittest.TestCase):
    def test_checksum_comparison_is_what_rejects(self):
        import inspect
        src = inspect.getsource(proto.Decoder.feed)
        guard = "if checksum(body) != self._buf[total - 1]:"
        self.assertIn(guard, src, "guard moved; update this mutation")
        ns = {}
        mutated = inspect.getsource(proto).replace(guard, "if False:")
        exec(compile(mutated, "proto_mutant", "exec"), ns)
        raw = bytearray(proto.encode(0x83, b"hello"))
        raw[-1] ^= 0xFF
        self.assertEqual(len(ns["Decoder"]().feed(bytes(raw))), 1,
                         "mutant should accept the corrupted frame the real decoder rejects")


def scope_state_payload(**kw):
    f = dict(ver=1, length=36, mode=0, running=1, tb_idx=0x10, tb_code=0x10, tb_tier=2, tmode=0,
             tedge=1, tsrc=0, level=-20, rate=12490, tcode=0x5A, cpd=32)
    f.update(kw)
    head = struct.pack("<BBBBBBBBBBhIBBH", f["ver"], f["length"], f["mode"], f["running"], f["tb_idx"],
                       f["tb_code"], f["tb_tier"], f["tmode"], f["tedge"], f["tsrc"], f["level"],
                       f["rate"], f["tcode"], 0, f["cpd"])
    return head + struct.pack("<BBBBI", 1, 6, 2, 0, 42950) + struct.pack("<BBBBI", 0, 8, 1, 0, 279050)


class TestControlSurface(unittest.TestCase):
    def test_scope_state_roundtrip(self):
        s = proto.parse_scope_state(scope_state_payload())
        self.assertEqual((s.mode_name, s.running, s.timebase_code, s.sample_rate_hz), ("scope", True, 0x10, 12490))
        self.assertEqual((s.trig_mode_name, s.trig_edge_name, s.trig_source_name, s.trig_level, s.trig_code),
                         ("auto", "falling", "CH1", -20, 0x5A))
        self.assertFalse(s.timebase_disagrees)
        self.assertEqual((s.ch[0].enabled, s.ch[0].vdiv_idx, s.ch[0].tier_name, s.ch[0].uv_per_div),
                         (True, 6, "measured", 42950))
        self.assertEqual((s.ch[1].enabled, s.ch[1].vdiv_idx, s.ch[1].tier_name), (False, 8, "provisional"))

    def test_divergence_is_visible(self):
        self.assertTrue(proto.parse_scope_state(scope_state_payload(tb_idx=0x0A, tb_code=0x08)).timebase_disagrees)

    def test_appended_fields_are_skipped_not_refused(self):
        p = bytearray(scope_state_payload()) + b"\x01\x02"
        p[1] = len(p)
        self.assertEqual(proto.parse_scope_state(bytes(p)).timebase_code, 0x10)

    def test_bad_scope_state_refused(self):
        good = scope_state_payload()
        for bad in (good[:35], good[:1], scope_state_payload(ver=2), scope_state_payload(length=40),
                    good + b"\x00"):                       # length byte no longer matches
            with self.assertRaises(proto.ProtocolError):
                proto.parse_scope_state(bad)

    def test_set_payloads(self):
        self.assertEqual(proto.payload_set_timebase(0x14), b"\x14")
        self.assertEqual(proto.payload_set_vdiv(2, 9), b"\x02\x09")
        self.assertEqual(proto.payload_set_trigger("normal", "falling", "ch2", -100), b"\x01\x01\x01\x9c\xff")
        self.assertEqual(proto.payload_set_trigger(2, 0, 0, 100), b"\x02\x00\x00\x64\x00")
        self.assertEqual(proto.payload_run_stop(False), b"\x00")
        self.assertEqual(proto.payload_set_mode("SETTINGS"), b"\x03")

    def test_out_of_range_never_encoded(self):
        for fn, args in ((proto.payload_set_timebase, (21,)), (proto.payload_set_timebase, (-1,)),
                         (proto.payload_set_vdiv, (3, 1)), (proto.payload_set_vdiv, (1, 10)),
                         (proto.payload_set_trigger, ("rolling",)), (proto.payload_set_trigger, ("auto", "up")),
                         (proto.payload_set_trigger, ("auto", "rising", "ch3")),
                         (proto.payload_set_trigger, ("auto", "rising", "ch1", 101)),
                         (proto.payload_set_mode, ("dmm",)), (proto.payload_set_mode, (4,))):
            with self.assertRaises(ValueError, msg=f"{fn.__name__}{args}"):
                fn(*args)


def sample_description(**kw) -> "proto.Description":
    rates = [(0, 0)] * 21
    rates[0x0D] = (119678, 1)
    rates[0x0E], rates[0x0F], rates[0x10] = (49930, 2), (24979, 2), (12490, 2)
    ranges = tuple(tuple((0, 0) if r < 4 else (1000 * (ch + 1) * 100 + r, 2 if r in (5, 6, 7) else 1)
                         for r in range(10)) for ch in range(2))
    f = dict(version=1, samples_per_div=32, rates=tuple(rates), counts_per_div=32, ranges=ranges,
             trigger=proto.TriggerScale(-100, 100, 128, 256, 206, 1, 254, -28), probe=(10, 1))
    f.update(kw)
    return proto.Description(**f)


class TestDescription(unittest.TestCase):
    """stage-086: GET_DESCRIPTION sections, skip rule, helpers, probe append."""

    def test_round_trip_of_each_section(self):
        d = sample_description()
        p = proto.encode_description(d)
        self.assertEqual(len(p), 239)              # the firmware's ESP_DESCRIPTION_LEN
        self.assertEqual(proto.parse_description(p), d)
        for drop in ("samples_per_div", "counts_per_div", "trigger", "probe"):  # each section alone missing
            part = sample_description(**{drop: None})
            back = proto.parse_description(proto.encode_description(part))
            self.assertEqual(back, sample_description(**{drop: None, **(
                {"rates": ()} if drop == "samples_per_div" else {"ranges": ()} if drop == "counts_per_div" else {})}))

    def test_unknown_section_is_skipped(self):
        p = bytearray(proto.encode_description(sample_description()))
        extra = struct.pack("<BH", 9, 4) + b"\xde\xad\xbe\xef"
        q = bytes(p[:1]) + extra + bytes(p[1:]) + struct.pack("<BH", 200, 0)
        d = proto.parse_description(q)
        self.assertEqual(d.skipped, (9, 200))
        self.assertEqual(d.sample_rate(0x10), 12490)
        self.assertEqual(d.probe, (10, 1))

    def test_appended_fields_inside_a_section_are_ignored(self):
        t = proto.encode_description(sample_description(samples_per_div=None, counts_per_div=None, probe=None))
        sec = bytearray(t[1:])
        sec[1:3] = struct.pack("<H", 12 + 2)           # trigger section grows by 2 bytes
        d = proto.parse_description(bytes(t[:1]) + bytes(sec) + b"\x00\x00")
        self.assertEqual(d.trigger.code(0), 128)

    def test_malformed_refused(self):
        good = proto.encode_description(sample_description())
        for bad in (b"", b"\x02" + good[1:], good[:-1], good + b"\x04",
                    good[:1] + struct.pack("<BHHB", 1, 3, 32, 21)):   # 21 entries announced, none sent
            with self.assertRaises(proto.ProtocolError, msg=bad[:8].hex()):
                proto.parse_description(bad)

    def test_tier_none_value_is_no_value(self):
        rates = [(0, 0)] * 21
        rates[3] = (5000, 0)                            # a value its tier disowns
        d = proto.parse_description(proto.encode_description(sample_description(rates=tuple(rates))))
        self.assertIsNone(d.sample_rate(3))
        self.assertEqual(d.tier("timebase", 3), "none")

    def test_helpers_in_practical_units(self):
        d = sample_description()
        self.assertAlmostEqual(d.time_per_div(0x10), 32 / 12490)
        self.assertIsNone(d.time_per_div(0x08))
        self.assertEqual(d.tier("timebase", 0x0D), "provisional")
        self.assertEqual(d.tier("range", 2, 6), "measured")
        self.assertAlmostEqual(d.volts_per_div(1, 6, probe=1), 0.100006)
        self.assertAlmostEqual(d.volts_per_div(1, 6), 1.00006)          # described probe x10
        self.assertAlmostEqual(d.volts_per_div(2, 6), 0.200006)         # CH2 x1
        self.assertIsNone(d.volts_per_div(1, 2))                         # railing range
        self.assertAlmostEqual(d.volts_per_level(2, 6), 0.200006 / 32 * 256 / 206)
        self.assertEqual(proto.format_seconds(d.time_per_div(0x10)), "2.56 ms")
        self.assertEqual(proto.format_seconds(None), "not calibrated")
        self.assertEqual(proto.format_volts(1.00006), "1 V")
        self.assertEqual(proto.format_volts(0.0429), "42.9 mV")

    # ── stage-190: section 7 identity ──
    UID = bytes.fromhex("0E927403" "0080006E" "2094CC07")   # words 0374920E 6E008000 07CC9420 (unit #1)

    def test_identity_round_trip(self):
        ident = proto.Identity(self.UID, 1, "guest-coldtrace", "0a1af35-dirty")
        d = sample_description(identity=ident)
        p = proto.encode_description(d)
        self.assertEqual(len(p), 239 + 3 + 12 + 2 + 16 + 14)    # what the firmware sends
        back = proto.parse_description(p)
        self.assertEqual(back, d)
        self.assertEqual(back.api_level, 1)
        self.assertEqual(back.skipped, ())
        self.assertEqual(back.identity.uid_words, (0x0374920E, 0x6E008000, 0x07CC9420))
        self.assertEqual(back.identity.uid_hex, "0374920E6E00800007CC9420")
        self.assertEqual(back.identity.usb_serial, "0B41262E6E00")   # the unit's Windows serial

    def test_old_description_is_api_level_0(self):
        d = proto.parse_description(proto.encode_description(sample_description()))
        self.assertIsNone(d.identity)
        self.assertEqual(d.api_level, 0)

    def test_identity_texts_cut_and_empty(self):
        ident = proto.Identity(self.UID, 7, "t" * 40, "")
        back = proto.parse_description(proto.encode_description(sample_description(identity=ident)))
        self.assertEqual(back.identity.target, "t" * 31)
        self.assertEqual(back.identity.git_rev, "")
        self.assertEqual(back.api_level, 7)

    def test_identity_appended_field_ignored_and_malformed_refused(self):
        sec = self.UID + struct.pack("<H", 2) + b"\x01x\x02ab" + b"\x99\x99"   # a later field
        p = bytes([1]) + struct.pack("<BH", 7, len(sec)) + sec
        i = proto.parse_description(p).identity
        self.assertEqual((i.api_level, i.target, i.git_rev), (2, "x", "ab"))
        for bad in (self.UID, self.UID + b"\x01\x00", self.UID + b"\x01\x00\x05ab",
                    self.UID + b"\x01\x00\x01x"):                     # short, no target, cut, no rev
            with self.assertRaises(proto.ProtocolError, msg=bad.hex()):
                proto.parse_description(bytes([1]) + struct.pack("<BH", 7, len(bad)) + bad)

    def test_trigger_scale_matches_the_firmware_formula(self):
        t = sample_description().trigger
        # fpga_trigger_code_from_level(): C division truncates toward zero
        self.assertEqual([t.code(v) for v in (-100, -50, -1, 0, 1, 50, 100, 150, -150)],
                         [4, 66, 127, 128, 129, 190, 252, 252, 4])
        self.assertEqual(t.record_counts(0), 100)

    def test_probe_payload(self):
        self.assertEqual(proto.payload_set_probe(1, 10), b"\x01\x0a")
        self.assertEqual(proto.payload_set_probe(2, 1), b"\x02\x01")
        for args in ((0, 1), (3, 1), (1, 2), (1, 0), (1, 100)):
            with self.assertRaises(ValueError):
                proto.payload_set_probe(*args)

    def test_scope_state_probe_append(self):
        old = scope_state_payload()                     # stage-085 firmware: 36 bytes, no probe
        self.assertIsNone(proto.parse_scope_state(old).probe)
        new = bytearray(old) + b"\x0a\x01"
        new[1] = len(new)
        s = proto.parse_scope_state(bytes(new))
        self.assertEqual(s.probe, (10, 1))
        self.assertEqual(s.ch[0].uv_per_div, 42950)     # uv_per_div keeps its meaning


METER_11 = tuple(proto.MeterFunction(i, i in (0, 1, 6, 7, 8), n, u) for i, (n, u) in enumerate((
    ("DC Voltage", "V"), ("AC Voltage", "V"), ("DC mA", "mA"), ("DC Current", "A"), ("AC mA", "mA"),
    ("AC Current", "A"), ("Resistance", "kOhm"), ("Continuity", "Ohm"), ("Diode", "V"),
    ("Capacitance", "nF"), ("Temperature", "C"))))


class TestMeterFunctions(unittest.TestCase):
    """stage-272: DESCRIPTION section 5 and SET_METER_MODE."""

    def test_section_5_round_trip(self):
        d = sample_description(meter_functions=METER_11)
        p = proto.encode_description(d)
        back = proto.parse_description(p)
        self.assertEqual(back, d)
        self.assertEqual(back.skipped, ())
        self.assertEqual([f.name for f in back.meter_functions if f.usable],
                         ["DC Voltage", "AC Voltage", "Resistance", "Continuity", "Diode"])

    def test_old_firmware_lists_no_functions(self):
        d = proto.parse_description(proto.encode_description(sample_description()))
        self.assertEqual(d.meter_functions, ())
        with self.assertRaisesRegex(ValueError, "does not list meter functions"):
            d.meter_function("dc-voltage")
        self.assertEqual(d.meter_function_name(6), "function 6")

    def test_name_matching_ignores_case_dash_underscore_space(self):
        d = sample_description(meter_functions=METER_11)
        for name in ("dc-voltage", "DC Voltage", "dc_voltage", "DCVOLTAGE", " dc voltage ", 0, "0"):
            self.assertEqual(d.meter_function(name).id, 0, name)
        self.assertEqual(d.meter_function("resistance").unit, "kOhm")
        self.assertEqual(d.meter_function("ac-ma").id, 4)
        self.assertEqual(d.meter_function(10).name, "Temperature")
        for bad in ("voltage", "dc", "ohm", 11, "11", -1, True):
            with self.assertRaises(ValueError, msg=repr(bad)):
                d.meter_function(bad)
        with self.assertRaisesRegex(ValueError, "dc-voltage, ac-voltage, dc-ma"):
            d.meter_function("nope")
        self.assertEqual(METER_11[3].cli_name, "dc-current")
        self.assertEqual(d.meter_function_name(7), "Continuity")
        self.assertEqual(d.meter_function_name(99), "function 99")

    def test_payload_set_meter_mode(self):
        self.assertEqual(proto.payload_set_meter_mode(0), b"\x00")
        self.assertEqual(proto.payload_set_meter_mode(10), b"\x0a")
        for bad in (11, -1, 255, "1", 1.0, True, None):
            with self.assertRaises(ValueError, msg=repr(bad)):
                proto.payload_set_meter_mode(bad)

    def test_texts_cut_appended_bytes_ignored_malformed_refused(self):
        long = (proto.MeterFunction(0, True, "N" * 20, "U" * 9),)
        back = proto.parse_description(proto.encode_description(sample_description(meter_functions=long)))
        self.assertEqual((back.meter_functions[0].name, back.meter_functions[0].unit), ("N" * 15, "U" * 7))
        sec = b"\x01\x00\x01\x02ab\x01V" + b"\x99"      # a later field after the list
        f = proto.parse_description(bytes([1]) + struct.pack("<BH", 5, len(sec)) + sec).meter_functions
        self.assertEqual(f, (proto.MeterFunction(0, True, "ab", "V"),))
        for bad in (b"", b"\x01", b"\x01\x00\x01", b"\x01\x00\x01\x05ab", b"\x01\x00\x01\x01a",
                    b"\x02\x00\x01\x01a\x00"):        # empty, no entry, no name len, cut, no unit, 2nd missing
            with self.assertRaises(proto.ProtocolError, msg=bad.hex()):
                proto.parse_description(bytes([1]) + struct.pack("<BH", 5, len(bad)) + bad)


if __name__ == "__main__":
    unittest.main(verbosity=2)
