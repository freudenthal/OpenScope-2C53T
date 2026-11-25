#!/usr/bin/env python3
"""Build and run the factory-cal-backup core host tests, then prove they can fail.

WHY THE SECOND HALF EXISTS
--------------------------
This project's recurring failure mode is a guard that is never seen to fail —
it passes whether or not it is doing anything, and nobody notices until the day
it is needed. cal_backup's whole job is to be trusted with an irreplaceable 4 KB
of per-device calibration, so its record validation and its "never overwrite a
programmed page" decision are exactly the guards that must be shown to bite.

So this suite does not only run firmware/tests/test_cal_backup.c against the
real cal_backup_core.c. For each guard it takes a copy of the source, deletes
that guard, rebuilds, and REQUIRES the C suite to go red. A mutation that still
passes is reported as a failure here.

The mutations are literal source substitutions. If a substitution string is no
longer found — because the code was refactored — this suite fails loudly rather
than silently skipping the check.
"""

from __future__ import annotations

import shutil
import subprocess
import sys
import tempfile
import unittest
from dataclasses import dataclass, field
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
SRC = REPO / "firmware" / "src" / "util" / "cal_backup_core.c"
HDR = REPO / "firmware" / "src" / "util" / "cal_backup_core.h"
TEST_C = REPO / "firmware" / "tests" / "test_cal_backup.c"

CFLAGS = ["-std=c11", "-Wall", "-Wextra", "-Werror", "-O1"]


@dataclass
class Mutation:
    """One guard, weakened. `expect_fail` = the run must exit non-zero."""

    name: str
    old: str
    new: str
    expect_fail: bool = True


MUTATIONS: tuple[Mutation, ...] = (
    Mutation(
        name='magic check dropped -> blank/foreign region misread as a record',
        old='if (h.magic != CAL_BACKUP_MAGIC) {\n        return CAL_REC_ERR_MAGIC;\n    }',
        new='if (0) {\n        return CAL_REC_ERR_MAGIC;\n    }',
    ),
    Mutation(
        name='header CRC check dropped -> corrupt header trusted',
        old='if (header_crc_of(rec, h.version) != h.header_crc) {\n        return CAL_REC_ERR_HEADER_CRC;\n    }',
        new='if (0) {\n        return CAL_REC_ERR_HEADER_CRC;\n    }',
    ),
    Mutation(
        name='v2 header CRC does not cover the UID and block CRC',
        old='c = cal_backup_crc32_update(c, h + 20, CAL_BACKUP_HEADER_LEN_V2 - 20u);',
        new='(void)h;',
    ),
    Mutation(
        name='v2 header length check dropped -> a cut header is read past its end',
        old='if (rec_len < h.header_len) {\n        return CAL_REC_ERR_TRUNCATED;',
        new='if (0) {\n        return CAL_REC_ERR_TRUNCATED;',
    ),
    Mutation(
        name='payload CRC check dropped -> corrupt payload restored',
        old='if (cal_backup_crc32(payload, h.payload_len) != h.payload_crc) {\n        return CAL_REC_ERR_PAYLOAD_CRC;\n    }',
        new='if (0) {\n        return CAL_REC_ERR_PAYLOAD_CRC;\n    }',
    ),
    Mutation(
        name='parse: v2 block CRC check dropped',
        old='cal_backup_crc32(payload + CAL_BLOCK_OFF, CAL_BLOCK_LEN) != h.block_crc',
        new='0',
    ),
    Mutation(
        name='version check dropped -> unknown future record misread',
        old='if (h.header_len == 0u) {\n        return CAL_REC_ERR_VERSION;\n    }',
        new='if (0) {\n        return CAL_REC_ERR_VERSION;\n    }',
    ),
    Mutation(
        name='auto-restore no longer refuses a programmed page',
        old='return live == CAL_PAGE_BLANK || live == CAL_PAGE_ZEROED;',
        new='return true;',
    ),
    Mutation(
        name='page classification collapses PROGRAMMED into BLANK',
        old='if (page[i] != 0xFFu) { all_ff = false; }',
        new='if (0) { all_ff = false; }',
    ),
    Mutation(
        name='live cal-block class taken from the whole page',
        old='out->block_class = cal_backup_classify_page(page + CAL_BLOCK_OFF, CAL_BLOCK_LEN);',
        new='out->block_class = out->page_class;',
    ),
    Mutation(
        name='scan: a torn header (magic blank, rest programmed) counts as an empty slot',
        old='all_bytes(head, n, 0xFFu)',
        new='all_bytes(head, 4u, 0xFFu)',
    ),
    Mutation(
        name='scan: payload length other than 4 KB accepted',
        old='if (s->info.hdr.payload_len != CAL_BACKUP_PAYLOAD_LEN) {',
        new='if (0) {',
    ),
    Mutation(
        name='scan: payload CRC check dropped',
        old='} else if (crc != s->info.hdr.payload_crc) {',
        new='} else if (0) {',
    ),
    Mutation(
        name='scan: v2 block CRC check dropped',
        old='} else if (s->info.hdr.version == CAL_BACKUP_VERSION_2 &&\n                   bcrc != s->info.hdr.block_crc) {',
        new='} else if (0) {',
    ),
    Mutation(
        name='scan: cal-block byte compare dropped -> a changed cal block reads equal',
        old='if (s->live != NULL && memcmp(b, s->live + lo, bn) != 0) {\n            s->block_eq = false;',
        new='if (0) {\n            s->block_eq = false;',
    ),
    Mutation(
        name='store: blank/zeroed live page backed up',
        old='if (live->page_class != CAL_PAGE_PROGRAMMED) {\n        return plan(CAL_PLAN_REFUSE, CAL_WHY_LIVE_BLANK);',
        new='if (0) {\n        return plan(CAL_PLAN_REFUSE, CAL_WHY_LIVE_BLANK);',
    ),
    Mutation(
        name='store: blank/zeroed live cal block backed up',
        old='if (live->block_class != CAL_PAGE_PROGRAMMED) {\n        return plan(CAL_PLAN_REFUSE, CAL_WHY_LIVE_BLOCK_BLANK);',
        new='if (0) {\n        return plan(CAL_PLAN_REFUSE, CAL_WHY_LIVE_BLOCK_BLANK);',
    ),
    Mutation(
        name='store: slot A written when it is not empty',
        old='if (a->state == CAL_SLOT_EMPTY) {',
        new='if (a->state != CAL_SLOT_VALID || force) {',
    ),
    Mutation(
        name='store: damaged slot A not refused',
        old='if (a->state != CAL_SLOT_VALID) {\n        return plan(CAL_PLAN_REFUSE, CAL_WHY_A_DAMAGED);',
        new='if (0) {\n        return plan(CAL_PLAN_REFUSE, CAL_WHY_A_DAMAGED);',
    ),
    Mutation(
        name="store: another unit's slot A not refused",
        old='if (uid_mismatch(&a->hdr, uid)) {\n        return plan(CAL_PLAN_REFUSE, CAL_WHY_UID_MISMATCH);',
        new='if (0) {\n        return plan(CAL_PLAN_REFUSE, CAL_WHY_UID_MISMATCH);',
    ),
    Mutation(
        name='store: a different cal block not refused',
        old='if (!a->block_equal_live && !force) {',
        new='if (0) {',
    ),
    Mutation(
        name='store: force ignored',
        old='if (!a->block_equal_live && !force) {',
        new='if (!a->block_equal_live) {',
    ),
    Mutation(
        name='store: slot B rewritten when it already holds this page',
        old='if (b->state == CAL_SLOT_VALID && b->page_equal_live && !uid_mismatch(&b->hdr, uid)) {',
        new='if (0) {',
    ),
    Mutation(
        name='restore: no valid source not refused',
        old='if (src->state != CAL_SLOT_VALID) {',
        new='if (0) {',
    ),
    Mutation(
        name="restore: another unit's backup not refused",
        old='if (uid_mismatch(&src->hdr, uid)) {',
        new='if (0) {',
    ),
    Mutation(
        name='restore: blank/zeroed backup cal block not refused',
        old='if (src->block_class != CAL_PAGE_PROGRAMMED) {',
        new='if (0) {',
    ),
    Mutation(
        name='restore: programmed live page overwritten without force',
        old='if (live->page_class == CAL_PAGE_PROGRAMMED && !force) {',
        new='if (0) {',
    ),
)


def compile_and_run(source_text: str, workdir: Path) -> subprocess.CompletedProcess:
    (workdir / "cal_backup_core.c").write_text(source_text)
    shutil.copy(HDR, workdir / "cal_backup_core.h")
    shutil.copy(TEST_C, workdir / "test_cal_backup.c")
    binary = workdir / "t"
    build = subprocess.run(
        ["gcc", *CFLAGS, "-o", str(binary),
         str(workdir / "test_cal_backup.c"), str(workdir / "cal_backup_core.c"),
         "-I", str(workdir)],
        capture_output=True, text=True,
    )
    if build.returncode != 0:
        return build
    return subprocess.run([str(binary)], capture_output=True, text=True)


class TestCalBackupCore(unittest.TestCase):
    def test_source_files_present(self):
        for path in (SRC, HDR, TEST_C):
            self.assertTrue(path.is_file(), f"missing {path}")

    def test_base_suite_passes(self):
        with tempfile.TemporaryDirectory() as tmp:
            proc = compile_and_run(SRC.read_text(), Path(tmp))
            self.assertEqual(proc.returncode, 0,
                             f"base suite failed:\n{proc.stdout}\n{proc.stderr}")

    def test_mutations_are_caught(self):
        base = SRC.read_text()
        for m in MUTATIONS:
            with self.subTest(mutation=m.name):
                self.assertIn(m.old, base,
                              f"mutation string not found (refactor?): {m.name}")
                mutated = base.replace(m.old, m.new, 1)
                self.assertNotEqual(mutated, base, "replacement was a no-op")
                with tempfile.TemporaryDirectory() as tmp:
                    proc = compile_and_run(mutated, Path(tmp))
                if m.expect_fail:
                    self.assertNotEqual(
                        proc.returncode, 0,
                        f"mutation '{m.name}' did NOT make the suite fail — "
                        f"the guard is not tested.\n{proc.stdout}")


if __name__ == "__main__":
    if shutil.which("gcc") is None:
        print("gcc not found: the cal-backup host tests cannot run", file=sys.stderr)
        sys.exit(0)
    unittest.main(verbosity=2)
