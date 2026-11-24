#!/usr/bin/env python3
"""Build and run the CDC firmware loader host tests, then prove the stage-167
slot check can fail.

WHAT IT RUNS
------------
firmware/tests/test_fw_loader.c against the real fw_loader.c (FW_LOADER_HOST_TEST:
the W25Q is a 2 MB array, the RAM installer is compiled out). Same build as
`make test-fw-loader`, plus -Werror.

WHY THE MUTATIONS
-----------------
Same reason as scripts/test_config_persist.py: a guard test passes whether or
not the guard does anything. Each mutation takes a copy of fw_loader.c, deletes
one part of the slot check (fw_loader_begin() erases a W25Q slot only when each
byte its staging will erase is ours or 0xFF), rebuilds, and REQUIRES the named
C checks to go red. Literal substitutions: a target string that is no longer
found exactly once fails this suite instead of silently skipping the check.
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
FW = REPO / "firmware"
TEST_C = FW / "tests" / "test_fw_loader.c"
SRC_REL = "src/drivers/fw_loader.c"
CFLAGS = ["-std=gnu11", "-Wall", "-Wextra", "-Werror", "-DFW_LOADER_HOST_TEST",
          "-I", str(FW / "src" / "drivers")]


def build_and_run(workdir: Path, src_text: str | None = None) -> subprocess.CompletedProcess:
    """Compile (optionally with fw_loader.c replaced by mutated text) and run.
    Never raises on a non-zero exit: red is the expected result for a mutant."""
    src = FW / SRC_REL
    if src_text is not None:
        src = workdir / "fw_loader.c"
        src.write_text(src_text)
    binary = workdir / "test_fw_loader"
    built = subprocess.run(["gcc", *CFLAGS, str(TEST_C), str(src), "-o", str(binary)],
                           capture_output=True, text=True)
    if built.returncode != 0:
        return built
    return subprocess.run([str(binary)], capture_output=True, text=True)


def failing_lines(output: str) -> list[str]:
    return [line for line in output.splitlines() if line.startswith("FAIL ")]


def failed(label: str, output: str) -> bool:
    return any(label in line for line in failing_lines(output))


@dataclass
class Mutation:
    """One part of the slot check, deleted. `expect_fail`: C messages that must go red."""

    name: str
    old: str
    new: str
    expect_fail: tuple[str, ...] = ()


MUTATIONS: tuple[Mutation, ...] = (
    Mutation(
        name="foreign-byte refusal",
        old="                fwl_dirty_off = off + i;\n                return FWL_SLOT_FOREIGN;\n",
        new="                fwl_dirty_off = off + i;\n                return FWL_SLOT_OURS;\n",
        expect_fail=("foreign byte in data span: begin must refuse",
                     "FAT boot sector at slot base: begin must refuse",
                     "foreign byte past an old FWC1 image: begin must refuse"),
    ),
    Mutation(
        name="erase span rounded up to the last erased sector",
        old="    uint32_t end = fwl_sector_roundup(FWL_DATA_OFF + size);\n",
        new="    uint32_t end = FWL_DATA_OFF + size;\n",
        expect_fail=("byte in the last erased sector: begin must refuse",),
    ),
    Mutation(
        name="FWCP marker accepted as ours",
        old="    if ((m.magic == FWL_MANIFEST_MAGIC || m.magic == FWL_STAGING_MAGIC) &&\n",
        new="    if ((m.magic == FWL_MANIFEST_MAGIC) &&\n",
        expect_fail=("re-stage over a torn staging of ours: begin must accept",),
    ),
    Mutation(
        name="FWCP marker written before the first data erase",
        old="        fwl_manifest_t mk = { FWL_STAGING_MAGIC, fwl_expected,\n",
        new="        fwl_manifest_t mk = { 0xFFFFFFFFu, 0xFFFFFFFFu,\n",
        expect_fail=("the first chunk must write the FWCP marker",
                     "re-stage over a torn staging of ours: begin must accept"),
    ),
    Mutation(
        name="no erase before the first data chunk",
        old="    fwl_erase_mark = fwl_slot_base(slot); /* manifest sector erased too:\n",
        new="    fwl_erase_mark = fwl_slot_base(slot);"
            " (void)fwl_w25q_write(fwl_erase_mark, (const uint8_t *)\"FWCP\", 4); /* mutant\n",
        expect_fail=("begin with no data must change nothing",),
    ),
    Mutation(
        name="tail check past an old image",
        old="        off = fwl_sector_roundup(FWL_DATA_OFF + m.size);\n",
        new="        off = end;\n",
        expect_fail=("foreign byte past an old FWC1 image: begin must refuse",),
    ),
    Mutation(
        name="manifest size check in the slot check",
        old="        fwl_manifest_size_ok(m.size)) {\n",
        new="        1) {\n",
        expect_fail=("FWCP with size 1: begin must refuse",),
    ),
)


class FwLoaderHostTests(unittest.TestCase):
    main: subprocess.CompletedProcess

    @classmethod
    def setUpClass(cls) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            cls.main = build_and_run(Path(tmp))

    def test_main_build_passes(self) -> None:
        self.assertEqual(self.main.returncode, 0,
                         f"fw_loader host tests failed:\n{self.main.stdout}\n{self.main.stderr}")
        self.assertIn("all fw_loader tests passed", self.main.stdout)
        self.assertEqual(failing_lines(self.main.stdout), [])

    def test_slot_check_is_present(self) -> None:
        """Guard against the slot cases quietly leaving the C suite."""
        text = TEST_C.read_text()
        self.assertIn("test_slot_check();", text)
        self.assertGreaterEqual(text.count("expect_refused("), 6)


class SlotCheckMutationTests(unittest.TestCase):
    """Each part of the slot check, deleted in turn. The C suite must notice."""

    def _mutate(self, m: Mutation) -> None:
        original = (FW / SRC_REL).read_text()          # universal newlines: "\n"
        self.assertEqual(
            original.count(m.old), 1,
            f"mutation target for {m.name!r} appears {original.count(m.old)} times in "
            f"{SRC_REL} (expected exactly 1). The code moved; update the mutation "
            f"rather than dropping the check.")
        with tempfile.TemporaryDirectory() as tmp:
            proc = build_and_run(Path(tmp), original.replace(m.old, m.new))
        self.assertNotIn("error:", proc.stderr,
                         f"the {m.name} mutant did not build:\n{proc.stderr}")
        self.assertNotEqual(proc.returncode, 0,
                            f"removing the {m.name} did NOT make the tests fail. "
                            f"It is untested:\n{proc.stdout}")
        for label in m.expect_fail:
            self.assertTrue(failed(label, proc.stdout),
                            f"removing the {m.name} should have failed {label!r}; "
                            f"failures were {failing_lines(proc.stdout)}")


def _make_case(m: Mutation):
    def case(self: SlotCheckMutationTests) -> None:
        self._mutate(m)
    case.__doc__ = f"deleting the {m.name} must turn the suite red"
    return case


for _i, _m in enumerate(MUTATIONS):
    _slug = "".join(c if c.isalnum() else "_" for c in _m.name).strip("_").lower()
    setattr(SlotCheckMutationTests, f"test_mutation_{_i:02d}_{_slug}", _make_case(_m))


if __name__ == "__main__":
    if shutil.which("gcc") is None:
        print("gcc not found: the fw_loader host tests cannot run", file=sys.stderr)
        raise SystemExit(1)
    unittest.main(verbosity=2)
