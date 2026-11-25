"""Firmware images for the factory IAP drive: manifests and checks.

A manifest is a small JSON file next to an image (<image>.manifest.json).
It records what the image is (target, build time, git revision, API level)
and its SHA-256, so that a host can refuse a wrong or damaged file before
the copy, and can check after the update that the unit runs the same build.

Stable API (the update flow and the GUI call these):

    Manifest(...)                    fields below; to_json(), from_json(),
                                     load(path), save(path)
    read_build_info(data) -> BuildInfo
        The build time and the git revision, read from the image bytes
        (the STATUS fw_version text and the shell "Build:" line).
    make_manifest(data, target, api_level=None, version=None, source=None,
                  product_commit=None, image_name=None) -> Manifest
    check_image(data, manifest, allow_unlisted=False) -> [refusal, ...]
        Empty list = the image may go to the unit.
    check_running(manifest, fw_version, identity) -> [difference, ...]
        Empty list = the unit runs the build of the manifest.

Builds are not reproducible (the image holds __DATE__ __TIME__): a rebuild
of the same commit has a new SHA-256 and a new build time.
"""
from __future__ import annotations

import hashlib
import json
import re
import struct
from dataclasses import asdict, dataclass, fields
from pathlib import Path
from typing import List, Optional, Union

GUEST_BASE = 0x08007000            # factory IAP jumps here (guest link address)
FLASH_CEILING = 0x080FF800         # old startup-mode record page: the image must end before it
MAX_SIZE = FLASH_CEILING - GUEST_BASE          # 1,017,856 B
SP_RANGE = (0x20000000, 0x20038000)            # initial SP: inside the 224 KB SRAM (end inclusive)
MANIFEST_SUFFIX = ".manifest.json"
FORMAT = 1

# Images that must never go through this tool. Stock goes back through
# stage-205's return-to-stock path, which has its own checks.
KNOWN_STOCK = {
    "a17c5c35c97bb898f15672a1747bc1041d8ed507c16999ddba0d1e4e2ec0c760":
        "FNIRSI stock firmware V1.2.0 (APP_2C53T_V1.2.0_251015.bin)",
}

TARGET_RE = re.compile(r"^guest(?:-[a-z0-9]+)*$")
TEST_ONLY_TARGETS = ("faulttest", "caldump")   # never leave a unit on these (testing.md C-180)

_FW_TEXT = re.compile(rb"OpenScope ([A-Z][a-z]{2} [ \d]\d \d{4}) (\d\d:\d\d:\d\d)"
                      rb"(?: ([0-9a-f]{7,40}(?:-dirty)?|unknown))?(?=\x00)")
_BUILD_LINE = re.compile(rb"Build: ([A-Z][a-z]{2} [ \d]\d \d{4}) (\d\d:\d\d:\d\d), gcc")
_TARGET_TEXT = re.compile(rb"\x00(guest(?:-[a-z0-9]+)*)\x00")


class ManifestError(ValueError):
    """A manifest that cannot be read or made."""


@dataclass(frozen=True)
class BuildInfo:
    build_time: Optional[str]      # "Nov 24 2026 22:11:53" (the __DATE__ __TIME__ text)
    git_rev: Optional[str]         # "9fa1d98", "9fa1d98-dirty", "unknown"; None = not in the image
    fw_version: Optional[str]      # the STATUS text, "OpenScope <date> <time>[ <rev>]"
    targets: tuple                 # NUL-delimited "guest..." strings in the image
    openscope_text: bool           # the image holds the text "OpenScope"


def read_build_info(data: bytes) -> BuildInfo:
    m = _FW_TEXT.search(data)
    b = _BUILD_LINE.search(data)
    build_time = git_rev = fw = None
    if m:
        build_time = f"{m.group(1).decode()} {m.group(2).decode()}"
        git_rev = m.group(3).decode() if m.group(3) else None
        fw = m.group(0).decode()
    elif b:
        build_time = f"{b.group(1).decode()} {b.group(2).decode()}"
    targets = tuple(sorted({t.group(1).decode() for t in _TARGET_TEXT.finditer(data)}))
    return BuildInfo(build_time, git_rev, fw, targets, b"OpenScope" in data)


@dataclass(frozen=True)
class Manifest:
    target: str                    # make goal, e.g. "guest-coldtrace"
    version: str                   # free text for people, e.g. "guest-coldtrace 9fa1d98"
    git_rev: Optional[str]         # from the image; None for a build with no revision text
    build_time: Optional[str]      # from the image, "Mmm dd yyyy hh:mm:ss"
    api_level: int                 # ESP_API_LEVEL of the source (remote_protocol.md §9.2)
    size: int
    sha256: str
    product_commit: Optional[str] = None   # full commit of the source tree, if known
    image: Optional[str] = None            # file name of the image next to the manifest
    format: int = FORMAT

    @property
    def fw_version(self) -> Optional[str]:
        """The STATUS fw_version text this build reports."""
        if not self.build_time:
            return None
        return f"OpenScope {self.build_time}" + (f" {self.git_rev}" if self.git_rev else "")

    def to_json(self) -> str:
        return json.dumps(asdict(self), indent=2) + "\n"

    @classmethod
    def from_json(cls, text: str) -> "Manifest":
        try:
            raw = json.loads(text)
        except ValueError as e:
            raise ManifestError(f"manifest is not JSON: {e}") from None
        if not isinstance(raw, dict):
            raise ManifestError("manifest is not a JSON object")
        if raw.get("format") != FORMAT:
            raise ManifestError(f"manifest format {raw.get('format')!r}, this tool reads {FORMAT}")
        names = {f.name for f in fields(cls)}
        missing = [n for n in ("target", "version", "api_level", "size", "sha256") if n not in raw]
        if missing:
            raise ManifestError("manifest has no " + ", ".join(missing))
        try:
            m = cls(**{k: v for k, v in raw.items() if k in names})
            if not isinstance(m.size, int) or not isinstance(m.api_level, int):
                raise TypeError("size and api_level must be integers")
            if not re.fullmatch(r"[0-9a-f]{64}", str(m.sha256)):
                raise TypeError("sha256 must be 64 lower-case hex digits")
        except TypeError as e:
            raise ManifestError(f"manifest is not valid: {e}") from None
        return m

    @classmethod
    def load(cls, path: Union[str, Path]) -> "Manifest":
        try:
            text = Path(path).read_text(encoding="utf-8")
        except OSError as e:
            raise ManifestError(f"cannot read manifest {path}: {e}") from None
        return cls.from_json(text)

    def save(self, path: Union[str, Path]) -> None:
        with open(path, "x", encoding="utf-8", newline="\n") as f:   # never overwrite
            f.write(self.to_json())


def manifest_path_for(image: Union[str, Path]) -> Path:
    return Path(str(image) + MANIFEST_SUFFIX)


def image_path_for(manifest_path: Union[str, Path], manifest: Manifest) -> Path:
    """The image a manifest file describes: its `image` field, next to the
    manifest; else the manifest path without the suffix."""
    p = Path(manifest_path)
    if manifest.image:
        if Path(manifest.image).name != manifest.image:
            raise ManifestError(f"manifest image name is not a plain file name: {manifest.image!r}")
        return p.parent / manifest.image
    if p.name.endswith(MANIFEST_SUFFIX):
        return p.with_name(p.name[: -len(MANIFEST_SUFFIX)])
    raise ManifestError(f"manifest {p} names no image")


def api_level_from_source(source: Union[str, Path]) -> int:
    """ESP_API_LEVEL from <source>/firmware/src/drivers/esp_comm.h."""
    h = Path(source) / "firmware" / "src" / "drivers" / "esp_comm.h"
    try:
        text = h.read_text(encoding="utf-8", errors="replace")
    except OSError as e:
        raise ManifestError(f"cannot read {h}: {e}") from None
    m = re.search(r"^\s*#define\s+ESP_API_LEVEL\s+(\d+)", text, re.M)
    if not m:
        raise ManifestError(f"{h} has no ESP_API_LEVEL")
    return int(m.group(1))


def make_manifest(data: bytes, target: str, api_level: Optional[int] = None,
                  version: Optional[str] = None, source: Optional[Union[str, Path]] = None,
                  product_commit: Optional[str] = None,
                  image_name: Optional[str] = None) -> Manifest:
    if not TARGET_RE.match(target or ""):
        raise ManifestError(f"target {target!r} is not a 2C53T guest target (guest-...)")
    if api_level is None:
        if source is None:
            raise ManifestError("give the API level, or the source tree to read ESP_API_LEVEL from")
        api_level = api_level_from_source(source)
    if api_level < 0:
        raise ManifestError("API level must be 0 or more")
    info = read_build_info(data)
    if not info.build_time:
        raise ManifestError("the image holds no OpenScope build time text: not an OpenScope image")
    if info.targets and target not in info.targets:
        raise ManifestError(f"the image says it is {', '.join(info.targets)}, not {target}")
    rev = info.git_rev
    return Manifest(target=target,
                    version=version or f"{target} {rev or info.build_time}",
                    git_rev=rev, build_time=info.build_time, api_level=api_level,
                    size=len(data), sha256=hashlib.sha256(data).hexdigest(),
                    product_commit=product_commit, image=image_name)


def check_image(data: bytes, manifest: Optional[Manifest], allow_unlisted: bool = False) -> List[str]:
    """Every reason not to copy this image to the IAP drive. [] = may copy."""
    out: List[str] = []
    sha = hashlib.sha256(data).hexdigest()
    if sha in KNOWN_STOCK:
        out.append(f"this is {KNOWN_STOCK[sha]}, not an OpenScope image "
                   "(return to stock is a separate path)")
    if manifest is None:
        if not allow_unlisted:
            out.append("no manifest for this image (make one with `openscope manifest`, "
                       "or give --allow-unlisted)")
    else:
        if manifest.sha256 != sha:
            out.append(f"SHA-256 {sha[:12]}... is not the manifest's {manifest.sha256[:12]}...: "
                       "wrong or damaged file")
        if manifest.size != len(data):
            out.append(f"size {len(data)} B is not the manifest's {manifest.size} B")
        if not TARGET_RE.match(manifest.target or ""):
            out.append(f"manifest target {manifest.target!r} is not a 2C53T guest target")
        elif any(t in manifest.target.split("-") for t in TEST_ONLY_TARGETS):
            out.append(f"manifest target {manifest.target} is a test image, not for a user unit")
    if len(data) > MAX_SIZE:
        out.append(f"size {len(data):,} B is more than {MAX_SIZE:,} B "
                   f"(0x{GUEST_BASE:08X}..0x{FLASH_CEILING:08X})")
    if len(data) < 8:
        out.append("file too short for a vector table")
        return out
    sp, rv = struct.unpack_from("<II", data, 0)
    if not SP_RANGE[0] <= sp <= SP_RANGE[1]:
        out.append(f"initial SP 0x{sp:08X} is not in SRAM 0x{SP_RANGE[0]:08X}..0x{SP_RANGE[1]:08X}")
    entry = rv & ~1
    if not rv & 1:
        out.append(f"reset vector 0x{rv:08X} has no Thumb bit")
    if entry < GUEST_BASE:
        out.append(f"reset vector 0x{rv:08X} is below 0x{GUEST_BASE:08X}: not a guest image (I-10)")
    elif entry >= GUEST_BASE + len(data):
        out.append(f"reset vector 0x{rv:08X} is past the end of the image "
                   f"(0x{GUEST_BASE + len(data):08X})")
    info = read_build_info(data)
    if not info.openscope_text:
        out.append("the image holds no 'OpenScope' text: not an OpenScope application")
    elif not info.build_time:
        out.append("the image holds no OpenScope build time text")
    if manifest is not None and info.build_time and manifest.build_time \
            and info.build_time != manifest.build_time:
        out.append(f"image build time {info.build_time} is not the manifest's {manifest.build_time}")
    if manifest is not None and info.targets and manifest.target not in info.targets:
        out.append(f"the image says it is {', '.join(info.targets)}, the manifest says {manifest.target}")
    return out


def check_running(manifest: Manifest, fw_version: Optional[str], identity) -> List[str]:
    """Differences between the manifest and what the unit reports (STATUS
    fw_version and DESCRIPTION section 7, None for firmware before
    stage-190). [] = the unit runs this build."""
    out: List[str] = []
    want = manifest.fw_version
    if want is None:
        out.append("the manifest has no build time: cannot compare")
    elif fw_version != want:
        out.append(f"firmware text {fw_version!r} is not {want!r}")
    if identity is None:
        if manifest.api_level > 0:
            out.append(f"the unit reports no identity (API level 0); the manifest says "
                       f"API level {manifest.api_level}")
        return out
    if identity.target != manifest.target:
        out.append(f"build target {identity.target!r} is not {manifest.target!r}")
    if manifest.git_rev and identity.git_rev != manifest.git_rev:
        out.append(f"git revision {identity.git_rev!r} is not {manifest.git_rev!r}")
    if identity.api_level != manifest.api_level:
        out.append(f"API level {identity.api_level} is not {manifest.api_level}")
    return out
