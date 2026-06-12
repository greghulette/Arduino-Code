#!/usr/bin/env python3
"""
sync_html.py — regenerate docs/index.html from SBUSController_html.h.

SBUSController_html.h is the canonical source: the firmware embeds its
contents (between the R"rawhtml( ... )rawhtml" delimiters) into flash and
serves them from the ESP32's web server.  GitHub Pages can't read C headers,
so we mirror the same HTML body into docs/index.html for the static / USB
Serial workflow.

Run this whenever you edit SBUSController_html.h:

    python sync_html.py

It just extracts the raw-string body and writes it (verbatim) to
docs/index.html.  No transformation, no minification, no smarts.  Both files
should always be checked in to git.
"""
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
SRC  = HERE / "SBUSController_html.h"
DST  = HERE / "docs" / "index.html"

START_MARKER = 'R"rawhtml('
END_MARKER   = ')rawhtml";'


def main() -> int:
    if not SRC.exists():
        print(f"error: source not found: {SRC}", file=sys.stderr)
        return 1

    text = SRC.read_text(encoding="utf-8")
    start = text.find(START_MARKER)
    if start < 0:
        print(f"error: opening marker {START_MARKER!r} not found in {SRC.name}",
              file=sys.stderr)
        return 1
    body_start = start + len(START_MARKER)
    body_end   = text.find(END_MARKER, body_start)
    if body_end < 0:
        print(f"error: closing marker {END_MARKER!r} not found after opening marker",
              file=sys.stderr)
        return 1

    body = text[body_start:body_end]
    # Strip a leading newline if present so the file starts with <!DOCTYPE html>.
    if body.startswith("\n"):
        body = body[1:]

    DST.parent.mkdir(parents=True, exist_ok=True)
    DST.write_text(body, encoding="utf-8", newline="\n")

    lines = body.count("\n") + (0 if body.endswith("\n") else 1)
    print(f"wrote {DST}  ({len(body):,} bytes, {lines:,} lines)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
