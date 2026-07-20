"""Catch misspelled all-caps kOS configuration symbols before flight.

kOS resolves archive-loaded variables at runtime, so a typo in a rarely taken
branch is not reported when the program is first compiled.  This lightweight
check combines the CZ10B scripts, removes comments/strings and reports any
all-caps variable that has no declaration.  Suffix names (``SHIP:ALTITUDE``)
are deliberately ignored.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT_DIR = ROOT / "Ships" / "Script" / "cz10b"
SCRIPTS = tuple(SCRIPT_DIR / name for name in (
    "config.ks", "controller.ks", "main.ks", "upper.ks"
))

# Language keywords, commands, built-ins and bound flight-control variables.
BUILTINS = {
    "ABS", "AG10", "AND", "AT", "BRAKES", "BREAK", "CHOOSE",
    "CLEARSCREEN", "CONSTANT", "COS", "DECLARE", "DO", "ELSE", "FALSE",
    "FOR", "FROM", "FUNCTION", "HASTARGET", "HEADING", "IF", "IN", "IS",
    "LAZYGLOBAL", "LIST", "LN", "LOCAL", "LOCK", "LOG", "LOOKDIRUP",
    "MAX", "MIN", "MOD", "NOT", "OFF", "ON", "OR", "PARAMETER", "PRINT",
    "PROGRADE", "RCS", "RETURN", "ROUND", "RUNPATH", "SAS", "SET",
    "SHIP", "SHUTDOWN", "SIN", "SQRT", "STAGE", "STEERING", "STEP", "TAN",
    "THROTTLE", "TIME", "TO", "TRUE", "UNLOCK", "UNTIL", "UP", "V",
    "VANG", "VDOT", "VCRS", "VESSELS", "VXCL", "WAIT", "WHEN", "WRITE",
}

STRING_RE = re.compile(r'"(?:[^"\\]|\\.)*"')
REFERENCE_RE = re.compile(r"(?<!:)(?<![A-Za-z0-9_])([A-Z][A-Z0-9_]*)(?![A-Za-z0-9_])")
DEFINITION_RES = (
    re.compile(r"\b(?:SET|LOCAL|DECLARE)\s+([A-Z][A-Z0-9_]*)\b"),
    re.compile(r"\bFUNCTION\s+([A-Z][A-Z0-9_]*)\b"),
    re.compile(r"\bFOR\s+([A-Z][A-Z0-9_]*)\s+IN\b"),
    re.compile(r"\bLIST\b[^.{}]*?\bIN\s+([A-Z][A-Z0-9_]*)\b"),
)


def executable_text(path: Path) -> str:
    lines: list[str] = []
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        without_strings = STRING_RE.sub("", raw_line)
        lines.append(without_strings.split("//", 1)[0])
    return "\n".join(lines)


def main() -> int:
    texts = {path: executable_text(path) for path in SCRIPTS}
    combined = "\n".join(texts.values())
    defined = set(BUILTINS)
    for pattern in DEFINITION_RES:
        defined.update(pattern.findall(combined))
    for parameter_list in re.findall(r"\bPARAMETER\s+([^.]*)\.", combined, re.S):
        defined.update(re.findall(r"\b[A-Z][A-Z0-9_]*\b", parameter_list))

    missing: dict[str, list[str]] = {}
    for path, text in texts.items():
        for line_number, line in enumerate(text.splitlines(), 1):
            for name in REFERENCE_RE.findall(line):
                if name not in defined:
                    missing.setdefault(name, []).append(
                        f"{path.relative_to(ROOT)}:{line_number}"
                    )

    if missing:
        for name, locations in sorted(missing.items()):
            print(f"undefined kOS symbol {name}: {', '.join(locations[:5])}")
        return 1

    print(f"kOS symbol check passed: {len(defined)} declared/builtin names")
    return 0


if __name__ == "__main__":
    sys.exit(main())
