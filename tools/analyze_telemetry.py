"""Summarize the newest kOS flight and emit conservative tuning hints."""

from __future__ import annotations

import csv
import sys
from pathlib import Path


FIELDS = [
    "mission", "phase", "ut", "altitude", "hook_height", "v_vertical",
    "v_horizontal", "h_error", "throttle", "tilt", "mass", "max_thrust",
]


def read_rows(path: Path) -> list[dict[str, float | str]]:
    rows: list[dict[str, float | str]] = []
    with path.open(newline="", encoding="utf-8-sig") as stream:
        for raw in csv.reader(stream):
            if not raw:
                continue
            # main.ks writes a fresh header at every launch while kOS appends to
            # the same archive. Keep only the newest header-delimited flight;
            # sandbox saves often reuse the same universal-time mission number.
            if raw[0] == "mission":
                rows.clear()
                continue
            if len(raw) < 10:
                continue
            try:
                row: dict[str, float | str] = {"mission": raw[0], "phase": raw[1]}
                for index, field in enumerate(FIELDS[2:], start=2):
                    row[field] = float(raw[index]) if index < len(raw) and raw[index] else 0.0
                rows.append(row)
            except ValueError:
                continue
    return rows


def main() -> int:
    default = Path(r"C:\Projects\Kerbal Space Program\Ships\Script\cz10b\telemetry.csv")
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else default
    if not path.exists():
        print(f"telemetry not found: {path}")
        return 2
    rows = read_rows(path)
    if not rows:
        print("no numeric telemetry rows found")
        return 2
    mission = rows[-1]["mission"]
    mission_rows = [row for row in rows if row["mission"] == mission]
    phase_counts = {
        name: sum(row["phase"] == name for row in mission_rows)
        for name in (
            "ASCENT", "COAST", "BOOSTBACK", "ENTRY", "TRAJECTORY",
            "H_STOPPING", "PID_TERMINAL", "TERMINAL",
        )
    }
    phase = next(
        (name for name in ("PID_TERMINAL", "TERMINAL", "HOVER_TEST", "RETURN")
         if any(row["phase"] == name for row in mission_rows)),
        "",
    )
    terminal_names = {"TRAJECTORY", "H_STOPPING", "PID_TERMINAL"} \
        if phase == "PID_TERMINAL" else {phase}
    flight = [row for row in mission_rows if row["phase"] in terminal_names]
    if not flight:
        present = sorted({str(row["phase"]) for row in mission_rows})
        print(f"mission={mission}: no terminal samples; present={present}")
        return 1

    # The controller may intentionally hover or climb after an early centerline
    # miss.  The final numeric sample immediately precedes CAPTURE/TIMEOUT and is
    # therefore the meaningful terminal state; minimum absolute height is not.
    entry = flight[-1]
    low = [row for row in flight if float(row["hook_height"]) < 100]
    saturated = sum(float(row["throttle"]) > 0.98 for row in flight) / len(flight)
    min_error = min(float(row["h_error"]) for row in low or flight)
    max_tilt_low = max(float(row["tilt"]) for row in low or flight)
    hover_time = 0.0
    entered_center = False
    rebound_after_center = 0.0
    for previous, current in zip(flight, flight[1:]):
        if (12.0 < float(previous["hook_height"]) < 150.0
                and abs(float(previous["v_vertical"])) < 0.3):
            hover_time += min(
                max(float(current["ut"]) - float(previous["ut"]), 0.0), 0.5
            )
        error = float(current["h_error"])
        if error <= 5.0:
            entered_center = True
        elif entered_center:
            rebound_after_center = max(rebound_after_center, error)

    print(f"mission={mission} phase={phase} samples={len(flight)}")
    print("phase_samples=" + ",".join(
        f"{name}:{count}" for name, count in phase_counts.items() if count
    ))
    print(f"entry hook_height={float(entry['hook_height']):.2f} m "
          f"vertical={float(entry['v_vertical']):.2f} m/s "
          f"lateral={float(entry['v_horizontal']):.2f} m/s "
          f"error={float(entry['h_error']):.2f} m")
    print(f"minimum_error_below_100m={min_error:.2f} m")
    print(f"throttle_saturation_fraction={saturated:.1%}")
    print(f"max_tilt_below_100m={max_tilt_low:.1f} deg")
    print(f"hover_time_between_12m_and_150m={hover_time:.2f} s")
    print(f"rebound_after_entering_5m={rebound_after_center:.2f} m")

    hints: list[str] = []
    if phase in {"RETURN", "TERMINAL", "PID_TERMINAL"} and float(entry["v_vertical"]) < -5:
        hints.append("terminal descent still fast: start the single ENTRY burn earlier or extend its maximum time")
    if saturated > 0.25:
        hints.append("insufficient control authority: reduce landing mass or use a stronger engine")
    if float(entry["h_error"]) > 8 and float(entry["v_horizontal"]) < 2:
        hints.append("slow lateral convergence: raise H_POS_KP_LOW by 10%")
    if rebound_after_center > 8.0:
        hints.append("lateral overshoot: enter HORIZONTAL_CORRIDOR_HEIGHT earlier or lower TERMINAL_HORIZONTAL_STOP_ACCEL")
    elif float(entry["v_horizontal"]) > 3.5:
        hints.append("final lateral speed high: increase H_VEL_KP only after checking the stopping corridor")
    if max_tilt_low >= 11.8:
        hints.append("tilt-limited near net: correct the trajectory earlier; do not raise LANDING_MAX_TILT first")
    if not hints:
        hints.append("terminal metrics are inside the nominal capture envelope; inspect KSP.log for CAPTURE/REJECT")
    print("hints:")
    for hint in hints:
        print(f"- {hint}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
