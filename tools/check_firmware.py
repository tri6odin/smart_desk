#!/usr/bin/env python3
"""Run host checks and optionally all firmware builds; never flash hardware."""

import argparse
from pathlib import Path
import shlex
import subprocess

ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / "firmware"
MODES = (
    ("ble-controller", "BLE_CONTROLLER"),
    ("read-only", "READ_ONLY"),
    ("idle", "IDLE_EMULATOR"),
    ("step-down", "STEP_DOWN_TEST"),
    ("hold-down", "HOLD_DOWN_TEST"),
    ("ble-logger", "BLE_HID_LOGGER"),
    ("ble-one-shot", "BLE_ONE_SHOT_TEST"),
    ("ble-calibration", "BLE_CALIBRATION"),
)


def run(args, cwd=ROOT):
    print(shlex.join(map(str, args)), flush=True)
    subprocess.run(args, cwd=cwd, check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--all-modes", action="store_true", help="build all 8 modes using EIM")
    parser.add_argument("--fresh-configs", action="store_true",
                        help="use isolated generated configs/builds for --all-modes")
    parser.add_argument("--sanitizers", metavar="LIST", help="e.g. address,undefined or undefined")
    args = parser.parse_args()
    build = FIRMWARE / ("build-host-sanitized" if args.sanitizers else "build-host")
    configure = ["cmake", "-S", str(FIRMWARE / "tests"), "-B", str(build),
                 "-D", "CMAKE_BUILD_TYPE=Debug",
                 "-D", "DESK_ENABLE_SANITIZERS=" + ("ON" if args.sanitizers else "OFF")]
    if args.sanitizers:
        configure += ["-D", "DESK_SANITIZERS=" + args.sanitizers]
    run(configure)
    run(["cmake", "--build", str(build)])
    run(["ctest", "--test-dir", str(build), "--output-on-failure", "--timeout", "30"])

    if args.all_modes:
        for mode, symbol in MODES:
            suffix = "" if mode == "read-only" else "." + mode
            config = "sdkconfig" + suffix
            output = "build" if not suffix else "build-" + mode
            defaults = "sdkconfig.defaults" + (";" + config + ".defaults" if suffix else "")
            if args.fresh_configs:
                config = "sdkconfig.check-" + mode
                output = "build-check-" + mode
                # This filename belongs to the check runner, never to a user build.
                (FIRMWARE / config).unlink(missing_ok=True)
            command = shlex.join(["idf.py", "-B", output, "-D", "IDF_TARGET=esp32s3",
                                 "-D", "SDKCONFIG=" + config,
                                 "-D", "SDKCONFIG_DEFAULTS=" + defaults, "build"])
            run(["eim", "run", command, "v6.1"], cwd=FIRMWARE)
            expected = "CONFIG_DESK_MODE_" + symbol + "=y"
            if expected not in (FIRMWARE / config).read_text().splitlines():
                raise RuntimeError(f"{mode}: unexpected effective firmware configuration")
            if 'CONFIG_IDF_TARGET="esp32s3"' not in (FIRMWARE / config).read_text().splitlines():
                raise RuntimeError(f"{mode}: unexpected ESP target")
            print(f"{mode}: PASS", flush=True)


if __name__ == "__main__":
    main()
