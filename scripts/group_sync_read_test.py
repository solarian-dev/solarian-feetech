#!/usr/bin/env python
"""Read a register from every detected Feetech motor using group sync read.

This is a diagnostic for the "Incorrect status packet" failures seen when
mixing motor models on one bus. It:

  1. Detects every motor on the port (broadcast ping + per-ID unicast sweep,
     across all baud-rates).
  2. Performs a grouped sync read of all motors that share a baud-rate.
  3. Repeats the grouped read per model (to reveal mixed-model incompatibility).
  4. Performs the CHUNKED read actually used by FeetechBus (per model, capped at
     --max-group motors), which is the reliable+fast strategy.
  5. Reads each motor individually as a control (this path is known-reliable).

Usage:
    python group_sync_read_test.py [--port PORT] [--baudrate BAUD]
                                   [--data REGISTER] [--loops N]
                                   [--max-group N]

If --port is omitted you'll be prompted to choose one. --data defaults to
"Present_Position".
"""

import argparse
import sys

import serial.tools.list_ports

from solarian.core.calibration import Motor, MotorNormMode
from solarian_feetech.bus import SYNC_READ_MAX_GROUP, chunk_by_model, grouped_sync_read
from solarian_feetech.driver.feetech import FeetechMotorsBus
from solarian_feetech.driver.feetech.tables import MODEL_NUMBER_TABLE

#: model number reported by a motor -> model name understood by the SDK
_MODEL_NB_TO_NAME = {nb: name for name, nb in MODEL_NUMBER_TABLE.items()}

#: Highest motor ID probed during the unicast sweep. Kept small because each
#: missing ID costs a full packet timeout (~50 ms); robot chains use low IDs.
_SWEEP_MAX_ID = 30


def pick_port(port: str | None) -> str | None:
    if port:
        return port
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No serial ports detected.")
        return None
    print("\n=== Available Ports ===")
    for idx, p in enumerate(ports, 1):
        print(f"{idx}. {p.device} ({p.description})")
    raw = input(f"Select a port (1-{len(ports)}), or type a path: ").strip()
    if raw.isdigit() and 1 <= int(raw) <= len(ports):
        return ports[int(raw) - 1].device
    return raw or None


def detect(port: str, only_baudrate: int | None) -> dict[int, dict[int, int]]:
    """Return {baudrate: {id: model_number}} for every responding motor.

    Tries the default (most common) baud-rate first and stops at the first rate
    that responds, since a chain always shares one baud-rate. A short unicast
    sweep (bounded to low IDs) catches any motor the broadcast ping missed.
    """
    bus = FeetechMotorsBus(port, {})
    bus._connect(handshake=False)
    found: dict[int, dict[int, int]] = {}
    try:
        if only_baudrate:
            baudrates = [only_baudrate]
        else:
            rest = [b for b in bus.available_baudrates if b != bus.default_baudrate]
            baudrates = [bus.default_baudrate, *rest]

        for baudrate in baudrates:
            bus.set_baudrate(baudrate)
            motors = dict(bus.broadcast_ping(num_retry=1) or {})
            if motors:
                # Catch motors the broadcast missed, then stop: the whole chain
                # lives on this one baud-rate.
                for motor_id in range(_SWEEP_MAX_ID + 1):
                    if motor_id in motors:
                        continue
                    model_nb = bus.ping(motor_id)
                    if model_nb is not None:
                        motors[motor_id] = model_nb
                found[baudrate] = motors
                break
    finally:
        bus.port_handler.closePort()
    return found


def _name(motor_id: int) -> str:
    return f"m{motor_id}"


def _group_read(bus: FeetechMotorsBus, names: list[str], data: str) -> None:
    """Attempt one grouped sync read and print the outcome."""
    try:
        values = bus.sync_read(data, names, normalize=False, num_retry=0)
        print(f"    OK  -> {values}")
    except Exception as e:  # noqa: BLE001 - diagnostic: report any failure
        print(f"    FAIL -> {e}")


def run_for_baudrate(
    port: str, baudrate: int, id_to_model_nb: dict[int, int], data: str, loops: int, max_group: int
) -> None:
    print(f"\n=== Baud-rate {baudrate} ===")

    motors: dict[str, Motor] = {}
    names_by_model: dict[str, list[str]] = {}
    for motor_id, model_nb in sorted(id_to_model_nb.items()):
        model = _MODEL_NB_TO_NAME.get(model_nb)
        if model is None:
            print(f"  Skipping id {motor_id}: unknown model number {model_nb}")
            continue
        name = _name(motor_id)
        motors[name] = Motor(motor_id, model, MotorNormMode.RANGE_M100_100)
        names_by_model.setdefault(model, []).append(name)

    if not motors:
        print("  No known motors on this baud-rate.")
        return

    print("  Motors: " + ", ".join(f"{_name(m.id)}={m.model}" for m in motors.values()))

    bus = FeetechMotorsBus(port, motors)
    bus._connect(handshake=False)
    bus.set_baudrate(baudrate)
    try:
        all_names = list(motors)

        for i in range(loops):
            label = f" (loop {i + 1}/{loops})" if loops > 1 else ""
            print(f"\n  [group read: ALL motors]{label}")
            _group_read(bus, all_names, data)

        if len(names_by_model) > 1:
            print("\n  [group read: per model]")
            for model, names in names_by_model.items():
                print(f"    model {model} ({', '.join(names)}):")
                _group_read(bus, names, data)

        groups = chunk_by_model(bus, all_names, max_group)
        print(f"\n  [chunked read: per model, capped at {max_group} — FeetechBus strategy]")
        print("    chunks: " + " | ".join(", ".join(g) for g in groups))
        values = grouped_sync_read(bus, data, all_names, normalize=False, max_group=max_group)
        print(f"    -> {values}")

        print("\n  [individual reads: control]")
        for name in all_names:
            try:
                value = bus.read(data, name, normalize=False, num_retry=1)
                print(f"    {name}: {value}")
            except Exception as e:  # noqa: BLE001
                print(f"    {name}: FAIL -> {e}")
    finally:
        bus.port_handler.closePort()


def main() -> None:
    parser = argparse.ArgumentParser(description="Group sync read test for Feetech motors.")
    parser.add_argument("--port", default=None, help="Serial port (prompted if omitted).")
    parser.add_argument("--baudrate", type=int, default=None, help="Only use this baud-rate.")
    parser.add_argument("--data", default="Present_Position", help="Register to read.")
    parser.add_argument("--loops", type=int, default=1, help="Repeat the ALL-motors group read N times.")
    parser.add_argument(
        "--max-group",
        type=int,
        default=SYNC_READ_MAX_GROUP,
        help=f"Max motors per chunk for the chunked read (FeetechBus default: {SYNC_READ_MAX_GROUP}).",
    )
    args = parser.parse_args()

    port = pick_port(args.port)
    if port is None:
        print("No port selected. Aborting.")
        sys.exit(1)

    print(f"\nDetecting motors on {port}...")
    found = detect(port, args.baudrate)
    if not found:
        print("No motors detected.")
        sys.exit(1)

    for baudrate in sorted(found):
        ids = sorted(found[baudrate])
        print(f"  baud {baudrate}: ids {ids}")

    for baudrate in sorted(found):
        run_for_baudrate(port, baudrate, found[baudrate], args.data, args.loops, args.max_group)


if __name__ == "__main__":
    main()
