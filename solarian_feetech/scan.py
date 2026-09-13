"""Scan a Feetech serial bus for responding motors."""

import serial.tools.list_ports

from .lerobot.feetech import FeetechMotorsBus
from .lerobot.feetech.tables import MODEL_NUMBER_TABLE

MODEL_NB_TO_NAME = {nb: name for name, nb in MODEL_NUMBER_TABLE.items()}

#: Highest motor ID probed during the unicast sweep. Kept small because each
#: missing ID costs a full packet timeout (~50 ms); robot chains use low IDs.
SWEEP_MAX_ID = 30


def prompt_for_port() -> str | None:
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No serial ports detected.")
        return None

    print("\n=== Available Ports ===")
    for idx, p in enumerate(ports, 1):
        print(f"{idx}. {p.device} ({p.description})")

    raw = input(f"Select a port (1-{len(ports)}), or type a path: ").strip()
    if raw.isdigit():
        idx = int(raw)
        if 1 <= idx <= len(ports):
            return ports[idx - 1].device
        print("Invalid selection.")
        return None
    return raw or None


def baudrates_to_scan(bus: FeetechMotorsBus, only_baudrate: int | None) -> list[int]:
    if only_baudrate:
        return [only_baudrate]
    rest = [b for b in bus.available_baudrates if b != bus.default_baudrate]
    return [bus.default_baudrate, *rest]


def unicast_sweep(bus: FeetechMotorsBus, skip: set[int], *, max_id: int = SWEEP_MAX_ID) -> dict[int, int]:
    """Ping IDs individually (low range only) and return ``{id: model_number}``."""
    motors: dict[int, int] = {}
    for motor_id in range(max_id + 1):
        if motor_id in skip:
            continue
        model_nb = bus.ping(motor_id, num_retry=2)
        if model_nb is not None:
            motors[motor_id] = model_nb
    return motors


def scan_motors(
    bus: FeetechMotorsBus,
    only_baudrate: int | None = None,
    *,
    verbose: bool = True,
) -> dict[int, dict[int, int]]:
    """Return ``{baudrate: {id: model_number}}`` for responding motors.

    A broadcast ping is fast but can silently miss motors whose
    ``Return_Delay_Time`` is large (the factory default), so we confirm with
    a short per-ID unicast sweep on the active baud-rate. Motors always share
    a single baud-rate, so we stop at the first rate that responds.
    """
    baudrates = baudrates_to_scan(bus, only_baudrate)
    if verbose:
        print("\nScanning bus...")
    found: dict[int, dict[int, int]] = {}
    for baudrate in baudrates:
        bus.set_baudrate(baudrate)
        motors = dict(bus.broadcast_ping(num_retry=1) or {})
        if motors:
            motors.update(unicast_sweep(bus, skip=set(motors)))
            found[baudrate] = motors
            break

    if not found:
        if verbose:
            print("Broadcast ping found nothing; falling back to per-ID scan...")
        for baudrate in baudrates:
            bus.set_baudrate(baudrate)
            motors = unicast_sweep(bus, skip=set())
            if motors:
                found[baudrate] = motors
                break

    return found
