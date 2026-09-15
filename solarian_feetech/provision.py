"""Provisioning primitives for Feetech servos, addressed by raw motor ID.

Setting an ID, resetting comms registers and centering the shaft all have to work
on a servo that is not (yet) a declared component: a fresh servo answers at the
factory ID 1 and its final ID is what we are about to choose. So everything here
takes a :class:`FeetechMotorsBus` built with no motors and an explicit
``(motor_id, model)`` pair, instead of a component name.

:class:`~solarian_feetech.bus.FeetechBus` and ``solarian feetech-set-id`` both
route through these helpers so the register sequence cannot drift between them.
"""

from __future__ import annotations

import time

from .lerobot.common.utils.encoding_utils import decode_sign_magnitude, encode_sign_magnitude
from .lerobot.feetech import FeetechMotorsBus, OperatingMode, TorqueMode
from .lerobot.feetech.tables import (
    MODEL_BAUDRATE_TABLE,
    MODEL_ENCODING_TABLE,
    MODEL_NUMBER_TABLE,
    MODEL_RESOLUTION,
)
from .lerobot.motors_bus import get_address
from .scan import MODEL_NB_TO_NAME, scan_motors


def write_reg(bus: FeetechMotorsBus, model: str, motor_id: int, data_name: str, value: int) -> None:
    addr, length = get_address(bus.model_ctrl_table, model, data_name)
    sign_bit = MODEL_ENCODING_TABLE.get(model, {}).get(data_name)
    encoded = value if sign_bit is None else encode_sign_magnitude(value, sign_bit)
    bus._write(addr, length, motor_id, encoded, raise_on_error=True)


def read_reg(bus: FeetechMotorsBus, model: str, motor_id: int, data_name: str) -> int:
    addr, length = get_address(bus.model_ctrl_table, model, data_name)
    raw, _, _ = bus._read(addr, length, motor_id, raise_on_error=True)
    sign_bit = MODEL_ENCODING_TABLE.get(model, {}).get(data_name)
    if sign_bit is None:
        return raw
    return decode_sign_magnitude(raw, sign_bit)


def ensure_single_turn_feedback(bus: FeetechMotorsBus, model: str, motor_id: int) -> bool:
    """Clear Phase bit 4 on STS3215 so feedback stays in ``[0, resolution)``."""
    if model != "sts3215" or "Phase" not in bus.model_ctrl_table.get(model, {}):
        return False
    phase = read_reg(bus, model, motor_id, "Phase")
    if not phase & 0x10:
        return False
    write_reg(bus, model, motor_id, "Phase", phase & ~0x10)
    return True


def open_position_registers(bus: FeetechMotorsBus, model: str, motor_id: int) -> None:
    """Clear Homing_Offset and open the position limits, so Goal is absolute.

    EEPROM writes need Lock=0, which ``_disable_torque`` clears. Homing_Offset is
    read back because a silent leftover shift sends every later goal to the wrong
    place.
    """
    resolution = MODEL_RESOLUTION[model]
    bus._disable_torque(motor_id, model)
    if MODEL_ENCODING_TABLE.get(model, {}).get("Homing_Offset") is not None:
        write_reg(bus, model, motor_id, "Homing_Offset", 0)
        got = read_reg(bus, model, motor_id, "Homing_Offset")
        if got != 0:
            raise RuntimeError(
                f"id={motor_id}: Homing_Offset read back {got} after writing 0. "
                "Unlock failed or the register was not written."
            )
    write_reg(bus, model, motor_id, "Min_Position_Limit", 0)
    write_reg(bus, model, motor_id, "Max_Position_Limit", resolution - 1)


def move_to_mid_encoder(
    bus: FeetechMotorsBus,
    model: str,
    motor_id: int,
    *,
    settle_s: float = 3.0,
    tol: int = 8,
) -> int:
    """Drive the shaft to mid-encoder, leave torque off, return the reading reached.

    Only for a free shaft: the goal is absolute mid-encoder, so it can be most of
    a turn away from where the shaft sits and a mounted linkage would be driven
    into a hard stop.
    """
    half = (MODEL_RESOLUTION[model] - 1) // 2
    open_position_registers(bus, model, motor_id)
    if "Operating_Mode" in bus.model_ctrl_table.get(model, {}):
        write_reg(bus, model, motor_id, "Operating_Mode", OperatingMode.POSITION.value)
    ensure_single_turn_feedback(bus, model, motor_id)

    addr, length = get_address(bus.model_ctrl_table, model, "Torque_Enable")
    bus._write(addr, length, motor_id, TorqueMode.ENABLED.value, raise_on_error=True)
    addr, length = get_address(bus.model_ctrl_table, model, "Lock")
    bus._write(addr, length, motor_id, 1, raise_on_error=True)
    try:
        write_reg(bus, model, motor_id, "Goal_Position", half)
        deadline = time.monotonic() + settle_s
        while time.monotonic() < deadline:
            if abs(read_reg(bus, model, motor_id, "Present_Position") - half) <= tol:
                break
            time.sleep(0.05)
    finally:
        bus._disable_torque(motor_id, model)

    return read_reg(bus, model, motor_id, "Present_Position")


def apply_comms_config(bus: FeetechMotorsBus, model: str, motor_id: int) -> None:
    """Return_Delay_Time=0: the ~500 us factory default makes grouped sync reads collide."""
    bus._disable_torque(motor_id, model)
    write_reg(bus, model, motor_id, "Return_Delay_Time", 0)


def write_id_and_baudrate(
    bus: FeetechMotorsBus,
    model: str,
    current_baudrate: int,
    current_id: int,
    target_id: int,
    target_baudrate: int | None = None,
) -> None:
    target_baudrate = target_baudrate or bus.default_baudrate

    bus.set_baudrate(current_baudrate)
    bus._disable_torque(current_id, model)

    id_addr, id_len = get_address(bus.model_ctrl_table, model, "ID")
    bus._write(id_addr, id_len, current_id, target_id, raise_on_error=True)
    print(f"  Wrote ID {current_id} -> {target_id}")

    # Addressed at the *new* ID from here on.
    baud_addr, baud_len = get_address(bus.model_ctrl_table, model, "Baud_Rate")
    baud_index = MODEL_BAUDRATE_TABLE[model][target_baudrate]
    bus._write(baud_addr, baud_len, target_id, baud_index, raise_on_error=True)
    print(f"  Set baud-rate -> {target_baudrate}")

    bus.set_baudrate(target_baudrate)


def verify_id(bus: FeetechMotorsBus, model: str, target_id: int) -> bool:
    model_nb = bus.ping(target_id)
    if model_nb == MODEL_NUMBER_TABLE.get(model):
        print(f"  Verified: motor now responds at ID {target_id}.")
        return True
    print(f"  Warning: could not verify motor at ID {target_id} (ping returned {model_nb!r}).")
    return False


# ---------------------------------------------------------------- interactive


def ask_yes_no(prompt: str, default: bool = True) -> bool:
    suffix = "[Y/n]" if default else "[y/N]"
    raw = input(f"{prompt} {suffix} ").strip().lower()
    if not raw:
        return default
    return raw in ("y", "yes")


def prompt_for_id(current_id: int) -> int | None:
    raw = input("New ID (1-253), or Enter to cancel: ").strip()
    if not raw:
        return None
    if not raw.isdigit():
        print("ID must be a number.")
        return None
    new_id = int(raw)
    if not (1 <= new_id <= 253):
        print("ID must be between 1 and 253.")
        return None
    if new_id == current_id:
        print(f"Motor already has ID {new_id}; nothing to do.")
        return None
    return new_id


def open_bus(port: str) -> FeetechMotorsBus:
    """A bus with no declared motors: enough for scanning and by-ID register access."""
    bus = FeetechMotorsBus(port, {})
    bus._connect(handshake=False)
    return bus


def scan(port: str, only_baudrate: int | None = None, *, verbose: bool = True) -> list[dict]:
    """Servos answering on *port* as ``[{"id", "model", "baudrate"}]``."""
    bus = open_bus(port)
    try:
        found = scan_motors(bus, only_baudrate=only_baudrate, verbose=verbose)
    finally:
        close_bus(bus)

    return [
        {
            "id": motor_id,
            "model": MODEL_NB_TO_NAME.get(model_nb, f"unknown({model_nb})"),
            "baudrate": baudrate,
        }
        for baudrate in sorted(found)
        for motor_id, model_nb in sorted(found[baudrate].items())
    ]


def close_bus(bus: FeetechMotorsBus) -> None:
    try:
        bus.port_handler.closePort()
    except Exception:  # noqa: BLE001 - best-effort cleanup
        pass


def provision(bus: FeetechMotorsBus, baudrate: int, motor_id: int, model: str) -> None:
    """Interactively make one servo ready to use: comms, ID, registers, centering.

    Shared by ``solarian calibrate-servo`` and ``solarian feetech-set-id``, both of
    which have already chosen the servo to work on.
    """
    bus.set_baudrate(baudrate)

    # Unconditional: also the point of re-running this on an already-numbered servo.
    apply_comms_config(bus, model, motor_id)
    print("  Set Return_Delay_Time -> 0")

    active_id = motor_id
    if ask_yes_no(
        f"Change this motor's ID? (currently ID {motor_id}, {model})",
        default=(motor_id == 1),
    ):
        target_id = prompt_for_id(current_id=motor_id)
        if target_id is not None:
            write_id_and_baudrate(bus, model, baudrate, motor_id, target_id)
            verify_id(bus, model, target_id)
            active_id = target_id

    half = (MODEL_RESOLUTION[model] - 1) // 2
    if not ask_yes_no(
        f"Drive shaft to mid-encoder ({half}) for horn mounting?\n"
        "  WARNING: Only if the output is free to rotate (no linkage / hard stops).",
        default=True,
    ):
        return

    try:
        before = read_reg(bus, model, active_id, "Present_Position")
        print(f"  Present before: {before}  ->  commanding Goal {half}")
        present = move_to_mid_encoder(bus, model, active_id)
    except Exception as exc:  # noqa: BLE001 - keep the provisioning loop alive
        print(f"  Mid-encoder move failed: {exc}")
        print("  Torque disabled; you can retry or mount manually.")
        return

    if abs(present - half) <= 8:
        print(
            f"  Mid-encoder reached (Present={present}). "
            "Mount the horn at mid-travel, then calibrate."
        )
    else:
        print(
            f"  Present={present} (wanted ~{half}). Shaft may be blocked — "
            "check free rotation and retry."
        )
