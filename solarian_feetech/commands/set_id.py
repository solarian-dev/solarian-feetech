"""``solarian feetech-set-id`` — interactively assign IDs to Feetech motors.

Fresh Feetech servos all ship with the same ID (1) and therefore collide on a
shared bus, which makes them impossible to address individually.  This command
lets you provision a daisy chain *without unplugging* anything:

1. Scan the bus across every supported baud-rate and list the motors found.
2. Pick a target motor (by default the one still at the factory ID ``1`` — i.e.
   the motor you just added to the chain).
3. Apply recommended comms settings (``Return_Delay_Time = 0``) to it.
4. Optionally write a new ID to it.
5. Optionally drive the shaft to mid-encoder so you can mount the horn at the
   mechanism's mid-travel (keeps later joint travel away from the 0/CPR wrap).
6. Repeat for the next motor.

Because ``Return_Delay_Time`` is applied to whichever motor you select, you can
also re-run this command purely to fix the comms settings on already-configured
motors (just decline the ID change).

Because every already-configured motor has a unique, non-default ID, the newly
added motor is always the only one still at ID 1 — so it can be identified by
its default ID and reassigned safely while the rest of the chain stays
connected (no motion required).
"""

import argparse
import sys
import time

from solarian.cli import BaseCommand, CommandRegistry

from ..lerobot.common.utils.encoding_utils import decode_sign_magnitude, encode_sign_magnitude
from ..lerobot.feetech import FeetechMotorsBus, OperatingMode, TorqueMode
from ..lerobot.motors_bus import get_address
from ..lerobot.feetech.tables import (
    MODEL_BAUDRATE_TABLE,
    MODEL_ENCODING_TABLE,
    MODEL_NUMBER_TABLE,
    MODEL_RESOLUTION,
)
from ..scan import MODEL_NB_TO_NAME, prompt_for_port, scan_motors


@CommandRegistry.register("feetech-set-id")
class SetIdCommand(BaseCommand):
    help = "Scan a Feetech bus, apply recommended comms settings, and assign a motor ID."

    def handle(self, cfg) -> None:
        args = self._parse_args()

        port = args.port or prompt_for_port()
        if port is None:
            print("No port selected. Aborting.")
            return

        # A single bus with no declared motors is enough for scanning and for
        # the low-level (by-ID) reads/writes we perform below.
        bus = FeetechMotorsBus(port, {})
        bus._connect(handshake=False)
        try:
            while True:
                found = scan_motors(bus, only_baudrate=args.baudrate)
                if not found:
                    print(
                        "\nNo motors responded on this port.\n"
                        "  - Check power and the data cable.\n"
                        "  - If several brand-new motors are chained together they all share\n"
                        "    ID 1 and collide; connect only one new motor at a time.\n"
                    )
                    if not self._ask_yes_no("Scan again?", default=True):
                        break
                    continue

                self._print_found(found)
                target = self._select_target(found)
                if target is None:
                    break

                baudrate, motor_id, model_nb = target
                model = MODEL_NB_TO_NAME.get(model_nb)
                if model is None:
                    print(
                        f"Motor at ID {motor_id} reports unknown model number {model_nb}; "
                        "cannot configure it."
                    )
                    if not self._ask_yes_no("Set up another motor?", default=True):
                        break
                    continue

                # Always apply the recommended comms settings (Return_Delay_Time=0)
                # to the selected motor. This is written to EPROM so it persists,
                # and it keeps multi-motor grouped sync reads reliable.
                self._apply_comms_config(bus, baudrate, motor_id, model)

                active_id = motor_id
                if self._ask_yes_no(
                    f"Change this motor's ID? (currently ID {motor_id}, {model})",
                    default=(motor_id == 1),
                ):
                    target_id = self._prompt_for_id(current_id=motor_id)
                    if target_id is not None:
                        self._write_id_and_baudrate(bus, model, baudrate, motor_id, target_id)
                        self._verify(bus, target_id, model)
                        active_id = target_id

                half = (MODEL_RESOLUTION[model] - 1) // 2
                if self._ask_yes_no(
                    f"Drive shaft to mid-encoder ({half}) for horn mounting?\n"
                    "  WARNING: Only if the output is free to rotate (no linkage / hard stops).",
                    default=True,
                ):
                    try:
                        self._move_to_mid_encoder(bus, baudrate, active_id, model)
                    except Exception as exc:  # noqa: BLE001 - keep provisioning loop alive
                        print(f"  Mid-encoder move failed: {exc}")
                        print("  Torque disabled; you can retry or mount manually.")

                if not self._ask_yes_no("Set up another motor?", default=True):
                    break
        finally:
            try:
                bus.port_handler.closePort()
            except Exception:  # noqa: BLE001 - best-effort cleanup
                pass

        print("\nDone.")

    # ------------------------------------------------------------------ args

    def _parse_args(self) -> argparse.Namespace:
        parser = argparse.ArgumentParser(prog="solarian feetech-set-id", add_help=False)
        parser.add_argument(
            "--port",
            default=None,
            help="Serial port of the bus (e.g. /dev/ttyUSB0). Prompted if omitted.",
        )
        parser.add_argument(
            "--baudrate",
            type=int,
            default=None,
            help="Only scan this baud-rate (faster). Otherwise all rates are scanned.",
        )
        # Ignore the global CLI flags (--conf/--settings) and anything else.
        args, _ = parser.parse_known_args(sys.argv[2:])
        return args

    def _print_found(self, found: dict[int, dict[int, int]]) -> None:
        print("\n=== Motors Found ===")
        for baudrate in sorted(found):
            for motor_id in sorted(found[baudrate]):
                model_nb = found[baudrate][motor_id]
                model = MODEL_NB_TO_NAME.get(model_nb, f"unknown({model_nb})")
                print(f"  baudrate={baudrate:>8}  id={motor_id:<3}  model={model}")

    def _select_target(self, found: dict[int, dict[int, int]]):
        """Pick a motor to configure.

        Returns a ``(baudrate, id, model_number)`` tuple, or ``None`` to stop.
        """
        # Flatten to a stable, ordered list of candidates.
        candidates: list[tuple[int, int, int]] = []
        for baudrate in sorted(found):
            for motor_id in sorted(found[baudrate]):
                candidates.append((baudrate, motor_id, found[baudrate][motor_id]))

        # Default target: the (single) motor still at the factory ID 1.
        id1 = [c for c in candidates if c[1] == 1]
        if len(id1) == 1:
            baudrate, motor_id, model_nb = id1[0]
            model = MODEL_NB_TO_NAME.get(model_nb, f"unknown({model_nb})")
            if self._ask_yes_no(
                f"Target the newly-added motor at ID 1 (baudrate={baudrate}, {model})?",
                default=True,
            ):
                return id1[0]
        elif len(id1) > 1:
            print(
                "\nWarning: multiple motors answered at ID 1. They cannot be told\n"
                "apart on a shared bus — connect only ONE new (ID 1) motor at a time."
            )

        print("\n=== Select Motor ===")
        for idx, (baudrate, motor_id, model_nb) in enumerate(candidates, 1):
            model = MODEL_NB_TO_NAME.get(model_nb, f"unknown({model_nb})")
            print(f"{idx}. baudrate={baudrate} id={motor_id} model={model}")
        raw = input(f"Motor number (1-{len(candidates)}), or Enter to stop: ").strip()
        if not raw:
            return None
        if raw.isdigit() and 1 <= int(raw) <= len(candidates):
            return candidates[int(raw) - 1]
        print("Invalid selection.")
        return None

    # --------------------------------------------------------------- writing

    def _apply_comms_config(self, bus: FeetechMotorsBus, baudrate: int, motor_id: int, model: str) -> None:
        """Write recommended communication settings to a motor.

        Sets ``Return_Delay_Time`` to 0 (the factory default is ~500 us, which
        adds latency and makes multi-motor grouped sync reads collide). Torque
        is disabled first so the EPROM lock is cleared and the write persists.
        """
        bus.set_baudrate(baudrate)
        bus._disable_torque(motor_id, model)
        addr, length = get_address(bus.model_ctrl_table, model, "Return_Delay_Time")
        bus._write(addr, length, motor_id, 0, raise_on_error=True)
        print("  Set Return_Delay_Time -> 0")

    def _write_id_and_baudrate(
        self,
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

        # Write the new ID (addressed at the *current* ID).
        id_addr, id_len = get_address(bus.model_ctrl_table, model, "ID")
        bus._write(id_addr, id_len, current_id, target_id, raise_on_error=True)
        print(f"  Wrote ID {current_id} -> {target_id}")

        # Write the new baud-rate (now addressed at the *new* ID).
        baud_addr, baud_len = get_address(bus.model_ctrl_table, model, "Baud_Rate")
        baud_index = MODEL_BAUDRATE_TABLE[model][target_baudrate]
        bus._write(baud_addr, baud_len, target_id, baud_index, raise_on_error=True)
        print(f"  Set baud-rate -> {target_baudrate}")

        bus.set_baudrate(target_baudrate)

    def _verify(self, bus: FeetechMotorsBus, target_id: int, model: str) -> None:
        model_nb = bus.ping(target_id)
        expected = MODEL_NUMBER_TABLE.get(model)
        if model_nb == expected:
            print(f"  Verified: motor now responds at ID {target_id}.")
        else:
            print(
                f"  Warning: could not verify motor at ID {target_id} "
                f"(ping returned {model_nb!r})."
            )

    def _move_to_mid_encoder(
        self,
        bus: FeetechMotorsBus,
        baudrate: int,
        motor_id: int,
        model: str,
        *,
        settle_s: float = 3.0,
        tol: int = 8,
    ) -> None:
        """Drive Present/Actual to mid-encoder and leave torque off.

        Clears ``Homing_Offset`` and opens position limits so Goal mid means
        Absolute mid. Intended for bare / free shafts before horn mounting —
        do not use against a hard-stop linkage.
        """
        resolution = MODEL_RESOLUTION[model]
        half = (resolution - 1) // 2
        bus.set_baudrate(baudrate)
        bus._disable_torque(motor_id, model)

        self._write_reg(bus, model, motor_id, "Homing_Offset", 0)
        self._write_reg(bus, model, motor_id, "Min_Position_Limit", 0)
        self._write_reg(bus, model, motor_id, "Max_Position_Limit", resolution - 1)
        if "Operating_Mode" in bus.model_ctrl_table.get(model, {}):
            self._write_reg(bus, model, motor_id, "Operating_Mode", OperatingMode.POSITION.value)
        self._ensure_single_turn_feedback(bus, model, motor_id)

        present_before = self._read_reg(bus, model, motor_id, "Present_Position")
        print(f"  Present before: {present_before}  →  commanding Goal {half}")

        self._enable_torque(bus, motor_id, model)
        try:
            self._write_reg(bus, model, motor_id, "Goal_Position", half)
            deadline = time.monotonic() + settle_s
            present = present_before
            while time.monotonic() < deadline:
                present = self._read_reg(bus, model, motor_id, "Present_Position")
                if abs(present - half) <= tol:
                    break
                time.sleep(0.05)
        finally:
            bus._disable_torque(motor_id, model)

        present = self._read_reg(bus, model, motor_id, "Present_Position")
        if abs(present - half) <= tol:
            print(f"  Mid-encoder reached (Present={present}). Mount the horn at mid-travel, then calibrate.")
        else:
            print(
                f"  Present={present} (wanted ~{half}). Shaft may be blocked — "
                "check free rotation and retry."
            )

    def _ensure_single_turn_feedback(
        self, bus: FeetechMotorsBus, model: str, motor_id: int
    ) -> None:
        """Clear Phase bit 4 on STS3215 so feedback stays in [0, resolution)."""
        if model != "sts3215":
            return
        if "Phase" not in bus.model_ctrl_table.get(model, {}):
            return
        phase = self._read_reg(bus, model, motor_id, "Phase")
        if phase & 0x10:
            self._write_reg(bus, model, motor_id, "Phase", phase & ~0x10)
            print("  Cleared Phase bit 4 (single-turn feedback)")

    def _enable_torque(self, bus: FeetechMotorsBus, motor_id: int, model: str) -> None:
        addr, length = get_address(bus.model_ctrl_table, model, "Torque_Enable")
        bus._write(addr, length, motor_id, TorqueMode.ENABLED.value, raise_on_error=True)
        addr, length = get_address(bus.model_ctrl_table, model, "Lock")
        bus._write(addr, length, motor_id, 1, raise_on_error=True)

    def _write_reg(
        self, bus: FeetechMotorsBus, model: str, motor_id: int, data_name: str, value: int
    ) -> None:
        addr, length = get_address(bus.model_ctrl_table, model, data_name)
        encoded = value
        sign_bit = MODEL_ENCODING_TABLE.get(model, {}).get(data_name)
        if sign_bit is not None:
            encoded = encode_sign_magnitude(value, sign_bit)
        bus._write(addr, length, motor_id, encoded, raise_on_error=True)

    def _read_reg(self, bus: FeetechMotorsBus, model: str, motor_id: int, data_name: str) -> int:
        addr, length = get_address(bus.model_ctrl_table, model, data_name)
        raw, _, _ = bus._read(addr, length, motor_id, raise_on_error=True)
        sign_bit = MODEL_ENCODING_TABLE.get(model, {}).get(data_name)
        if sign_bit is not None:
            return decode_sign_magnitude(raw, sign_bit)
        return raw

    # ---------------------------------------------------------------- helpers

    def _prompt_for_id(self, current_id: int) -> int | None:
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

    def _ask_yes_no(self, prompt: str, default: bool = True) -> bool:
        suffix = "[Y/n]" if default else "[y/N]"
        raw = input(f"{prompt} {suffix} ").strip().lower()
        if not raw:
            return default
        return raw in ("y", "yes")
