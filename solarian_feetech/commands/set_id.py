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

``solarian calibrate-servo`` does the same for any servo brand; this command
keeps the Feetech-only flags (``--port``, ``--baudrate``) and the "target the
motor still at ID 1" shortcut, and shares the register work through
:mod:`solarian_feetech.provision`.
"""

import argparse
import sys

from solarian.cli import BaseCommand, CommandRegistry

from .. import provision
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

        bus = provision.open_bus(port)
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
                    if not provision.ask_yes_no("Scan again?", default=True):
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
                else:
                    provision.provision(bus, baudrate, motor_id, model)

                if not provision.ask_yes_no("Set up another motor?", default=True):
                    break
        finally:
            provision.close_bus(bus)

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
            if provision.ask_yes_no(
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
