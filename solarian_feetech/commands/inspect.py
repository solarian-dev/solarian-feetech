"""``solarian feetech-inspect`` — list motors on a Feetech bus, with firmware.

Feetech's bootloader protocol is proprietary and unpublished. Firmware
*write* still requires the official Windows FD tool; this command only
*reads* the version registers so mixed-model / mixed-firmware daisy chains
can be diagnosed on Linux.
"""

import argparse
import sys

from solarian.cli import BaseCommand, CommandRegistry

from ..lerobot.feetech import FeetechMotorsBus
from ..lerobot.motors_bus import get_address
from ..scan import MODEL_NB_TO_NAME, prompt_for_port, scan_motors


@CommandRegistry.register("feetech-inspect")
class InspectCommand(BaseCommand):
    help = "Scan a Feetech bus and print each motor's model and firmware version."

    def handle(self, cfg) -> None:
        args = self._parse_args()

        port = args.port or prompt_for_port()
        if port is None:
            print("No port selected. Aborting.")
            return

        bus = FeetechMotorsBus(port, {})
        bus._connect(handshake=False)
        try:
            found = scan_motors(bus, only_baudrate=args.baudrate)
            if not found:
                print(
                    "\nNo motors responded on this port.\n"
                    "  - Check power and the data cable.\n"
                    "  - If several brand-new motors are chained together they all share\n"
                    "    ID 1 and collide; connect only one new motor at a time.\n"
                )
                return

            self._print_report(bus, found)
        finally:
            try:
                bus.port_handler.closePort()
            except Exception:  # noqa: BLE001 - best-effort cleanup
                pass

    def _parse_args(self) -> argparse.Namespace:
        parser = argparse.ArgumentParser(prog="solarian feetech-inspect", add_help=False)
        parser.add_argument(
            "--port",
            default=None,
            help="Serial port of the bus (e.g. /dev/ttyACM0). Prompted if omitted.",
        )
        parser.add_argument(
            "--baudrate",
            type=int,
            default=None,
            help="Only scan this baud-rate (faster). Otherwise all rates are scanned.",
        )
        args, _ = parser.parse_known_args(sys.argv[2:])
        return args

    def _print_report(self, bus: FeetechMotorsBus, found: dict[int, dict[int, int]]) -> None:
        print("\n=== Feetech bus ===")
        print(f"{'id':>4}  {'model':<12}  {'firmware':<10}  {'return_delay':>12}")
        firmware_by_id: dict[int, str] = {}
        models: list[str] = []
        for baudrate in sorted(found):
            bus.set_baudrate(baudrate)
            for motor_id in sorted(found[baudrate]):
                model_nb = found[baudrate][motor_id]
                model = MODEL_NB_TO_NAME.get(model_nb, f"unknown({model_nb})")
                models.append(model)
                firmware = self._firmware(bus, motor_id)
                firmware_by_id[motor_id] = firmware
                delay = self._return_delay(bus, motor_id, model)
                delay_s = "—" if delay is None else str(delay)
                print(f"{motor_id:>4}  {model:<12}  {firmware:<10}  {delay_s:>12}")

        versions = {v for v in firmware_by_id.values() if v != "—"}
        if len(set(models)) > 1:
            print(
                "\nMixed models on one daisy chain. Grouped sync reads already "
                "split by model; unicast still works if Return_Delay_Time is 0."
            )
        if len(versions) > 1:
            print(
                "\nMixed firmware versions: "
                + ", ".join(sorted(versions))
                + ".\nFeetech has no public Linux flashing protocol — matching "
                "firmware still requires the Windows FD tool "
                "(https://www.feetechrc.com/software)."
            )

    def _firmware(self, bus: FeetechMotorsBus, motor_id: int) -> str:
        versions = bus._read_firmware_version([motor_id], raise_on_error=False)
        return versions.get(motor_id, "—")

    def _return_delay(self, bus: FeetechMotorsBus, motor_id: int, model: str) -> int | None:
        if model not in bus.model_ctrl_table:
            return None
        addr, length = get_address(bus.model_ctrl_table, model, "Return_Delay_Time")
        bus.port_handler.clearPort()
        value, comm, _error = bus._read(addr, length, motor_id, raise_on_error=False)
        if not bus._is_comm_success(comm):
            return None
        return int(value)
