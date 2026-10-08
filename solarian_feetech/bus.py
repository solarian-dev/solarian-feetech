"""
Feetech Actuator Bus

Bus implementation for communicating with Feetech servos.
"""

import logging
from typing import Dict, Any

import serial.tools.list_ports

from solarian.core.data import PosData, SpeedData, TorqueEnableData, Data, TemperatureData
from . import provision
from .driver.feetech import FeetechMotorsBus
from solarian.core.calibration import Motor, MotorCalibration, MotorNormMode

from solarian.core.component import Component
# from solarian.core.configs import ValueName
from solarian.core.actuator_bus import ActuatorBus

logger = logging.getLogger(__name__)


# WRITE_MAP = {
#     ValueName.POS: "Goal_Position",
#     ValueName.SPEED: "Goal_Velocity",
#     ValueName.TORQUE_ENABLE: "Torque_Enable",
# }

# READ_MAP = {
#     ValueName.POS: "Present_Position",
#     ValueName.SPEED: "Present_Velocity",
# }

WRITE_MAP = {
    PosData: "Goal_Position",
    SpeedData: "Goal_Velocity",
    TorqueEnableData: "Torque_Enable",
}

READ_MAP = {
    PosData: "Present_Position",
    SpeedData: "Present_Velocity",
    TemperatureData: "Present_Temperature",
}

def map_write(data_type: type[Data]):
    if data_type in WRITE_MAP:
        return WRITE_MAP[data_type]
    else:
        raise ValueError(f"Unsupported data type: {data_type}")

def map_read(data_type: type[Data]):
    if data_type in READ_MAP:
        return READ_MAP[data_type]
    else:
        raise ValueError(f"Unsupported data type: {data_type}")

def map_motor_mode(mode: str):
    if mode == "range_0_100":
        return MotorNormMode.RANGE_0_100
    elif mode == "range_m100_100":
        return MotorNormMode.RANGE_M100_100
    elif mode == "degrees":
        return MotorNormMode.DEGREES
    else:
        raise ValueError(f"Unknown motor mode: {mode}")


#: Feetech grouped sync reads get corrupted ("Incorrect status packet" / "no
#: status packet") when motors that don't share the same response timing sit in
#: one frame, even though each reads fine on its own. In practice this happens
#: for mixed *models* and also for the same model running mixed *firmware* (e.g.
#: 3.9 vs 3.10), whose status-packet timing differs enough to collide. We
#: therefore split reads into per-(model, firmware) chunks; this cap
#: additionally bounds the chunk size for very large homogeneous chains.
SYNC_READ_MAX_GROUP = 7


def _firmware_by_name(bus: FeetechMotorsBus) -> Dict[str, str | None]:
    """Return ``{motor_name: firmware_version}`` read once and cached on *bus*.

    Firmware version is a read-only EEPROM field, so it is read a single time
    (the first grouped read after connect) and memoized; subsequent calls are
    free. Motors whose firmware can't be read map to ``None`` and are grouped by
    model alone.
    """
    cache = getattr(bus, "_firmware_version_cache", None)
    if cache is None:
        by_id = bus._read_firmware_version([bus.motors[n].id for n in bus.motors])
        cache = {name: by_id.get(bus.motors[name].id) for name in bus.motors}
        bus._firmware_version_cache = cache
    return cache


def chunk_by_model(
    bus: FeetechMotorsBus, motor_names: list[str], max_group: int = SYNC_READ_MAX_GROUP
) -> list[list[str]]:
    """Split motors into groups that sync-read reliably.

    Motors are grouped by ``(model, firmware version)`` -- a single frame
    spanning mixed models *or* mixed firmware drops packets -- and each group is
    capped at ``max_group`` motors.
    """
    firmware = _firmware_by_name(bus)
    by_key: Dict[tuple[str, str | None], list[str]] = {}
    for name in motor_names:
        key = (bus.motors[name].model, firmware.get(name))
        by_key.setdefault(key, []).append(name)

    groups: list[list[str]] = []
    for names in by_key.values():
        for i in range(0, len(names), max_group):
            groups.append(names[i : i + max_group])
    return groups


def grouped_sync_read(
    bus: FeetechMotorsBus,
    data_name: str,
    motor_names: list[str],
    *,
    normalize: bool = True,
    max_group: int = SYNC_READ_MAX_GROUP,
) -> Dict[str, Any]:
    """Read ``data_name`` from every motor using chunked group sync reads.

    Motors are chunked with :func:`chunk_by_model` and each chunk is read with a
    single (retried) sync read; if a chunk still fails it falls back to reliable
    per-motor reads for that chunk only.
    """
    result: Dict[str, Any] = {}
    for group in chunk_by_model(bus, motor_names, max_group):
        try:
            result.update(bus.sync_read(data_name, motors=group, normalize=normalize, num_retry=10))
        except ConnectionError:
            logger.warning(
                "Feetech grouped sync read failed for %s; using per-motor reads.",
                group,
            )
            for name in group:
                result[name] = bus.read(data_name, name, normalize=normalize, num_retry=10)
    return result


class FeetechBus(ActuatorBus):
    """
    Actuator bus for Feetech servos.
    
    Handles communication with Feetech motors via serial port.
    Automatically used when FeetechServo actuators are defined in a module.
    """

    #: Max motors per grouped sync read (see :data:`SYNC_READ_MAX_GROUP`).
    _SYNC_READ_MAX_GROUP = SYNC_READ_MAX_GROUP

    def __init__(self, port: str, components: Dict[str, Component], calibration: Dict[str, MotorCalibration]):
        super().__init__(port, components, calibration)

        self.bus = FeetechMotorsBus(
            port=port,
            motors={
                name: Motor(component.cfg.id, component.cfg.model, map_motor_mode(component.cfg.mode)) 
                for name, component in components.items()
            },
            calibration=calibration
        )

    def is_connected(self) -> bool:
        return self.bus.is_connected

    def connect(self):
        self.bus.connect()
        # Software center_offset is the only wrap shift. A leftover EEPROM
        # Homing_Offset (from older calibrations) would double-apply and send
        # hold-present / 0 cm to the wrong end of travel.
        names = list(self.components.keys())
        self.bus.disable_torque(names)
        for name in names:
            self._write_open_hardware_limits(name)

    def disconnect(self):
        if self.bus.is_connected:
            self.bus.disconnect()

    def read(self, data_type: type[Data], component_names: list[str], normalize: bool = True):
        raw = grouped_sync_read(
            self.bus,
            map_read(data_type),
            component_names,
            normalize=False,
            max_group=self._SYNC_READ_MAX_GROUP,
        )
        if normalize and data_type is PosData:
            return self._normalize({name: float(val) for name, val in raw.items()})
        return raw

    def write(self, values: Dict[str, Data], normalize: bool = True):
        values_by_class: dict[type[Data], dict[str, Any]] = {}
        for name, value in values.items():
            values_by_class.setdefault(value.__class__, {})[name] = value.value
        for class_type, class_values in values_by_class.items():
            data_name = map_write(class_type)
            if normalize and class_type is PosData:
                class_values = {
                    name: int(round(val))
                    for name, val in self._unnormalize(class_values).items()
                }
            else:
                class_values = {name: int(val) for name, val in class_values.items()}
            self.bus.sync_write(data_name, values=class_values, normalize=False)

    def counts_per_revolution(self, name: str) -> int | None:
        """Encoder resolution (counts/rev) for motor ``name`` from its model.

        Powers the servo-agnostic IK-alignment angular hook: Feetech STS/SMS
        servos are 4096 counts/rev per :data:`MODEL_RESOLUTION`.
        """
        try:
            model = self.bus.motors[name].model
            return int(self.bus.model_resolution_table[model])
        except Exception:
            return None

    def center_offset_in_hardware(self, name: str) -> bool:
        del name
        return False

    def max_homing_offset_abs(self, name: str) -> int | None:
        """Hardware ``Homing_Offset`` register cap (STS/SMS sign bit 11 → ±2047).

        Unused while :meth:`center_offset_in_hardware` is False; software
        ``center_offset`` is capped at CPR-1 instead.
        """
        from .driver.feetech.tables import MODEL_ENCODING_TABLE

        try:
            model = self.bus.motors[name].model
            sign_bit = MODEL_ENCODING_TABLE.get(model, {}).get("Homing_Offset")
            if sign_bit is None:
                return None
            return (1 << int(sign_bit)) - 1
        except Exception:
            return None

    def is_calibrated(self) -> bool:
        if not self.calibration:
            return False
        return all(name in self.calibration for name in self.components)

    def reset_calibration(self, component_names: list[str]):
        if self.calibration is None:
            self.calibration = {}
        for name in component_names:
            self._write_open_hardware_limits(name)
            cfg = self.components[name].cfg
            cpr = self.counts_per_revolution(name) or 4096
            self.calibration[name] = MotorCalibration(
                id=int(cfg.id) if cfg is not None else 0,
                drive_mode=0,
                homing_offset=0,
                range_min=0,
                range_max=cpr - 1,
            )
        self.bus.calibration = dict(self.calibration)

    def read_calibration(self) -> dict[str, MotorCalibration]:
        return dict(self.calibration or {})

    def _write_open_hardware_limits(self, name: str) -> None:
        """Homing_Offset=0 and full encoder travel, so software owns the wrap shift.

        A leftover EEPROM Homing_Offset (from older calibrations) would double-apply
        on top of ``center_offset`` and send hold-present / 0 cm to the wrong end of
        travel, so the write is read back.
        """
        motor = self.bus.motors[name]
        provision.open_position_registers(self.bus, motor.model, motor.id)

    def drive_to_encoder_center(self, component_names: list[str]) -> Dict[str, int]:
        reached: Dict[str, int] = {}
        for name in component_names:
            motor = self.bus.motors[name]
            half = ((self.counts_per_revolution(name) or 4096) - 1) // 2
            present = provision.move_to_mid_encoder(self.bus, motor.model, motor.id)
            reached[name] = present
            if abs(present - half) > 8:
                logger.warning(
                    "%s: reached %d, wanted ~%d. Shaft may be blocked.", name, present, half
                )
        return reached

    # -- provisioning (no declared components: servos are addressed by ID) --
    @classmethod
    def available_ports(cls) -> list[str]:
        return [p.device for p in serial.tools.list_ports.comports()]

    @classmethod
    def scan_port(cls, port: str) -> list[dict]:
        return provision.scan(port)

    @classmethod
    def provision_servo(cls, port: str, found: dict) -> None:
        bus = provision.open_bus(port)
        try:
            provision.provision(bus, int(found["baudrate"]), int(found["id"]), str(found["model"]))
        finally:
            provision.close_bus(bus)

    def write_calibration(self, calibration_dict: dict[str, MotorCalibration]) -> dict[str, MotorCalibration]:
        if self.calibration is None:
            self.calibration = {}
        for name, calibration in calibration_dict.items():
            cpr = self.counts_per_revolution(name) or 4096
            calibration.range_min = max(int(calibration.range_min), 0)
            calibration.range_max = min(int(calibration.range_max), cpr - 1)
            self.calibration[name] = calibration
            self._write_open_hardware_limits(name)
        self.bus.calibration = dict(self.calibration)
        return calibration_dict

    def configure(self, params: dict[str, int], component_names: list[str]):
        for name in component_names:
            for param_name, value in params.items():
                self.bus.write(param_name, name, value, normalize=False)

    def enable_torque(self, component_names: list[str]):
        self.bus.enable_torque(component_names)

    def disable_torque(self, component_names: list[str]):
        self.bus.disable_torque(component_names)

    def start_calibration_mode(self):
        """Sets the actuators where the arm can be moved manually freely"""
        names = list(self.components.keys())
        self.bus.disable_torque(names)
        # A non-zero Return_Delay_Time (the factory default on fresh servos)
        # makes grouped sync reads collide ("Incorrect status packet!").
        # Torque is now disabled (so the EPROM lock is cleared), so force the
        # recommended value of 0 to guarantee reliable reads during calibration.
        for name in names:
            self.bus.write("Return_Delay_Time", name, 0, normalize=False)

    def end_calibration_mode(self):
        pass
        
    def set_half_turn_homings(self, component_names: list[str]) -> Dict[str, int]:
        """Software ``center_offset`` so the current pose reads mid-encoder.

        Hardware Homing_Offset stays 0. Feetech group sync reads get corrupted
        once too many motors (or mixed models) share one frame, so this uses
        the chunked, retrying read path instead of the vendored no-retry helper.
        """
        for name in component_names:
            self._write_open_hardware_limits(name)

        positions = grouped_sync_read(
            self.bus,
            "Present_Position",
            component_names,
            normalize=False,
            max_group=self._SYNC_READ_MAX_GROUP,
        )

        if self.calibration is None:
            self.calibration = {}
        homing_offsets: Dict[str, int] = {}
        for name in component_names:
            raw = int(positions[name])
            cpr = self.counts_per_revolution(name) or 4096
            offset = raw - ((cpr - 1) // 2)
            entry = self.calibration.get(name)
            if entry is None:
                cfg = self.components[name].cfg
                entry = MotorCalibration(
                    id=int(cfg.id) if cfg is not None else 0,
                    drive_mode=0,
                    homing_offset=offset,
                    range_min=0,
                    range_max=cpr - 1,
                )
                self.calibration[name] = entry
            else:
                entry.homing_offset = offset
            homing_offsets[name] = offset
        self.bus.calibration = dict(self.calibration)
        return homing_offsets

    def setup_component(self, component_name: str, configs: Dict[str, Any]):
        """Setup config for a component"""
        print("Setting the following configs for component:", component_name)
        print(configs)
        input("Connect the servo to the controller, make sure only the servo to be configured is connected. Press Enter when ready.")
        self.bus.setup_motor(component_name, initial_baudrate=configs["baudrate"], initial_id=configs["id"])
        print("Configuration completed.")
