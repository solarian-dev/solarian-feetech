"""
Feetech Actuator Bus

Bus implementation for communicating with Feetech servos.
"""

from typing import Dict, Any

from solarian.core.data import PosData, SpeedData, TorqueEnableData, Data
from .lerobot.feetech import FeetechMotorsBus
from .lerobot.feetech.feetech import NORMALIZED_DATA
from solarian.core.calibration import Motor, MotorCalibration, MotorNormMode

from solarian.core.component import Component
# from solarian.core.configs import ValueName
from solarian.core.actuator_bus import ActuatorBus


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


class FeetechBus(ActuatorBus):
    """
    Actuator bus for Feetech servos.
    
    Handles communication with Feetech motors via serial port.
    Automatically used when FeetechServo actuators are defined in a module.
    """

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
        
    def disconnect(self):
        if self.bus.is_connected:
            self.bus.disconnect()

    def read(self, data_type: type[Data], component_names: list[str], normalize: bool = True):
        return self.bus.sync_read(map_read(data_type), motors=component_names, normalize=normalize, num_retry=10)

    def write(self, values: Dict[str, Data], normalize: bool = True):
        # group values by class type, extracting raw values from Data objects
        values_by_class = {}
        for name, value in values.items():
            if value.__class__ not in values_by_class:
                values_by_class[value.__class__] = {}
            values_by_class[value.__class__][name] = value.value
        for class_type, class_values in values_by_class.items():
            data_name = map_write(class_type)
            if not normalize or data_name not in NORMALIZED_DATA:
                class_values = {k: int(v) for k, v in class_values.items()}
            self.bus.sync_write(data_name, values=class_values, normalize=normalize)

    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated

    def reset_calibration(self, component_names: list[str]):
        self.bus.reset_calibration(component_names)

    def read_calibration(self) -> dict[str, MotorCalibration]:
        return self.bus.read_calibration()
        
    def write_calibration(self, calibration_dict: dict[str, MotorCalibration]) -> dict[str, MotorCalibration]:
        # bound min and max to range of motion
        for name, calibration in calibration_dict.items():
            calibration.range_min = max(calibration.range_min, 0)
            calibration.range_max = min(calibration.range_max, 4095)
        self.bus.write_calibration(calibration_dict)
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
        self.bus.disable_torque(list(self.components.keys()))
        # self.write(TorqueEnableData(value=False), {name: 0 for name in list(self.components.keys())})

    def end_calibration_mode(self):
        pass
        
    def set_half_turn_homings(self, component_names: list[str]) -> Dict[str, int]:
        """Set half turn homings for the actuators. Read present position and set/store"""
        return self.bus.set_half_turn_homings(component_names)

    def setup_component(self, component_name: str, configs: Dict[str, Any]):
        """Setup config for a component"""
        print("Setting the following configs for component:", component_name)
        print(configs)
        input("Connect the servo to the controller, make sure only the servo to be configured is connected. Press Enter when ready.")
        self.bus.setup_motor(component_name, initial_baudrate=configs["baudrate"], initial_id=configs["id"])
        print("Configuration completed.")
