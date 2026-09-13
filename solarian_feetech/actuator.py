"""
Feetech Servo Actuator

Actuator class for Feetech motors (e.g., STS3215, SCS series).
"""

from solarian.core.servo import Servo
from solarian.core.data import PosData, SpeedData, TemperatureData

from solarian_feetech.bus import FeetechBus


class FeetechServo(Servo):
    """
    Servo actuator for Feetech motors (e.g., STS3215, SCS series).
    
    Retrieve this class from the plugin registry:
    
        from solarian.plugins import actuators
        FeetechServo = actuators.get("feetech")
        
        class MyArmModule(Module):
            shoulder = FeetechServo(model="sts3215")
            wrist_roll = FeetechServo(model="sts3215", is_continuous=True)
    
    The corresponding FeetechBus will be automatically used for communication.
    Servo id and port stay in the module yaml (deployment).
    """
    
    bus_class = FeetechBus
    default_mode = "range_m100_100"

    pos = PosData()
    speed = SpeedData()
    temperature = TemperatureData()
    
    