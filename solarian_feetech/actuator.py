"""
Feetech Servo Actuator

Actuator class for Feetech motors (e.g., STS3215, SCS series).
"""

from solarian.core.servo import Servo
from solarian.core.data import PosData, SpeedData

from solarian_feetech.bus import FeetechBus


class FeetechServo(Servo):
    """
    Servo actuator for Feetech motors (e.g., STS3215, SCS series).
    
    Retrieve this class from the plugin registry:
    
        from solarian.plugins import actuators
        FeetechServo = actuators.get("feetech")
        
        class MyArmModule(Module):
            shoulder: FeetechServo = FeetechServo(id=1, model="sts3215")
            elbow: FeetechServo = FeetechServo(id=2, model="sts3215")
    
    The corresponding FeetechBus will be automatically used for communication.
    """
    
    bus_class = FeetechBus

    pos = PosData()
    speed = SpeedData()
    
    