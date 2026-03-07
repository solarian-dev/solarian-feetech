# Solarian-Feetech

Feetech servo actuator and bus plugin for the Solarian robotics framework.

## Installation

```bash
pip install solarian-feetech
```

Or install from source:

```bash
cd solarian-feetech
pip install -e .
```

## Usage

After installation, the plugin is automatically discovered by Solarian:

```python
from solarian.core.module import Module
from solarian.plugins import actuators

# Get the Feetech actuator class
FeetechServo = actuators.get("feetech")

class MyArmModule(Module):
    shoulder: FeetechServo = FeetechServo(id=1, model="sts3215")
    elbow: FeetechServo = FeetechServo(id=2, model="sts3215")
```

The `FeetechBus` is automatically used for communication - no configuration needed!

## Supported Models

- STS3215
- SCS series
- Other Feetech servo models

## Requirements

- Solarian framework (>=0.1.0)
- feetech-servo-sdk (>=1.0.0)
- lerobot (>=0.1.0)

## Repository

This plugin should be maintained in its own repository:
- GitHub: `https://github.com/yourorg/solarian-feetech`
- Can be published to PyPI for easy installation
