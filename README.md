# Omni-directional Vehicle (ODV) Programs

This repository contains Python programs for controlling omni-directional vehicles built using LEGO® components, using [Pybricks](https://pybricks.com/). These programs are designed to run on the LEGO® Technic Hub and were written using the Pybricks development environment. The code is based on Pybricks sample programs.

The Omni-directional Vehicle system is a fan-made MOC (My Own Creation) built using LEGO® bricks. It is not an official LEGO® product.

## Overview

Omni-directional Vehicles (ODVs) are robots capable of precise multi-directional movement. These programs demonstrate various configurations for both manual and autonomous control.

The Omni-directional Vehicle system is a fan-made MOC (My Own Creation), built using LEGO® bricks. This is not an official LEGO® product. The system includes fully omnidirectional vehicles and their dedicated floor plates. These programs are specifically developed to operate with that system.

For a detailed explanation and demonstration of the Omni-directional Vehicles, please refer to the video below:

[Watch the demonstration video](https://youtu.be/EG9ISMjM-uQ?feature=shared)

Building instructions for the hardware are available for purchase at the following link:

[Building Instructions](https://www.planet-gbc.com/)

Configurations `Cfg02`, `Cfg03`, and `Cfg04` are specifically designed for use in Great Ball Contraption (GBC) modules, where vehicles transport balls between loader and unloader stations in a synchronized and visually engaging manner.
Configurations `Cfg04` and `Cfg05` utilize Bluetooth Low Energy (BLE) broadcasting to synchronize multiple vehicles during autonomous operation.

## Compatibility

These programs have been tested with **Pybricks firmware version 3.6** on the Technic Hub.

## Program List and Descriptions

Each program below includes both a functional description and a list of required hardware for setup.

### `ODV_Cfg01_RemoteControl.py`

- **Type**: Manual control
- **Requirements**:
  - Base Grid: 2×2 (minimum 4 plates)
  - ODV: 1 unit
  - Remote Control: 1 unit (LEGO 88010)
- **Purpose**: Use the LEGO remote control to move the vehicle in 8 directions
- **Deployment**: Upload to the vehicle’s Technic Hub

### `ODV_Cfg02_SingleVehicle.py`

- **Type**: Autonomous single vehicle
- **Requirements**:
  - Base Grid: 3×2 (6 plates, including 1 fenced base grid)
  - ODV: 1 unit
  - GBC Loader: 1 unit
  - GBC Unloader: 1 unit
- **Grid**: 3×2 base plates
- **Operation**: Vehicle travels between GBC loader and unloader
- **Notes**:
  - Starts from back-right corner
  - LED indicates homing and operational status
  - Adjustable speed and homing parameters

### `ODV_Cfg03_SingleVehicle.py`
- **Type**: Autonomous single vehicle
- **Requirements**:
  - Base Grid: L-shaped 4×3 (6 plates total, including 1 fenced base grid)
  - ODV: 1 unit
  - GBC Loader: 1 unit
  - GBC Unloader: 1 unit
- **Grid**: L-shaped 4×3 base plates
- **Operation**: Similar to Cfg02 but with a different layout
- **Notes**:
  - The layout connects the loader and unloader, allowing standalone operation without external modules.
  - Starts from back-right corner
  - Identical behavior logic, adapted to new geometry

### `ODV_Cfg04_MultiVehicle.py`

- **Type**: Autonomous multi-vehicle (2 units)
- **Requirements**:
  - Base Grid: 4×2 (8 plates, including 1 fenced base grid)
  - ODV: 2 units
  - GBC Loader: 1 unit
  - GBC Unloader: 1 unit
- **Grid**: 4×2 base plates
- **Operation**: Two vehicles operate in coordination
- **Deployment**:
  - Each vehicle runs the same code
  - Each must be assigned a unique `VEHICLE_ID`
- **Features**:
  - One vehicle becomes the Bluetooth synchronization sender
  - Synchronized operation after a wait period
  - Supports emergency stop via optional single sender hub

### `ODV_Cfg05_MultiVehicle.py`

- **Type**: Autonomous multi-vehicle (4 units)
- **Note**: This configuration is not GBC-specific, but demonstrates diverse synchronized behaviors among four vehicles.
- **Requirements**:
  - Base Grid: 4×4 (16 plates, including 1 fenced base grid)
  - ODV: 4 units
- **Grid**: 4×4 base plates
- **Operation**: Vehicles perform coordinated behaviors in a square grid
- **Deployment**:
  - Same as Cfg04, extended to four vehicles
  - Supports emergency stop and precise sync

### `ODV_MultiVehicle_Hub_Op.py`

- **Type**: Auxiliary sender hub program
- **Purpose**: Dedicated emergency stop and synchronization hub
- **Use Case**:
  - All vehicle-side programs must have `USE_SINGLE_SENDER_VEHICLE = False`
  - The `BROADCAST_CHANNEL` in this program must match the one used in the vehicle programs
  - Start this program on the sender hub after launching all vehicle programs
  - Only used in multi-vehicle setups (Cfg04, Cfg05)
  - Sends emergency stop if flipped
  - Improves sync timing precision

## Status Indicator Reference

| Color                 | Meaning                                                |
| --------------------- | ------------------------------------------------------ |
| Green                 | Homing or waiting to receive sync                      |
| Blue                  | Operating normally                                     |
| Yellow                | Operating - Battery at warning level                   |
| Red                   | Operating - Battery at critical level / Emergency stop |
| Magenta               | Invalid sync message received                          |
| Blinking White/Blue   | Synchronized operation (battery OK)                    |
| Blinking White/Yellow | Synchronized operation (battery warning)               |
| Blinking White/Red    | Synchronized operation (battery critical)              |

## Setup and Usage

1. Connect to the Pybricks Code Web IDE
2. Install Pybricks firmware to each Technic Hub
3. Upload the appropriate `.py` script to each hub:
   - For single-vehicle: Choose `Cfg02` or `Cfg03`
   - For multi-vehicle: Use `Cfg04` or `Cfg05` on all hubs, with unique IDs
   - Optionally upload `ODV_MultiVehicle_Hub_Op.py` to a separate sync hub
4. Position vehicles in designated starting locations (back-right corner)
5. Start all vehicle hubs, then the sender hub if used

## License

These programs are released under the MIT License. See `LICENSE` for details.

This software references Pybricks sample code, also licensed under the MIT License.

LEGO® is a trademark of the LEGO Group of companies which does not sponsor, authorize or endorse these programs.

## Acknowledgments

Special thanks to the Pybricks team for providing an outstanding development platform for LEGO robotics.

## Contact

For questions, suggestions, or collaboration opportunities, feel free to open an issue or submit a pull request.
