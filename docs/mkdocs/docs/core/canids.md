# CAN Devices

This page documents all CAN devices used in the robot, their IDs, and their associated bus.

## Device Table

### Control and Power Devices
| ID | Device Name   | Abbreviation | CAN Bus      |
|----|---------------|--------------|--------------|
| 1  | Rev PDH       | RPDH         | CAN 2.0 (2)  |
| 3  | Pigeon Gyro   | GYRO         | CANivore (1) |
| 4  | Rev Servo Hub | RSVH         | CAN 2.0 (2)  |

### Drivetrain (Swerve)
| ID | Device Name             | Abbreviation | CAN Bus      |
|----|-------------------------|--------------|--------------|
| 10 | Front Left Drive Motor  | FLDR         | CANivore (1) |
| 11 | Front Left Steer Motor  | FLST         | CANivore (1) |
| 12 | Front Left Encoder      | FLEC         | CANivore (1) |
| 13 | Front Right Drive Motor | FRDR         | CANivore (1) |
| 14 | Front Right Steer Motor | FRST         | CANivore (1) |
| 15 | Front Right Encoder     | FREC         | CANivore (1) |
| 16 | Back Left Drive Motor   | BLDR         | CANivore (1) |
| 17 | Back Left Steer Motor   | BLST         | CANivore (1) |
| 18 | Back Left Encoder       | BLEC         | CANivore (1) |
| 19 | Back Right Drive Motor  | BRDR         | CANivore (1) |
| 20 | Back Right Steer Motor  | BRST         | CANivore (1) |
| 21 | Back Right Encoder      | BREC         | CANivore (1) |

### Elevator
| ID | Device Name          | Abbreviation | CAN Bus     |
|----|----------------------|--------------|-------------|
| 30 | Left Elevator Motor  | LELV         | CAN 2.0 (2) |
| 31 | Right Elevator Motor | RELV         | CAN 2.0 (2) |
| 32 | Elevator CANrange    | ELCR         | CAN 2.0 (2) |

### Wrist and Intake
| ID | Device Name         | Abbreviation | CAN Bus     |
|----|---------------------|--------------|-------------|
| 33 | Wrist Angle Motor   | WRAG         | CAN 2.0 (2) |
| 34 | Wrist Angle Encoder | WREC         | CAN 2.0 (2) |
| 35 | Intake Motor        | INTK         | CAN 2.0 (2) |
| 36 | Intake CANrange     | INCR         | CAN 2.0 (2) |

### LED Control
| ID | Device Name | Abbreviation | CAN Bus     |
|----|-------------|--------------|-------------|
| 40 | CANdle      | CNDL         | CAN 2.0 (2) |

## CAN Bus Summary

The robot uses two CAN buses:
1. **CANivore (1)** - Primary bus for drivetrain components (motors, encoders, and gyro)
2. **CAN 2.0 (2)** - Secondary bus for mechanism components and power distribution

## Subsystem Distribution

### Drivetrain (Swerve)
All swerve drive components are on the CANivore bus, including:
- All drive motors (IDs 10, 13, 16, 19)
- All steer motors (IDs 11, 14, 17, 20)
- All encoders (IDs 12, 15, 18, 21)
- Pigeon Gyro (ID 3)

### Mechanisms
All mechanism components are on the CAN 2.0 bus, including:
- Elevator motors and sensors (IDs 30, 31, 32)
- Wrist components (IDs 33, 34)
- Intake components (IDs 35, 36)
- CANdle for LED control (ID 40)

### Power and Control
- Rev PDH (ID 1) - Power Distribution Hub on CAN 2.0
- Rev Servo Hub (ID 4) - For controlling servos on CAN 2.0

## Configuration and Setup

When configuring devices in code, ensure:
1. The correct CAN bus is specified for each device
2. IDs match this documentation
3. Firmware is up-to-date on all devices

## Related Documentation

- [Swerve Subsystem](/5152_Reefscape/library/subsystems/swerve)
- [Elevator Subsystem](/5152_Reefscape/reefscape/subsystems/elevator)
- [Wrist Subsystem](/5152_Reefscape/reefscape/subsystems/wrist)
- [Intake Subsystem](/5152_Reefscape/reefscape/subsystems/intake)
- [Bling Subsystem](/5152_Reefscape/library/subsystems/bling)
