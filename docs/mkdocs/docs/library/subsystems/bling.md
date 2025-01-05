# Bling Subsystem

The Bling subsystem controls the robot's LED lighting system using the CTRE CANdle device. It provides visual feedback about robot state through different colors and animations.

## Constructor

```java
public BlingSubsystem(BlingIO io)
```

- `io`: The BlingIO implementation to use (either BlingIOReal for hardware or BlingIOSim for simulation)

## Commands Using This Subsystem

- [No Alliance Waiting Command](/5152_Template/library/commands/bling/noalliancewaiting)
- [Set To Alliance Color Command](/5152_Template/library/commands/bling/settoalliancecolor)

## Configuration Requirements

1. CANdle CAN ID must be set in Constants file
2. LED strip configuration in BlingConstants:
    - Number of LEDs (NUM_LEDS)
    - LED strip type (LED_TYPE)
    - Maximum brightness (MAX_LED_BRIGHTNESS)
    - LED offset (LED_OFFSET)
    - Status LED state (DISABLE_STATUS_LED)

## Reference Documentation
[Bling Subsystem Javadoc](/5152_Template/javadoc/frc/alotobots/library/subsystems/bling/package-summary.html)
