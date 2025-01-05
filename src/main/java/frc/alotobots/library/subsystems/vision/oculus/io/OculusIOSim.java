/*
* ALOTOBOTS - FRC Team 5152
  https://github.com/5152Alotobots
* Copyright (C) 2024 ALOTOBOTS
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Source code must be publicly available on GitHub or an alternative web accessible site
*/
package frc.alotobots.library.subsystems.vision.oculus.io;

/** Simulation implementation of OculusIO that provides static test values. */
public class OculusIOSim implements OculusIO {
  @Override
  public void updateInputs(OculusIOInputs inputs) {
    // Simulate static position for testing
    inputs.position = new float[] {0.0f, 0.0f, 0.0f};
    inputs.quaternion = new float[] {1.0f, 0.0f, 0.0f, 0.0f};
    inputs.eulerAngles = new float[] {0.0f, 0.0f, 0.0f};
    inputs.timestamp = -1.0;
    inputs.frameCount = -1;
    inputs.batteryPercent = 100.0;
    inputs.misoValue = 0;
  }

  @Override
  public void setMosi(int value) {
    // Simulation does not need to handle MOSI values
  }

  @Override
  public void setResetPose(double x, double y, double rotation) {
    // Simulation does not need to handle pose reset
  }
}
