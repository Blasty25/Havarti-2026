// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import org.littletonrobotics.junction.AutoLog;

public interface GripperIO {
  @AutoLog
  public class GripperIOInputs {
    public double Voltage = 0.0;
    public double Velocity = 0.0;
    public double StatorCurrent = 0.0;
    public double SupplyCurrent = 0.0;
    public double Temperature = 0.0;
    public boolean isConnected = false;
    public boolean isDetected = false;
  }

  public default void updateInputs(GripperIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setDetected(boolean detected) {}
}
