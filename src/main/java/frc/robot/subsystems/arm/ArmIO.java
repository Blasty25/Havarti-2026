// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ArmIO {
  @AutoLog
  public class ArmData {
    public Rotation2d ArmAngle = Rotation2d.kZero;
    public double ArmVelocity = 0.0;
    public double ArmAcceleration = 0.0;
    public double setpoint = 0.0;

    public boolean MotorConnected = false;
    public double MotorVoltage = 0.0;
    public double MotorTemperature = 0.0;
    public double MotorStatorCurrent = 0;
    public double MotorSupplyCurrent = 0;
  }

  public default void setAngle(double rads) {}

  public default void getData(ArmData data) {}

  public default void runVolts(double volts) {}

  public default void resetEncoder() {}
}
