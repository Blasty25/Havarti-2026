// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

/** Add your docs here. */
public class GripperConstants {
  public static class VoltageStates {
    // TODO: Change these
    public static final double hold = 12.0;
    public static final double intake = 12.0;
    public static final double outtake = -12.0;
  }

  public static class Current {
    public static final int maxSupply = 60;
    public static final int maxStator = 90;
    public static final int peakCurrent = 120;
    public static final int freeCurrent = 80;
  }
}
