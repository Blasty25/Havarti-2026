// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTunableNumber;

/** Add your docs here. */
public class ArmConstants {
  public class Mechanism {
    public static final double gearing = 1.0 / 39.85;
    public static final boolean inverted = true;
    public static final double tolerance = 0.02;
    public static final double armLength = Units.inchesToMeters(28);
    public static final double armMass = 10;
  }

  public class MotorLimits {
    public static final int maxSupply = 60;
    public static final int maxStator = 90;
    public static final int peakCurrent = 120;
    public static final double maxVelocity = 0.2;
    public static final double maxAcceleration = 0.2;
  }

  public class Tuning {
    public static final LoggedTunableNumber kP =
        new LoggedTunableNumber("/Arm/kP", 40.691); // 31.254
    public static final LoggedTunableNumber kI = new LoggedTunableNumber("/Arm/kI", 0.0);
    public static final LoggedTunableNumber kD = new LoggedTunableNumber("/Arm/kD", 0.0); // 6.4986
    public static final LoggedTunableNumber kS = new LoggedTunableNumber("/Arm/kS", 0.059292);
    public static final LoggedTunableNumber kV = new LoggedTunableNumber("/Arm/kV", 0.0); // 28.931
    public static final LoggedTunableNumber kA = new LoggedTunableNumber("/Arm/kA", 0.0); // 1.5141
    public static final LoggedTunableNumber kG = new LoggedTunableNumber("/Arm/kG", 0.9272);
  }
}
