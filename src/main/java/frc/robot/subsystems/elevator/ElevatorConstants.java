// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.util.LoggedTunableNumber;

public class ElevatorConstants {

  // sim stuff
  public static final double elevatorMass =
      Pounds.of(30.0).in(Kilograms); // Returns The mass in kilograms
  public static final double maxHeight = 0.95; // Inches.of(43.3071).in(Meters);
  public static final double minHeight = Inches.of(0).in(Meters);
  public static final boolean simGravity = false;
  public static final DCMotor gearbox = DCMotor.getKrakenX60(2);

  public static LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/PID/kP", 5.7998);
  public static LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/PID/kI", 0.0);
  public static LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/PID/kD", 1.5227);

  public static LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/FF/kS", 0.294);
  public static LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/FF/kG", 1.0);
  public static LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/FF/kV", 8.731);
  public static LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/FF/kA", 1.5);

  // Real Robot numbers
  public static double rampRate = 0.1;
  public static double maxVelocity = 1.25;
  public static double maxAcceleration = 10.0;
  public static int statorCurrent = 120;
  public static int supplyCurrent = 80;
  public static int supplyCurrentLow = 60;

  public static final double drumRadius = (5.0 / 1000.0 * 30) / (2 * Math.PI);
  public static final double gearing = (11.25 / 1.0); // Gearing is gearing
  public static final double positionConversionFactor = drumRadius * 2 * Math.PI / gearing;
  public static final double tolerance = 0.05;

  public static final double homingVoltage = -2;
}
