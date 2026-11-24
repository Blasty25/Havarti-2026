// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private final ArmIO io;
  private final ArmDataAutoLogged data = new ArmDataAutoLogged();
  public double elevatorSetpoint = 0.0;
  private final SysIdRoutine routine;

  // This is armed with knowledge.
  public Arm(ArmIO io) {
    this.io = io;

    routine =
        new SysIdRoutine(
            new Config(null, Volts.of(3.0), null),
            new Mechanism(
                (voltIn) -> io.runVolts(voltIn.in(Volts)),
                (log) -> {
                  log.motor("Arm")
                      .voltage(Volts.of(data.MotorVoltage))
                      .current(Amps.of(data.MotorStatorCurrent))
                      .angularPosition(Rotations.of(data.ArmAngle))
                      .angularVelocity(
                          RotationsPerSecond.of(Units.radiansToRotations(data.ArmVelocity)));
                },
                this));
  }

  public Command positionCommand(DoubleSupplier angle) {
    return Commands.run(
        () -> {
          io.setAngle(angle.getAsDouble());
        });
  }

  public Command resetEncoder() {
    return Commands.runOnce(
        () -> {
          io.resetEncoder();
        });
  }

  // returns rads
  public double getAngle() {
    return data.ArmAngle;
  }

  public Command sysID() {
    return Commands.sequence(
        routine
            .dynamic(Direction.kForward)
            .until(() -> MathUtil.isNear(-Math.PI / 2, this.getAngle(), 0.02)),
        routine
            .dynamic(Direction.kReverse)
            .until(() -> MathUtil.isNear(-Math.PI / 2, this.getAngle(), 0.02)),
        routine
            .quasistatic(Direction.kForward)
            .until(() -> MathUtil.isNear(-Math.PI / 2, this.getAngle(), 0.02)),
        routine
            .quasistatic(Direction.kReverse)
            .until(() -> MathUtil.isNear(-Math.PI / 2, this.getAngle(), 0.02)));
  }

  @AutoLogOutput(key = "Arm/Setpoint")
  public boolean atSetpoint() {
    return MathUtil.isNear(
        data.setpoint, data.ArmAngle, ArmConstants.Mechanism.tolerance);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.getData(data);
    Logger.processInputs("Arm", data);
  }
}
