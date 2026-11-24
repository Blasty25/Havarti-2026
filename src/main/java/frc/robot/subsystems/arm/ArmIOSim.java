// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.arm.ArmConstants.Tuning;

/** Add your docs here. */
public class ArmIOSim implements ArmIO {
  private SingleJointedArmSim armSim;
  private PIDController pidController =
      new PIDController(
          10.0, ArmConstants.Tuning.kI.getAsDouble(), ArmConstants.Tuning.kD.getAsDouble());
  private ArmFeedforward feedforwards = new ArmFeedforward(0.0, 0.0, 0);
  private double setpoint = 0.0;

  public ArmIOSim() {
    armSim =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60(1),
            ArmConstants.Mechanism.gearing,
            SingleJointedArmSim.estimateMOI(
                ArmConstants.Mechanism.armLength, ArmConstants.Mechanism.armMass),
            ArmConstants.Mechanism.armLength,
            -2 * Math.PI,
            2 * Math.PI,
            false,
            0.0);

    pidController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void getData(ArmData data) {
    armSim.update(0.020);
    if (ArmConstants.Tuning.kP.hasChanged(hashCode())
        || ArmConstants.Tuning.kI.hasChanged(hashCode())
        || ArmConstants.Tuning.kD.hasChanged(hashCode())
        || ArmConstants.Tuning.kG.hasChanged(hashCode())) {
      pidController.setPID(
          Tuning.kP.getAsDouble(), Tuning.kI.getAsDouble(), Tuning.kD.getAsDouble());
      feedforwards.setKg(Tuning.kG.getAsDouble());
    }

    armSim.setInputVoltage(pidController.calculate(armSim.getAngleRads()));

    data.ArmAngle = armSim.getAngleRads();
    data.ArmVelocity = armSim.getVelocityRadPerSec();
    data.MotorConnected = true;
    data.MotorStatorCurrent = (int) armSim.getCurrentDrawAmps();
    data.MotorVoltage = armSim.getInput(0);
    data.setpoint = setpoint;
  }

  @Override
  public void setAngle(double position) {
    this.setpoint = position;
    pidController.setSetpoint(position);
  }

  @Override
  public void runVolts(double volts) {
    armSim.setInputVoltage(volts);
  }

  @Override
  public void resetEncoder() {
    pidController.setSetpoint(0.0);
  }
}
