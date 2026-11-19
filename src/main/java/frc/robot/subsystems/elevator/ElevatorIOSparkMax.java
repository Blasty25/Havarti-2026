// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;
import static frc.robot.util.SparkUtil.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotMap;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOSparkMax implements ElevatorIO {

  private SparkMax leftMax;
  private SparkMax rightMax;

  private ElevatorFeedforward ff =
      new ElevatorFeedforward(
          ElevatorConstants.kS.getAsDouble(),
          ElevatorConstants.kG.getAsDouble(),
          ElevatorConstants.kV.getAsDouble());

  private SparkClosedLoopController controller;

  private SparkMaxConfig config;

  private double setpoint = 0.0;
  private double prevVelocitySample = 0.0;
  private double prevTimeSample = Timer.getFPGATimestamp();
  private double currentAcceleration = 0.0;

  private RelativeEncoder encoder;

  public ElevatorIOSparkMax() {

    this.leftMax = new SparkMax(RobotMap.Elevator.leftID, MotorType.kBrushless);
    this.rightMax = new SparkMax(RobotMap.Elevator.rightID, MotorType.kBrushless);

    this.config = new SparkMaxConfig();

    config.idleMode(IdleMode.kCoast).inverted(false).voltageCompensation(12).smartCurrentLimit(95);

    config
        .encoder
        .positionConversionFactor(ElevatorConstants.positionConversionFactor)
        .velocityConversionFactor(ElevatorConstants.positionConversionFactor / 60);

    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    config.closedLoop.pid(
        ElevatorConstants.kP.getAsDouble(),
        ElevatorConstants.kI.getAsDouble(),
        ElevatorConstants.kD.getAsDouble());

    leftMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    config.follow(leftMax, true);

    rightMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    this.controller = leftMax.getClosedLoopController();
    this.encoder = leftMax.getEncoder();

    this.encoder.setPosition(0.0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.leftConnected = true;
    inputs.rightConnected = true;

    inputs.leftSupplyCurrent = leftMax.getOutputCurrent();
    inputs.leftVolts = leftMax.getBusVoltage() * leftMax.getAppliedOutput();

    inputs.rightSupplyCurrent = rightMax.getOutputCurrent();
    inputs.rightVolts = rightMax.getBusVoltage() * rightMax.getAppliedOutput();

    inputs.position = encoder.getPosition();
    inputs.velocity = encoder.getVelocity();

    double currentTime = Timer.getFPGATimestamp();

    double dt = currentTime - prevTimeSample;
    inputs.acceleration = (encoder.getVelocity() - prevVelocitySample) / dt;
    currentAcceleration = inputs.acceleration;

    prevVelocitySample = encoder.getVelocity();
    prevTimeSample = currentTime;

    inputs.targetHeight = setpoint;

    Logger.recordOutput(
        "/Elevator/MaxAcceleration", ff.maxAchievableAcceleration(12, inputs.acceleration));
    Logger.recordOutput("/Elevator/MaxVelocity", ff.maxAchievableVelocity(12, inputs.velocity));

    if (kP.hasChanged(hashCode())) {
      resetValues();
    }

    if (kI.hasChanged(hashCode())) {
      resetValues();
    }

    if (kD.hasChanged(hashCode())) {
      resetValues();
    }

    if (kV.hasChanged(hashCode())) {
      resetValues();
    }

    if (kS.hasChanged(hashCode())) {
      resetValues();
    }

    if (kA.hasChanged(hashCode())) {
      resetValues();
    }

    if (kG.hasChanged(hashCode())) {
      resetValues();
    }
  }

  @Override
  public void setControl(double position) {
    this.setpoint = position;
    controller.setReference(
        position,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        ff.calculate(0.0),
        ArbFFUnits.kVoltage);
  }

  @Override
  public void setVolts(double volts) {
    leftMax.setVoltage(volts);
  }

  public void resetValues() {
    config.closedLoop.pid(kP.getAsDouble(), kI.getAsDouble(), kD.getAsDouble());
    config.disableFollowerMode();
    tryUntilOk(
        leftMax,
        5,
        () ->
            leftMax.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void resetEncoder() {
    encoder.setPosition(0.0);
  }
}
