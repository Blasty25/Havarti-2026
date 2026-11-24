// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.Tuning.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotMap;

public class ArmIOSparkMax implements ArmIO {
  private final TalonFX armTalon = new TalonFX(RobotMap.Arm.pivot);
  private final SparkMax sparkMax = new SparkMax(RobotMap.Arm.pivot, MotorType.kBrushless);
  private final SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
  private SparkClosedLoopController controller = null;
  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  private double setpoint = 0.0;

  public ArmIOSparkMax() {
    sparkMaxConfig.closedLoop.feedForward.svag(ArmConstants.Tuning.kS.get(), ArmConstants.Tuning.kV.get(),
        ArmConstants.Tuning.kA.get(), 0.0);
    sparkMaxConfig.closedLoop.pid(ArmConstants.Tuning.kP.get(), ArmConstants.Tuning.kI.get(),
        ArmConstants.Tuning.kD.get());
    sparkMaxConfig.closedLoop.positionWrappingEnabled(true);
    sparkMaxConfig.closedLoop.positionWrappingInputRange(-0.5, 0.5);
    sparkMaxConfig.closedLoop.feedForward.kCos(ArmConstants.Tuning.kG.get());
    sparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    sparkMaxConfig.idleMode(IdleMode.kBrake);
    sparkMaxConfig.encoder.positionConversionFactor(ArmConstants.Mechanism.gearing);
    sparkMaxConfig.encoder.velocityConversionFactor(ArmConstants.Mechanism.gearing / 60.0);
    this.controller = sparkMax.getClosedLoopController();
    sparkMax.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    sparkMax.getEncoder().setPosition(0.25);

  }

  @Override
  public void getData(ArmData data) {

    data.ArmAngle = sparkMax.getEncoder().getPosition();
    data.MotorVoltage = sparkMax.getBusVoltage() * sparkMax.getAppliedOutput();
    data.MotorSupplyCurrent = sparkMax.getOutputCurrent();
    data.setpoint = setpoint;
    

    // Resetting PID & FF Values
    if (kP.hasChanged(hashCode())) {
      this.resetValues();
    }

    if (kI.hasChanged(hashCode())) {
      this.resetValues();
    }

    if (kD.hasChanged(hashCode())) {
      this.resetValues();
    }

    if (kV.hasChanged(hashCode())) {
      this.resetValues();
    }

    if (kS.hasChanged(hashCode())) {
      this.resetValues();
    }

    if (kA.hasChanged(hashCode())) {
      this.resetValues();
    }

    if (kG.hasChanged(hashCode())) {
      this.resetValues();
    }

  }

  private void resetValues() {
    sparkMaxConfig.closedLoop.pid(kP.get(), kI.getAsDouble(), kD.get()).feedForward.svag(kS.getAsDouble(), kV.get(),
        kA.get(), kG.get());
    sparkMax.configure(
        sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  

  @Override
  public void setAngle(double rads) {
    this.setpoint = rads;
    controller.setSetpoint(rads, ControlType.kPosition);
  }

  @Override
  public void runVolts(double volts) {
  }

  @Override
  public void resetEncoder() {
    armTalon.setPosition(Math.PI / 2);
  }
}
