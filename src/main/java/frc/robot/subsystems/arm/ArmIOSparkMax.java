// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.Tuning.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.RobotMap;

public class ArmIOSparkMax implements ArmIO {
  private final TalonFX armTalon = new TalonFX(RobotMap.Arm.pivot);

  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  private double setpoint = 0.0;

  public ArmIOSparkMax() {
    
    armTalon.setPosition(0.25);
  }

  @Override
  public void getData(ArmData data) {

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
    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.withKP(kP.getAsDouble());
    slot0Configs.withKI(kI.getAsDouble());
    slot0Configs.withKD(kD.getAsDouble());
    slot0Configs.withKV(kV.getAsDouble());
    slot0Configs.withKA(kA.getAsDouble());
    slot0Configs.withKG(kG.getAsDouble());
    slot0Configs.withKS(kS.getAsDouble());
    armTalon.getConfigurator().apply(slot0Configs, 0.25);
  }

  @Override
  public void setAngle(double rads) {
    this.setpoint = rads;
  }

  @Override
  public void runVolts(double volts) {
  }

  @Override
  public void resetEncoder() {
    armTalon.setPosition(Math.PI / 2);
  }
}
