// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.Tuning.*;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotMap;
import frc.robot.subsystems.arm.ArmConstants.MotorLimits;

public class ArmIOTalonFX implements ArmIO {
  private final TalonFX armTalon = new TalonFX(RobotMap.Arm.pivot);
  private final VoltageOut voltage = new VoltageOut(0.0).withEnableFOC(false);
  private final MotionMagicVoltage position = new MotionMagicVoltage(Math.PI / 2).withSlot(0);

  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<AngularAcceleration> accelerationSignal;
  private final StatusSignal<Voltage> mVolts;
  private final StatusSignal<Current> stator;
  private final StatusSignal<Current> supply;
  private final StatusSignal<Temperature> temperature;

  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  private double setpoint = 0.0;

  public ArmIOTalonFX() {
    config.Audio.AllowMusicDurDisable = true;
    config.Audio.BeepOnBoot = true;
    config.Audio.BeepOnConfig = true;

    config.Feedback.SensorToMechanismRatio = ArmConstants.Mechanism.gearing;
    config.ClosedLoopGeneral.ContinuousWrap = true;
    config.MotorOutput.Inverted =
        ArmConstants.Mechanism.inverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    config.Slot0.kP = kP.get();
    config.Slot0.kI = kI.get();
    config.Slot0.kD = kD.get();
    config.Slot0.kS = kS.get();
    config.Slot0.kV = kV.get();
    config.Slot0.kA = kA.get();
    config.Slot0.kG = kG.get();

    config.CurrentLimits.StatorCurrentLimit = MotorLimits.maxStator;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = MotorLimits.maxSupply;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotionMagic.MotionMagicAcceleration = MotorLimits.maxAcceleration;
    config.MotionMagic.MotionMagicCruiseVelocity = MotorLimits.maxVelocity;

    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;

    tryUntilOk(5, () -> armTalon.getConfigurator().apply(config, 0.25));

    positionSignal = armTalon.getPosition();
    velocitySignal = armTalon.getVelocity();
    accelerationSignal = armTalon.getAcceleration();

    mVolts = armTalon.getMotorVoltage();
    stator = armTalon.getTorqueCurrent();
    supply = armTalon.getSupplyCurrent();
    temperature = armTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        positionSignal,
        velocitySignal,
        accelerationSignal,
        mVolts,
        stator,
        supply,
        temperature);
    armTalon.optimizeBusUtilization();

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

    BaseStatusSignal.refreshAll(
        positionSignal, velocitySignal, accelerationSignal, mVolts, stator, supply, temperature);

    data.ArmAcceleration = accelerationSignal.getValueAsDouble();
    data.ArmVelocity = velocitySignal.getValueAsDouble();
    data.ArmAngle =
        Units.rotationsToRadians(positionSignal.getValueAsDouble());
    data.MotorVoltage = mVolts.getValueAsDouble();
    data.MotorTemperature = temperature.getValueAsDouble();
    data.MotorStatorCurrent = stator.getValueAsDouble();
    data.MotorSupplyCurrent = supply.getValueAsDouble();
    data.setpoint = this.setpoint;

    data.MotorConnected =
        connectedDebouncer.calculate(
            BaseStatusSignal.isAllGood(
                positionSignal,
                velocitySignal,
                accelerationSignal,
                mVolts,
                stator,
                supply,
                temperature));
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
    armTalon.setControl(position.withPosition(Units.radiansToRotations(rads)));
  }

  @Override
  public void runVolts(double volts) {
    armTalon.setControl(voltage.withOutput(volts));
  }

  @Override
  public void resetEncoder() {
    armTalon.setPosition(Math.PI / 2);
  }
}
