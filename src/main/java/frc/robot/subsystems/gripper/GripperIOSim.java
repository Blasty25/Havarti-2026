// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class GripperIOSim implements GripperIO {

  private DCMotorSim sim;
  private LoggedNetworkBoolean detected = new LoggedNetworkBoolean("Tuning/Gripper", false);

  public GripperIOSim() {
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.01, 5),
            DCMotor.getKrakenX60(1));
  }

  @Override
  public void updateInputs(GripperIOInputs inputs) {
    sim.update(0.02);
    inputs.StatorCurrent = sim.getCurrentDrawAmps();
    inputs.Temperature = 25;
    inputs.Velocity = sim.getAngularPositionRad();
    inputs.isConnected = true;
    inputs.isDetected = detected.get();
    inputs.Voltage = sim.getInputVoltage();
  }

  @Override
  public void setVoltage(double volts) {
    sim.setInputVoltage(MathUtil.clamp(volts, -12, 12));
  }

  @Override
  public void setDetected(boolean detected) {
    this.detected.set(detected);
  }
}
