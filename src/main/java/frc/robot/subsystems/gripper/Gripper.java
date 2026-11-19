// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.gripper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Gripper extends SubsystemBase {

  private GripperIOInputsAutoLogged inputs = new GripperIOInputsAutoLogged();
  // The Wheely thing at the end of the arm
  private final GripperIO io;

  // The Wheely thing at the end of the arm
  public Gripper(GripperIO io) {
    this.io = io;
  }

  public Command setVoltage(DoubleSupplier voltage) {
    return Commands.run(
        () -> {
          io.setVoltage(voltage.getAsDouble());
        },
        this);

    // We might have to get rid of the hold here, but idk rn keep it just in case?
  }

  public void manualPower(DoubleSupplier power) {
    System.out.println(power.getAsDouble());
    io.setVoltage(power.getAsDouble());
  }

  public void stop() {
    io.setVoltage(0.0);
  }

  public boolean getDetected() {
    return inputs.isDetected;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Gripper", inputs);
  }
}
