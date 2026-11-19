package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOSim implements ElevatorIO {

  private final ElevatorSim elevator;
  private final ProfiledPIDController pid =
      new ProfiledPIDController(
          kP.getAsDouble(), 0.0, 0.0, new TrapezoidProfile.Constraints(3.8, 4));
  private final ElevatorFeedforward ff =
      new ElevatorFeedforward(
          kS.getAsDouble(), kG.getAsDouble(), kV.getAsDouble(), kA.getAsDouble());

  private double volts = 0.0;
  private double sampleTime = Timer.getFPGATimestamp();
  private double previousVelocitySample = 0.0;

  public ElevatorIOSim() {
    this.pid.setTolerance(0.01);
    this.elevator =
        new ElevatorSim(
            LinearSystemId.createElevatorSystem(gearbox, elevatorMass, drumRadius, gearing),
            gearbox,
            minHeight,
            maxHeight,
            simGravity,
            minHeight);
    this.pid.reset(0.0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    elevator.update(0.02);

    inputs.position = elevator.getPositionMeters();
    inputs.velocity = elevator.getVelocityMetersPerSecond();
    inputs.leftVolts = volts;
    inputs.rightVolts = volts;

    inputs.leftSupplyCurrent = elevator.getCurrentDrawAmps();
    inputs.rightSupplyCurrent = elevator.getCurrentDrawAmps();

    inputs.targetHeight = pid.getSetpoint().position;
    inputs.goalTarget = pid.getGoal().position;

    double currentTime = Timer.getFPGATimestamp();
    double dt = currentTime - sampleTime;
    inputs.acceleration = (elevator.getVelocityMetersPerSecond() - previousVelocitySample) / dt;

    sampleTime = currentTime;
    previousVelocitySample = elevator.getVelocityMetersPerSecond();

    Logger.recordOutput(
        "/Elevator/MaxAcceleration", ff.maxAchievableAcceleration(12, inputs.acceleration));
    Logger.recordOutput("/Elevator/MaxVelocity", ff.maxAchievableVelocity(12, inputs.velocity));

    if (kP.hasChanged(hashCode())) pid.setP(kP.getAsDouble());
    if (kI.hasChanged(hashCode())) pid.setI(kI.getAsDouble());
    if (kD.hasChanged(hashCode())) pid.setD(kD.getAsDouble());
    if (kV.hasChanged(hashCode())) ff.setKv(kV.getAsDouble());
    if (kS.hasChanged(hashCode())) ff.setKs(kS.getAsDouble());
    if (kA.hasChanged(hashCode())) ff.setKa(kA.getAsDouble());
    if (kG.hasChanged(hashCode())) ff.setKg(kG.getAsDouble());

    // Control loop
    this.setVolts(
        pid.calculate(elevator.getPositionMeters(), pid.getGoal().position)
            + ff.calculate(pid.getGoal().velocity));
  }

  @Override
  public void setControl(double position) {
    pid.setGoal(new State(position, 0.0));
  }

  @Override
  public void setVolts(double voltage) {
    this.volts = voltage;
    elevator.setInputVoltage(MathUtil.clamp(voltage, -12, 12));
  }

  @Override
  public void resetEncoder() {
    pid.setGoal(new State(0.0, 0.0));
  }

  @Override
  public double positionError() {
    return pid.getPositionError();
  }
}
