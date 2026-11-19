// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Superstructure;
import frc.robot.Superstructure.SuperstructureSetpoint;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperConstants.VoltageStates;
import frc.robot.util.FieldConstants.AutoConstants;
import frc.robot.util.FieldConstants.BargeConstants;
import frc.robot.util.FieldConstants.DistanceCheckers;
import frc.robot.util.FieldConstants.DistanceCheckers.IntakeSource;
import java.util.ArrayList;
import java.util.List;

public class AutoRoutines extends SubsystemBase {

  private Pose2d target;

  public Command autoTesting(
      Drive drive, Superstructure superstructure, Elevator elevator, Arm arm) {
    PathConstraints constraints =
        new PathConstraints(
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
            Math.pow(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond), 2),
            drive.getMaxAngularSpeedRadPerSec(),
            Math.pow(drive.getMaxAngularSpeedRadPerSec(), 2));

    Command path1 =
        AutoBuilder.pathfindToPose(
            DistanceCheckers.getBestAlgaeIntakePose(drive::getPose, IntakeSource.L2), constraints);
    Command path2 = AutoBuilder.pathfindToPose(AutoConstants.aprilTags[0], constraints);
    Pose2d firstTarget = DistanceCheckers.getBestAlgaeIntakePose(drive::getPose, IntakeSource.L2);

    return path1
        .andThen(DriveCommands.autoAlign(drive, () -> firstTarget))
        .until(() -> DriveCommands.isNear(firstTarget, drive.getPose()))
        .andThen(superstructure.extend(SuperstructureSetpoint.A3, elevator, arm))
        .andThen(Commands.waitSeconds(2.0))
        .andThen(superstructure.extend(SuperstructureSetpoint.IDLE, elevator, arm))
        .withTimeout(3.0)
        .andThen(path2)
        .andThen(DriveCommands.autoAlign(drive, () -> AutoConstants.aprilTags[0]));
  }

  public Command driveForward(Drive drive) {
    target =
        drive
            .getPose()
            .transformBy(
                new Transform2d(
                    DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red)
                        ? 1.0
                        : -1.0,
                    0.0,
                    AllianceFlipUtil.apply(Rotation2d.kPi)));

    return DriveCommands.autoAlign(drive, () -> target)
        .until(() -> DriveCommands.isNear(target, drive.getPose()));
  }

  // Ts will work trust chat
  public Command oneAlgaeAuto(
      Drive drive, Superstructure superstructure, Elevator elevator, Arm arm, Gripper gripper) {

    PathConstraints constraints =
        new PathConstraints(
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
            Math.pow(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond), 2),
            drive.getMaxAngularSpeedRadPerSec(),
            Math.pow(drive.getMaxAngularSpeedRadPerSec(), 2));

    Command path1 =
        Commands.sequence(
            AutoBuilder.pathfindToPose(AutoConstants.offsetedPose[0], constraints, 0.0),
            Commands.waitSeconds(1),
            superstructure.extend(SuperstructureSetpoint.A2, elevator, arm).withTimeout(7.0));

    Command path2 =
        Commands.sequence(
            AutoBuilder.pathfindToPose(BargeConstants.bargePoses[1], constraints, 0.0),
            Commands.waitSeconds(1),
            superstructure.extend(SuperstructureSetpoint.AN, elevator, arm).withTimeout(5.0),
            Commands.runOnce(() -> gripper.manualPower(() -> VoltageStates.outtake)),
            Commands.waitSeconds(1.0),
            Commands.runOnce(() -> gripper.stop()));

    Command algaeA2 =
        Commands.sequence(
            Commands.parallel(
                DriveCommands.autoAlign(drive, () -> AutoConstants.offsetedPose[0])
                    .withTimeout(5.0),
                Commands.runOnce(() -> gripper.manualPower(() -> VoltageStates.intake))));

    return Commands.sequence(path1, algaeA2, path2);
  }

  // Anthony why :sob:
  public Command dynamicAlgaeAuto(
      Drive drive, Superstructure superstructure, Elevator elevator, Arm arm, Gripper gripper) {

    Pose2d startingPose = drive.getPose();
    PathConstraints constraints =
        new PathConstraints(
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
            Math.pow(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond), 2),
            drive.getMaxAngularSpeedRadPerSec(),
            Math.pow(drive.getMaxAngularSpeedRadPerSec(), 2));

    List<Command> toReef = new ArrayList<>();
    List<Command> algaeIntake = new ArrayList<>();
    List<Command> toNet = new ArrayList<>();

    if (startingPose.getY() < 3.0) {
      for (int i = 5; i > 0; i--) {
        final int idx = i;

        algaeIntake.add(
            Commands.sequence(
                Commands.parallel(
                    DriveCommands.autoAlign(drive, () -> AutoConstants.offsetedPose[idx])
                        .withTimeout(5.0),
                    Commands.runOnce(() -> gripper.manualPower(() -> VoltageStates.intake)))));
        toNet.add(
            Commands.sequence(
                Commands.parallel(
                    AutoBuilder.pathfindToPose(BargeConstants.bargePoses[1], constraints, 0.0),
                    Commands.sequence(
                        Commands.waitSeconds(1),
                        superstructure
                            .extend(SuperstructureSetpoint.AN, elevator, arm)
                            .withTimeout(7.0))),
                Commands.runOnce(() -> gripper.manualPower(() -> VoltageStates.outtake)),
                Commands.waitSeconds(1.0),
                Commands.runOnce(() -> gripper.stop())));
        if (AutoConstants.offsetedPose[idx] == AutoConstants.offsetedPose[0]
            || AutoConstants.offsetedPose[idx] == AutoConstants.offsetedPose[2]
            || AutoConstants.offsetedPose[idx] == AutoConstants.offsetedPose[4]) {
          toReef.add(
              Commands.parallel(
                  AutoBuilder.pathfindToPose(AutoConstants.offsetedPose[idx], constraints, 0.0),
                  Commands.sequence(
                      Commands.waitSeconds(1),
                      superstructure
                          .extend(SuperstructureSetpoint.A2, elevator, arm)
                          .withTimeout(7.0))));
        } else {
          toReef.add(
              Commands.parallel(
                  AutoBuilder.pathfindToPose(AutoConstants.offsetedPose[idx], constraints, 0.0),
                  Commands.sequence(
                      Commands.waitSeconds(1),
                      superstructure
                          .extend(SuperstructureSetpoint.A3, elevator, arm)
                          .withTimeout(7.0))));
        }
      }
      return Commands.sequence(
          toReef.get(0),
          algaeIntake.get(0),
          toNet.get(0),
          toReef.get(1),
          algaeIntake.get(1),
          toNet.get(1),
          toReef.get(2),
          algaeIntake.get(2),
          toNet.get(2),
          toReef.get(3),
          algaeIntake.get(3),
          toNet.get(3));
    } else if (startingPose.getY() >= 3.0 && startingPose.getY() < 5.0) {
      for (int i = 0; i < AutoConstants.offsetedPose.length; i++) {
        final int idx = i;

        algaeIntake.add(
            Commands.sequence(
                Commands.parallel(
                    DriveCommands.autoAlign(drive, () -> AutoConstants.offsetedPose[idx])
                        .withTimeout(5.0),
                    Commands.runOnce(() -> gripper.manualPower(() -> VoltageStates.intake)))));
        toNet.add(
            Commands.sequence(
                Commands.parallel(
                    AutoBuilder.pathfindToPose(BargeConstants.bargePoses[1], constraints, 0.0),
                    Commands.sequence(
                        Commands.waitSeconds(1),
                        superstructure
                            .extend(SuperstructureSetpoint.AN, elevator, arm)
                            .withTimeout(7.0))),
                Commands.runOnce(() -> gripper.manualPower(() -> VoltageStates.outtake)),
                Commands.waitSeconds(1.0),
                Commands.runOnce(() -> gripper.stop())));
        if (AutoConstants.offsetedPose[idx] == AutoConstants.offsetedPose[0]
            || AutoConstants.offsetedPose[idx] == AutoConstants.offsetedPose[2]
            || AutoConstants.offsetedPose[idx] == AutoConstants.offsetedPose[4]) {
          toReef.add(
              Commands.parallel(
                  AutoBuilder.pathfindToPose(AutoConstants.offsetedPose[idx], constraints, 0.0),
                  Commands.sequence(
                      Commands.waitSeconds(1),
                      superstructure
                          .extend(SuperstructureSetpoint.A2, elevator, arm)
                          .withTimeout(7.0))));
        } else {
          toReef.add(
              Commands.parallel(
                  AutoBuilder.pathfindToPose(AutoConstants.offsetedPose[idx], constraints, 0.0),
                  Commands.sequence(
                      Commands.waitSeconds(1),
                      superstructure
                          .extend(SuperstructureSetpoint.A3, elevator, arm)
                          .withTimeout(7.0))));
        }
      }
      return Commands.sequence(
          toReef.get(0),
          algaeIntake.get(0),
          toNet.get(0),
          toReef.get(1),
          algaeIntake.get(1),
          toNet.get(1),
          toReef.get(2),
          algaeIntake.get(2),
          toNet.get(2),
          toReef.get(3),
          algaeIntake.get(3),
          toNet.get(3));
    } else {
      for (int i = 1; i < 5; i++) {
        final int idx = i;

        algaeIntake.add(
            Commands.sequence(
                Commands.parallel(
                    DriveCommands.autoAlign(drive, () -> AutoConstants.offsetedPose[idx])
                        .withTimeout(5.0),
                    Commands.runOnce(() -> gripper.manualPower(() -> VoltageStates.intake)))));
        toNet.add(
            Commands.sequence(
                Commands.parallel(
                    AutoBuilder.pathfindToPose(BargeConstants.bargePoses[1], constraints, 0.0),
                    Commands.sequence(
                        Commands.waitSeconds(1),
                        superstructure
                            .extend(SuperstructureSetpoint.AN, elevator, arm)
                            .withTimeout(7.0))),
                Commands.runOnce(() -> gripper.manualPower(() -> VoltageStates.outtake)),
                Commands.waitSeconds(1.0),
                Commands.runOnce(() -> gripper.stop())));
        if (AutoConstants.offsetedPose[idx] == AutoConstants.offsetedPose[0]
            || AutoConstants.offsetedPose[idx] == AutoConstants.offsetedPose[2]
            || AutoConstants.offsetedPose[idx] == AutoConstants.offsetedPose[4]) {
          toReef.add(
              Commands.parallel(
                  AutoBuilder.pathfindToPose(AutoConstants.offsetedPose[idx], constraints, 0.0),
                  Commands.sequence(
                      Commands.waitSeconds(1),
                      superstructure
                          .extend(SuperstructureSetpoint.A2, elevator, arm)
                          .withTimeout(7.0))));
        } else {
          toReef.add(
              Commands.parallel(
                  AutoBuilder.pathfindToPose(AutoConstants.offsetedPose[idx], constraints, 0.0),
                  Commands.sequence(
                      Commands.waitSeconds(1),
                      superstructure
                          .extend(SuperstructureSetpoint.A3, elevator, arm)
                          .withTimeout(7.0))));
        }
      }
      return Commands.sequence(
          toReef.get(0),
          algaeIntake.get(0),
          toNet.get(0),
          toReef.get(1),
          algaeIntake.get(1),
          toNet.get(1),
          toReef.get(2),
          algaeIntake.get(2),
          toNet.get(2),
          toReef.get(3),
          algaeIntake.get(3),
          toNet.get(3));
    }
  }

  @Override
  public void periodic() {}
}
