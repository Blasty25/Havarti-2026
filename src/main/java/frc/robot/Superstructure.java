// Copyright (c) FIRST and other WPILib contributors.-
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperConstants;
import frc.robot.subsystems.gripper.GripperConstants.VoltageStates;
import frc.robot.util.FieldConstants;
import frc.robot.util.FieldConstants.BargeConstants;
import frc.robot.util.FieldConstants.DistanceCheckers;
import frc.robot.util.FieldConstants.DistanceCheckers.IntakeSource;
import java.util.EnumMap;
import java.util.Map;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

public class Superstructure extends SubsystemBase {

    public static enum State {
        Idle,
        Manual_Intake,
        L2_Intake,
        L3_Intake,
        Ground_Intake,
        Eject,
        Ready,
        Manual_Score,
        Processor_Score,
        Barge_Score,
        Lollipop_Intake
    }

    // Maps to store triggers for state requests and state-based conditions
    private Map<State, Trigger> stateRequests = new EnumMap<>(State.class);
    private Map<State, Trigger> stateTriggers = new EnumMap<>(State.class);

    // Logging the current and previous robot state
    @AutoLogOutput(key = "RobotState/CurrentState")
    private State state = State.Idle;

    @AutoLogOutput(key = "RobotState/PreviousState")
    private State previousState = State.Idle;

    // Format
    // Elevator: # Meters, Arm: # Radians
    // Elevator is always in meters and arm is always in radians no switching
    // between the two
    public static enum SuperstructureSetpoint {
        L4(ElevatorConstants.maxHeight, Units.degreesToRadians(45)),
        L3(ElevatorConstants.maxHeight, Units.degreesToRadians(120)),
        L2(0.79, Units.degreesToRadians(120)),
        L1(0.58, Units.degreesToRadians(30)),
        GI(0.0, Units.degreesToRadians(0)), // Ground Intake
        AN(ElevatorConstants.maxHeight, Units.degreesToRadians(45)),
        A3(0.83, -Units.degreesToRadians(6)),
        A2(0.45, Units.degreesToRadians(0)),
        AP(0.0, Units.degreesToRadians(0)),
        IDLE(0.0, (Math.PI / 2));

        public double height;
        public double armPosition;

        private SuperstructureSetpoint(double elevatorHeight, double armPosition) {
            this.height = elevatorHeight;
            this.armPosition = armPosition;
        }
    }

    private final Drive drive;
    private final Gripper gripper;
    private final Elevator elevator;
    private final Arm arm;

    private final CommandXboxController driver;
    private final CommandXboxController operator;

    private final LoggedMechanism2d body = new LoggedMechanism2d(1.0, 1.0);
    private final LoggedMechanismLigament2d elevatorLigament = new LoggedMechanismLigament2d("Elevator", 1.7, 90.0);
    private final LoggedMechanismLigament2d armLigament = new LoggedMechanismLigament2d(
            "Arm", ArmConstants.Mechanism.armLength, Units.degreesToRadians(0));

    @AutoLogOutput(key = "RobotState/SuperstructureSetpoint")
    private SuperstructureSetpoint currentSetpoint = SuperstructureSetpoint.IDLE;

    private Trigger scoreRequest;
    private Trigger intakeRequest;
    private Trigger ejectRequest;
    private Trigger manualIntakeRequest;
    private Trigger manualShootRequest;

    /** Creates a new Superstructure. */
    public Superstructure(
            Drive drive,
            Gripper gripper,
            Elevator elevator,
            Arm arm,
            Trigger scoreRequest,
            Trigger intakeRequest,
            Trigger ejectRequest,
            Trigger manualIntakeRequest,
            Trigger manualShootRequest,
            CommandXboxController driver,
            CommandXboxController operator) {

        // Initializing Subsystems
        this.drive = drive;
        this.gripper = gripper;
        this.elevator = elevator;
        this.arm = arm;
        this.driver = driver;
        this.operator = operator;
        this.scoreRequest = scoreRequest;
        this.intakeRequest = intakeRequest;
        this.ejectRequest = ejectRequest;
        this.manualIntakeRequest = manualIntakeRequest;
        this.manualShootRequest = manualShootRequest;

        var root = body.getRoot("Mech", 0.8, 0.0125);
        root.append(elevatorLigament);
        elevatorLigament.append(armLigament);
        armLigament.setColor(new Color8Bit(255, 0, 0));

        stateRequests.put(State.Idle, driver.povLeft());
        stateRequests.put(State.L2_Intake, operator.x());
        stateRequests.put(State.L3_Intake, operator.a());
        stateRequests.put(State.Ground_Intake, operator.leftTrigger());
        stateRequests.put(State.Manual_Intake, operator.back());
        stateRequests.put(State.Eject, driver.povUp());
        stateRequests.put(State.Barge_Score, operator.rightTrigger());
        stateRequests.put(State.Ready, operator.start());

        for (State state : State.values()) {
            stateTriggers.put(state, new Trigger(() -> this.state == state && DriverStation.isEnabled()));
        }

        this.setupIdle();
        this.setupReady();
        this.setupProcessorScore();
        this.setupReefIntake();
        this.setupGroundIntake();
        this.setupShoot();
        this.setupEject();
        this.setupManualIntake();
        this.setupManualShoot();
    }

    public void setupIdle() {

        stateTriggers.get(State.Idle).onTrue(Commands.runOnce(() -> gripper.stop()));

        stateTriggers.get(State.Idle).and(gripper::getDetected).onTrue(this.forceState(State.Ready));

        stateTriggers
                .get(State.Idle)
                .and(driver.leftTrigger())
                .and(() -> IntakeSource.Ground_Algae == DistanceCheckers.getBestIntake(drive::getPose))
                .onTrue(forceState(State.Ground_Intake));

        stateTriggers
                .get(State.Idle)
                .and(driver.leftTrigger())
                .and(() -> IntakeSource.L2 == DistanceCheckers.getBestIntake(drive::getPose))
                .onTrue(forceState(State.L2_Intake));

        stateTriggers
                .get(State.Idle)
                .and(driver.leftTrigger())
                .and(() -> IntakeSource.L3 == DistanceCheckers.getBestIntake(drive::getPose))
                .onTrue(forceState(State.L3_Intake));

        stateTriggers
                .get(State.Idle)
                .and(driver.a())
                .whileTrue(
                        Commands.parallel(
                                arm.positionCommand(() -> this.getArmAngleFromDistance()),
                                DriveCommands.joystickDriveAtAngle(
                                        drive,
                                        () -> -driver.getLeftY(),
                                        () -> -driver.getLeftX(),
                                        () -> new Rotation2d(
                                                this.getAprilTagBargePose().getX() - drive.getPose().getX(),
                                                this.getAprilTagBargePose().getY() - drive.getPose().getY()))));
    }

    public Pose2d getAprilTagBargePose() {
        return FieldConstants.fieldLayout.getTagPose(5).get().toPose2d();
    }

    public void setupReady() {

        stateTriggers.get(State.Ready).onTrue(this.extend(SuperstructureSetpoint.IDLE, elevator, arm));
        stateTriggers.get(State.Ready).onTrue(gripper.setVoltage(() -> VoltageStates.hold));

        stateTriggers
                .get(State.Ready)
                .and(driver.rightTrigger())
                .and(
                        () -> DistanceCheckers.Target.Processor == FieldConstants.DistanceCheckers
                                .getNearestScore(drive::getPose))
                .onTrue(forceState(State.Processor_Score));

        stateTriggers
                .get(State.Ready)
                .and(driver.rightTrigger())
                .and(
                        () -> DistanceCheckers.Target.Barge == FieldConstants.DistanceCheckers
                                .getNearestScore(drive::getPose))
                .onTrue(forceState(State.Barge_Score));

        stateTriggers
                .get(State.Ready)
                .and(() -> !gripper.getDetected())
                .onTrue(this.forceState(State.Idle));
    }

    public void setupProcessorScore() {
        stateTriggers
                .get(State.Processor_Score)
                .onTrue(this.extend(SuperstructureSetpoint.AP, elevator, arm));

        stateTriggers
                .get(State.Processor_Score)
                .and(scoreRequest)
                .onTrue(gripper.setVoltage(() -> GripperConstants.VoltageStates.outtake));

        stateTriggers
                .get(State.Processor_Score)
                .and(() -> !gripper.getDetected())
                .onTrue(forceState(State.Idle));
    }

    public void setupReefIntake() {

        stateTriggers
                .get(State.L2_Intake)
                .and(intakeRequest)
                .whileTrue(
                        DriveCommands.autoAlign(
                                drive,
                                () -> DistanceCheckers.getBestAlgaeIntakePose(drive::getPose, IntakeSource.L2))
                                .until(
                                        () -> FieldConstants.inTolerance(
                                                drive::getPose,
                                                () -> DistanceCheckers.getBestAlgaeIntakePose(
                                                        drive::getPose, IntakeSource.L2),
                                                0.1,
                                                0.1))
                                .andThen(this.extend(SuperstructureSetpoint.A2, elevator, arm))
                                .andThen(
                                        DriveCommands.joystickDriveAtAngle(
                                                drive,
                                                () -> -driver.getLeftY(),
                                                () -> -driver.getLeftX(),
                                                () -> DistanceCheckers.getRotationToFace(drive::getPose)
                                                        .getRotation())));
        stateTriggers
                .get(State.L3_Intake)
                .or(stateTriggers.get(State.L2_Intake))
                .or(stateTriggers.get(State.Ground_Intake))
                .and(intakeRequest)
                .onTrue(gripper.setVoltage(() -> VoltageStates.intake));

        stateTriggers
                .get(State.L3_Intake)
                .and(intakeRequest)
                .whileTrue(
                        DriveCommands.autoAlign(
                                drive,
                                () -> DistanceCheckers.getBestAlgaeIntakePose(drive::getPose, IntakeSource.L3))
                                .until(
                                        () -> FieldConstants.inTolerance(
                                                drive::getPose,
                                                () -> DistanceCheckers.getBestAlgaeIntakePose(
                                                        drive::getPose, IntakeSource.L3),
                                                0.1,
                                                0.1))
                                .andThen(this.extend(SuperstructureSetpoint.A3, elevator, arm))
                                .andThen(
                                        DriveCommands.joystickDriveAtAngle(
                                                drive,
                                                () -> -driver.getLeftY(),
                                                () -> -driver.getLeftX(),
                                                () -> DistanceCheckers.getRotationToFace(drive::getPose)
                                                        .getRotation())));

        stateTriggers
                .get(State.Ground_Intake)
                .and(intakeRequest)
                .whileTrue(
                        Commands.parallel(
                                DriveCommands.joystickDriveAtAngle(
                                        drive,
                                        () -> -driver.getLeftY(),
                                        () -> -driver.getLeftX(),
                                        () -> new Rotation2d(
                                                DistanceCheckers.getBestAlgaeIntakePose(
                                                        drive::getPose, IntakeSource.Ground_Algae)
                                                        .getX()
                                                        - drive.getPose().getX(),
                                                DistanceCheckers.getBestAlgaeIntakePose(
                                                        drive::getPose, IntakeSource.Ground_Algae)
                                                        .getY()
                                                        - drive.getPose().getY())),
                                this.extend(SuperstructureSetpoint.GI, elevator, arm)));

        stateTriggers
                .get(State.L3_Intake)
                .or(stateTriggers.get(State.L2_Intake))
                .or(stateTriggers.get(State.Ground_Intake))
                .and(gripper::getDetected)
                .onTrue(this.forceState(State.Ready));
    }

    public void setupGroundIntake() {
        stateTriggers
                .get(State.Ground_Intake)
                .and(gripper::getDetected)
                .onTrue(forceState(State.Ready));
    }

    public void setupEject() {
        ejectRequest.onTrue(
                Commands.sequence(
                        this.extend(SuperstructureSetpoint.IDLE, elevator, arm),
                        gripper.setVoltage(() -> -2.0).withTimeout(1.0),
                        forceState(State.Idle)));
    }

    public void setupShoot() {

        stateTriggers
                .get(State.Barge_Score)
                .and(() -> !gripper.getDetected())
                .onTrue(this.forceState(State.Idle));

        stateTriggers
                .get(State.Barge_Score)
                .and(scoreRequest)
                .and(() -> BargeConstants.nearNet(drive::getPose))
                .onTrue(
                        Commands.sequence(
                                this.extend(SuperstructureSetpoint.AN, elevator, arm),
                                gripper.setVoltage(() -> VoltageStates.outtake)));
    }

    public void setupManualIntake() {

        manualIntakeRequest.onTrue(this.forceState(State.Manual_Intake));

        stateTriggers
                .get(State.Manual_Intake)
                .and(driver.x())
                .onTrue((this.extend(SuperstructureSetpoint.A3, elevator, arm)));

        stateTriggers
                .get(State.Manual_Intake)
                .and(driver.b())
                .onTrue(this.extend(SuperstructureSetpoint.A2, elevator, arm));

        stateTriggers
                .get(State.Manual_Intake)
                .and(intakeRequest)
                .onTrue(gripper.setVoltage(() -> VoltageStates.intake));

        stateTriggers
                .get(State.Manual_Intake)
                .and(driver.povDown())
                .onTrue(Commands.sequence(this.forceState(State.Ready)));
    }

    public void setupManualShoot() {

        manualShootRequest.onTrue(this.forceState(State.Manual_Score));

        stateTriggers
                .get(State.Manual_Score)
                .and(driver.y())
                .onTrue(this.extend(SuperstructureSetpoint.AN, elevator, arm));

        stateTriggers
                .get(State.Manual_Score)
                .and(driver.a())
                .onTrue(this.extend(SuperstructureSetpoint.AP, elevator, arm));

        stateTriggers
                .get(State.Manual_Score)
                .and(scoreRequest)
                .onTrue(gripper.setVoltage(() -> VoltageStates.outtake));

        // stateTriggers
        // .get(State.Manual_Score)
        // .and(() -> !gripper.getDetected())
        // .onTrue(this.forceState(State.Idle));
    }

    public double getArmAngleFromDistance() {
        double distanceFromTag = drive.getPose().getTranslation()
                .getDistance(BargeConstants.bargePoses[1].getTranslation());
        double angle = (-0.172064 * distanceFromTag) + 1.5708;
        Logger.recordOutput("Arm/DistanceFromTag", distanceFromTag);
        Logger.recordOutput("Arm/ArmAngleFromTag", angle);
        return angle <= 0 ? 0.0 : angle;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (Robot.isSimulation() || !DriverStation.isFMSAttached()) {
            // Log All Necessary Superstructure Data
            elevatorLigament.setLength(elevator.getPosition());
            armLigament.setAngle(Rotation2d.fromRadians(arm.getAngle()));
            Logger.recordOutput("Superstructure/Mechanism", body);
        }
    }

    public Command extend(SuperstructureSetpoint setpoint, Elevator elevator, Arm arm) {
        return Commands.parallel(
                elevator.positionCommand(setpoint.height),
                Commands.runOnce(
                        () -> {
                            currentSetpoint = setpoint;
                        }),
                arm.positionCommand(() -> setpoint.armPosition));
    }

    public Command home(SuperstructureSetpoint setpoint, Elevator elevator, Arm arm) {
        return Commands.parallel(
                arm.positionCommand(() -> setpoint.armPosition),
                Commands.waitUntil(arm::atSetpoint),
                elevator.positionCommand(setpoint.height),
                Commands.runOnce(
                        () -> {
                            currentSetpoint = setpoint;
                        }));
    }

    public Command resetSuperstructure(Elevator elevator, Arm arm) {
        return elevator
                .resetEncoder()
                .andThen(elevator.positionCommand(0.3))
                .until(() -> elevator.atSetpoint())
                .andThen(
                        arm.positionCommand(() -> Math.PI / 2)
                                .withTimeout(1.0)
                                .andThen(elevator.homeElevator()));
    }

    private Command forceState(State nextState) {
        return Commands.runOnce(
                () -> {
                    System.out.println("Changing state to " + nextState);
                    previousState = state;
                    state = nextState;
                });
    }

    public static Command rumbleCommand(
            CommandXboxController controller, double seconds, double intensity) {
        return Commands.run(
                () -> {
                    controller.setRumble(RumbleType.kBothRumble, intensity);
                })
                .withTimeout(seconds)
                .finallyDo(
                        () -> {
                            controller.setRumble(RumbleType.kBothRumble, 0.0);
                        });
    }
}
