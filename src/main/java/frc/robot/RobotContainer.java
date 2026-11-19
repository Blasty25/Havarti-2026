package frc.robot;

import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Superstructure.SuperstructureSetpoint;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmIOSparkMax;
import frc.robot.subsystems.arm.ArmIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOSparkMax;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperIO;
import frc.robot.subsystems.gripper.GripperIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.vision.VisionIOSouthStar;
import frc.robot.subsystems.vision.VisionIOSouthStarSim;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.AutoRoutines;
import frc.robot.util.FieldConstants;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public final Drive drive;
  private final Elevator elevator;
  private final Vision vision;
  private final Arm arm;
  private final Gripper gripper;
//   private final Superstructure superstructure;
  private final AutoRoutines auto;

  // Controller
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                FieldConstants.fieldLayout,
                new VisionIOPhotonVision(
                    "CamLeft",
                    VisionConstants.robotToLeftCam,
                    drive::getRotation,
                    FieldConstants.fieldLayout),
                new VisionIOPhotonVision(
                    "CamRight",
                    VisionConstants.robotToRightCam,
                    drive::getRotation,
                    FieldConstants.fieldLayout),
                new VisionIOSouthStar(drive::getPose, VisionConstants.camProps));
        gripper = new Gripper(new GripperIOSim());
        elevator = new Elevator(new ElevatorIOSparkMax());
        arm = new Arm(new ArmIOSparkMax());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                FieldConstants.fieldLayout,
                new VisionIOPhotonVisionSim(
                    "Left Cam",
                    VisionConstants.robotToLeftCam,
                    drive::getPose,
                    VisionConstants.fieldLayout),
                new VisionIOPhotonVisionSim(
                    "Right Cam",
                    VisionConstants.robotToRightCam,
                    drive::getPose,
                    VisionConstants.fieldLayout),
                new VisionIOSouthStarSim(drive::getPose, VisionConstants.camProps));
        arm = new Arm(new ArmIOSim());
        elevator = new Elevator(new ElevatorIOSim());
        gripper = new Gripper(new GripperIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        vision =
            new Vision(
                drive::addVisionMeasurement,
                FieldConstants.fieldLayout,
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {});
        arm = new Arm(new ArmIO() {});
        gripper = new Gripper(new GripperIO() {});
        break;
    }

    // superstructure =
    //     new Superstructure(
    //         drive,
    //         gripper,
    //         elevator,
    //         arm,
    //         driver.rightTrigger(),
    //         driver.leftTrigger(),
    //         driver.povLeft(),
    //         driver.back(),
    //         driver.start(),
    //         driver,
    //         operator);
    auto = new AutoRoutines();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    // Set up SysId routines
    autoChooser.addDefaultOption("elevator sys", elevator.sysId());
    autoChooser.addOption("arm sys", arm.sysID());

    // Autos Autos
    autoChooser.addOption("Match Auto Drive Fowards", auto.driveForward(drive));
    // autoChooser.addOption("Auto", auto.oneAlgaeAuto(drive, superstructure,
    // elevator, arm,
    // gripper));
    // Warm up pathfinding
    // PathfindingCommand.warmupCommand().schedule();

    // Configure the button bindingsw
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    // drive.setDefaultCommand(
    //     DriveCommands.joystickDrive(
    //         drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));

    // Don't run this if robot is enabled for now!
    // driver
    //     .y()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     drive.setPose(
    //                         new Pose2d(
    //                             drive.getPose().getTranslation(),
    //                             AllianceFlipUtil.apply(Rotation2d.kZero))),
    //                 drive)
    //             .ignoringDisable(true));

    driver.y().whileTrue(
        Commands.runOnce(()-> elevator.setPosition(SuperstructureSetpoint.L4.height))
    );

    driver.x().whileTrue(
        Commands.runOnce(()-> elevator.setPosition(SuperstructureSetpoint.L3.height))
    );
    driver.b().whileTrue(
        Commands.runOnce(()-> elevator.setPosition(SuperstructureSetpoint.L2.height))
    );
    driver.a().whileTrue(
        Commands.runOnce(()-> elevator.setPosition(SuperstructureSetpoint.L1.height))
    );

    driver.povRight().whileTrue(Commands.runOnce(()-> arm.positionCommand(()-> 0.0)));
    // driver
    //     .a()
    //     .whileTrue(
    //         DriveCommands.autoAlign(
    //             drive,
    //             () -> DistanceCheckers.getBestAlgaeIntakePose(drive::getPose, IntakeSource.L2)));

    // Tuning Superstructure NOT TO BE ENABLED DURING COMP
    // TODO: Use this in telop dont use auto sys id
    // operator.a().whileTrue(elevator.sysId());
    operator.b().onTrue(arm.sysID());

    // In Auto this command will run to reset superstructure for comp!
    // operator.povDown().onTrue(superstructure.resetSuperstructure(elevator, arm));
  }

  public Command resetGyro() {
    return Commands.runOnce(
        () ->
            drive.setPose(
                new Pose2d(
                    drive.getPose().getTranslation(), AllianceFlipUtil.apply(Rotation2d.kZero))),
        drive);
  }

//   public Command getAutonomousCommand() {
//     return auto.autoTesting(drive, superstructure, elevator, arm);
//   }

//   public Command getResetCommand() {
//     return superstructure.resetSuperstructure(elevator, arm);
//   }
}
