package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.flywheel.FlywheelVoltageCommand;
import frc.robot.commands.position_joint.PositionJointPositionCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Module;
import frc.robot.subsystems.drive.azimuth_motor.AzimuthMotorConstants;
import frc.robot.subsystems.drive.azimuth_motor.AzimuthMotorIOReplay;
import frc.robot.subsystems.drive.azimuth_motor.AzimuthMotorIOSim;
import frc.robot.subsystems.drive.azimuth_motor.AzimuthMotorIOSparkMax;
import frc.robot.subsystems.drive.drive_motor.DriveMotorConstants;
import frc.robot.subsystems.drive.drive_motor.DriveMotorIOReplay;
import frc.robot.subsystems.drive.drive_motor.DriveMotorIOSim;
import frc.robot.subsystems.drive.drive_motor.DriveMotorIOSparkMax;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOPigeon2;
import frc.robot.subsystems.drive.odometry_threads.PhoenixOdometryThread;
import frc.robot.subsystems.drive.odometry_threads.SparkOdometryThread;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelConstants;
import frc.robot.subsystems.flywheel.FlywheelIOReplay;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOSparkMax;
import frc.robot.subsystems.position_joint.PositionJoint;
import frc.robot.subsystems.position_joint.PositionJointConstants;
import frc.robot.subsystems.position_joint.PositionJointIOReplay;
import frc.robot.subsystems.position_joint.PositionJointIOSim;
import frc.robot.subsystems.position_joint.PositionJointIOSparkMax;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.vision.VisionIOPhotonVisionTrig;
import frc.robot.subsystems.vision.VisionIOQuestNav;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private PositionJoint rightCoralRotationMotor;
  private Flywheel rightCoralRollerMotor;

  @SuppressWarnings("unused")
  private final Vision vision;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(13, ""),
                new Module(
                    new DriveMotorIOSparkMax(
                        "FrontLeftDrive", DriveMotorConstants.FRONT_LEFT_CONFIG),
                    DriveMotorConstants.DRIVE_GAINS,
                    new AzimuthMotorIOSparkMax(
                        "FrontLeftAz", AzimuthMotorConstants.FRONT_LEFT_CONFIG),
                    AzimuthMotorConstants.Azimuth_GAINS),
                new Module(
                    new DriveMotorIOSparkMax(
                        "FrontRightDrive", DriveMotorConstants.FRONT_RIGHT_CONFIG),
                    DriveMotorConstants.DRIVE_GAINS,
                    new AzimuthMotorIOSparkMax(
                        "FrontRightAz", AzimuthMotorConstants.FRONT_RIGHT_CONFIG),
                    AzimuthMotorConstants.Azimuth_GAINS),
                new Module(
                    new DriveMotorIOSparkMax("BackLeftDrive", DriveMotorConstants.BACK_LEFT_CONFIG),
                    DriveMotorConstants.DRIVE_GAINS,
                    new AzimuthMotorIOSparkMax(
                        "BackLeftAz", AzimuthMotorConstants.BACK_LEFT_CONFIG),
                    AzimuthMotorConstants.Azimuth_GAINS),
                new Module(
                    new DriveMotorIOSparkMax(
                        "BackRightDrive", DriveMotorConstants.BACK_RIGHT_CONFIG),
                    DriveMotorConstants.DRIVE_GAINS,
                    new AzimuthMotorIOSparkMax(
                        "BackRightAz", AzimuthMotorConstants.BACK_RIGHT_CONFIG),
                    AzimuthMotorConstants.Azimuth_GAINS),
                PhoenixOdometryThread.getInstance(),
                SparkOdometryThread.getInstance());

        VisionIOQuestNav questNav =
            new VisionIOQuestNav(
                VisionConstants.robotToCamera0,
                new VisionIOPhotonVisionTrig(
                    "USB_Camera", VisionConstants.robotToCamera1, drive::getRotation));
        driverController.y().onTrue(Commands.runOnce(questNav::resetPose).ignoringDisable(true));
        // Reset gyro to 0° when B button is pressed
        driverController.b().onTrue(Commands.runOnce(questNav::resetHeading).ignoringDisable(true));

        rightCoralRotationMotor =
            new PositionJoint(
                new PositionJointIOSparkMax(
                    "RightCoralRotateMotor",
                    PositionJointConstants.RIGHT_CORAL_INTAKE_RROTATION_CONFIG),
                PositionJointConstants.CORAL_INTAKE_ROTATION_GAINS);

        rightCoralRollerMotor =
            new Flywheel(
                new FlywheelIOSparkMax(
                    "RightCoralRollerMotor",
                    FlywheelConstants.RIGHT_CORAL_INTAKE_ROLLERS_CONFG,
                    true),
                FlywheelConstants.RIGHT_CORAL_INTAKE_ROLLER_GAINS);

        vision = new Vision(drive::addVisionMeasurement, questNav);
        break;

      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new Module(
                    new DriveMotorIOSim("FrontLeftDrive", DriveMotorConstants.FRONT_LEFT_CONFIG),
                    DriveMotorConstants.DRIVE_GAINS_SIM,
                    new AzimuthMotorIOSim("FrontLeftAz", AzimuthMotorConstants.FRONT_LEFT_CONFIG),
                    AzimuthMotorConstants.Azimuth_GAINS_SIM),
                new Module(
                    new DriveMotorIOSim("FrontRightDrive", DriveMotorConstants.FRONT_RIGHT_CONFIG),
                    DriveMotorConstants.DRIVE_GAINS_SIM,
                    new AzimuthMotorIOSim("FrontRightAz", AzimuthMotorConstants.FRONT_RIGHT_CONFIG),
                    AzimuthMotorConstants.Azimuth_GAINS_SIM),
                new Module(
                    new DriveMotorIOSim("BackLeftDrive", DriveMotorConstants.BACK_LEFT_CONFIG),
                    DriveMotorConstants.DRIVE_GAINS_SIM,
                    new AzimuthMotorIOSim("BackLeftAz", AzimuthMotorConstants.BACK_LEFT_CONFIG),
                    AzimuthMotorConstants.Azimuth_GAINS_SIM),
                new Module(
                    new DriveMotorIOSim("BackRightDrive", DriveMotorConstants.BACK_RIGHT_CONFIG),
                    DriveMotorConstants.DRIVE_GAINS_SIM,
                    new AzimuthMotorIOSim("BackRightAz", AzimuthMotorConstants.BACK_RIGHT_CONFIG),
                    AzimuthMotorConstants.Azimuth_GAINS_SIM),
                null,
                null);

        rightCoralRotationMotor =
            new PositionJoint(
                new PositionJointIOSim(
                    "RightCoralRotateMotor",
                    PositionJointConstants.RIGHT_CORAL_INTAKE_RROTATION_CONFIG),
                PositionJointConstants.CORAL_INTAKE_ROTATION_GAINS);
        rightCoralRollerMotor =
            new Flywheel(
                new FlywheelIOSim(
                    "RightCoralRollerMotor", FlywheelConstants.RIGHT_CORAL_INTAKE_ROLLERS_CONFG),
                FlywheelConstants.RIGHT_CORAL_INTAKE_ROLLER_GAINS);

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new Module(
                    new DriveMotorIOReplay("FrontLeftDrive"),
                    DriveMotorConstants.DRIVE_GAINS,
                    new AzimuthMotorIOReplay("FrontLeftAz"),
                    AzimuthMotorConstants.Azimuth_GAINS),
                new Module(
                    new DriveMotorIOReplay("FrontRightDrive"),
                    DriveMotorConstants.DRIVE_GAINS,
                    new AzimuthMotorIOReplay("FrontRightAz"),
                    AzimuthMotorConstants.Azimuth_GAINS),
                new Module(
                    new DriveMotorIOReplay("BackLeftDrive"),
                    DriveMotorConstants.DRIVE_GAINS,
                    new AzimuthMotorIOReplay("BackLeftAz"),
                    AzimuthMotorConstants.Azimuth_GAINS),
                new Module(
                    new DriveMotorIOReplay("BackRightDrive"),
                    DriveMotorConstants.DRIVE_GAINS,
                    new AzimuthMotorIOReplay("BackRightAz"),
                    AzimuthMotorConstants.Azimuth_GAINS),
                null,
                null);

        rightCoralRotationMotor =
            new PositionJoint(
                new PositionJointIOReplay("RightCoralRotateMotor"),
                PositionJointConstants.CORAL_INTAKE_ROTATION_GAINS);

        rightCoralRollerMotor =
            new Flywheel(
                new FlywheelIOReplay("RightCoralRollerMotor"),
                FlywheelConstants.RIGHT_CORAL_INTAKE_ROLLER_GAINS);

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /**************************************************************
     * Drivetrain Controls *
     **************************************************************/
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> driverController.getLeftY(),
            () -> driverController.getLeftX(),
            () -> -driverController.getRightX()));

    // Lock to 0° when A button is held
    driverController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> driverController.getLeftY(),
                () -> driverController.getLeftX(),
                () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // // Reset gyro / odometry
    final Runnable resetGyro =
        () ->
            drive.setPose(
                new Pose2d(
                    drive.getPose().getTranslation(),
                    DriverStation.getAlliance().isPresent()
                        ? (DriverStation.getAlliance().get() == DriverStation.Alliance.Red
                            ? new Rotation2d(Math.PI)
                            : new Rotation2d())
                        : new Rotation2d())); // zero gyro

    // Reset gyro to 0° when B button is pressed
    driverController.b().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

    /********************** Autodriving Controls ******************/
    try {
      // Load the path you want to follow using its name in the GUI
      PathPlannerPath path = PathPlannerPath.fromPathFile("FirstPath");

      // Create a path following command using AutoBuilder. This will also trigger
      // event markers.
      driverController
          .a()
          .onTrue(
              DriveCommands.joystickDriveAlongTrajectory(
                  drive,
                  path,
                  () -> driverController.getLeftY(),
                  () -> driverController.getLeftX(),
                  () -> -driverController.getRightX()));

      driverController
          .y()
          .onTrue(
              DriveCommands.pointToPointDriveCommand(
                  drive,
                  new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                  new GoalEndState(0, Rotation2d.fromDegrees(0))));

    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
      // Raise exception
      throw new RuntimeException(e);
    }

    /****************************************************************
     * Right Coral Intake Controls*
     ****************************************************************/
    driverController
        .rightBumper()
        .whileTrue(
            Commands.parallel(
                new PositionJointPositionCommand(
                    rightCoralRotationMotor,
                    () -> PositionJointConstants.CORAL_ROTATION_POSITIONS.DOWN),
                new FlywheelVoltageCommand(rightCoralRollerMotor, () -> 5)));

    driverController
        .rightBumper()
        .whileFalse(
            Commands.parallel(
                new PositionJointPositionCommand(
                    rightCoralRotationMotor,
                    () -> PositionJointConstants.CORAL_ROTATION_POSITIONS.UP),
                new FlywheelVoltageCommand(rightCoralRollerMotor, () -> -1)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
