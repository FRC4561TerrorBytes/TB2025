// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.camera0Name;
import static frc.robot.subsystems.vision.VisionConstants.camera1Name;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoOuttakeDirection;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToPose;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOReal;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.WristIO;
import frc.robot.subsystems.wrist.WristIOReal;
import frc.robot.subsystems.wrist.WristIOSim;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants.Reef;
import frc.robot.util.RobotVisualizer;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public enum ElevatorPosition {
    STOW(0, 45.0, 135),
    CLIMBPREP(0.0, 50.0, 0),
    CLIMBFULL(0.11, 5, 100),
    L1(0.1, 30.0, 0.0),
    L2FRONT(0.07, 47.5, 100.0),
    L2BACK(0.0, 88.0, 135),
    L2ALGAE(0.1, 45.0, 0.0),
    L3FRONT(0.41, 60, 95),
    L3BACK(0.37, 87.0, 130),
    L3ALGAE(0.4, 57.0, -10.0),
    GROUND(0.1, 0.5, -3);

    public double extensionPosition;
    public double pivotPosition;
    public double wristPosition;

    private ElevatorPosition(double extensionPosition, double pivotPosition, double wristPosition) {
      this.extensionPosition = extensionPosition;
      this.pivotPosition = pivotPosition;
      this.wristPosition = wristPosition;
    }
  }

  public enum ScoreLevel {
    L1,
    L2,
    L3;
  }

  private static double distanceAway = 1.25;

  public static Pose2d[] reefFaces = {
    new Pose2d(
        Units.inchesToMeters(144.003), Units.inchesToMeters(158.500), Rotation2d.fromDegrees(180)),
    new Pose2d(
        Units.inchesToMeters(160.373), Units.inchesToMeters(186.857), Rotation2d.fromDegrees(120)),
    new Pose2d(
        Units.inchesToMeters(193.116), Units.inchesToMeters(186.858), Rotation2d.fromDegrees(60)),
    new Pose2d(
        Units.inchesToMeters(209.489), Units.inchesToMeters(158.502), Rotation2d.fromDegrees(0)),
    new Pose2d(
        Units.inchesToMeters(193.118), Units.inchesToMeters(130.145), Rotation2d.fromDegrees(-60)),
    new Pose2d(
        Units.inchesToMeters(160.375), Units.inchesToMeters(130.144), Rotation2d.fromDegrees(-120)),
    new Pose2d(
        Units.inchesToMeters(68) + 0.5,
        Units.inchesToMeters(71) + 0.5,
        Rotation2d.fromDegrees(-126)), // this is Right source, angle could be inaccurate
    new Pose2d(
        Units.inchesToMeters(67.5) + 0.5,
        Units.inchesToMeters(244.5) - 0.5,
        Rotation2d.fromDegrees(126)), // this is left source, angle could be inaccurate
    new Pose2d(
        Units.inchesToMeters(240),
        Units.inchesToMeters(55) + 0.75,
        Rotation2d.fromDegrees(-90)) // this is processer, angle could be inaccurate
  };

  public enum ReefScorePositions {
    FRONT(
        new Pose2d(
            Math.cos(reefFaces[0].getRotation().getRadians()) * distanceAway
                + reefFaces[0].getTranslation().getX(),
            Math.sin(reefFaces[0].getRotation().getRadians()) * distanceAway
                + reefFaces[0].getTranslation().getY(),
            reefFaces[0].getRotation()),
        18),
    FRONTLEFT(
        new Pose2d(
            Math.cos(reefFaces[1].getRotation().getRadians()) * distanceAway
                + reefFaces[1].getTranslation().getX(),
            Math.sin(reefFaces[1].getRotation().getRadians()) * distanceAway
                + reefFaces[1].getTranslation().getY(),
            reefFaces[1].getRotation()),
        19),

    BACKLEFT(
        new Pose2d(
            Math.cos(reefFaces[2].getRotation().getRadians()) * distanceAway
                + reefFaces[2].getTranslation().getX(),
            Math.sin(reefFaces[2].getRotation().getRadians()) * distanceAway
                + reefFaces[2].getTranslation().getY(),
            reefFaces[2].getRotation()),
        20),
    BACK(
        new Pose2d(
            Math.cos(reefFaces[3].getRotation().getRadians()) * distanceAway
                + reefFaces[3].getTranslation().getX(),
            Math.sin(reefFaces[3].getRotation().getRadians()) * distanceAway
                + reefFaces[3].getTranslation().getY(),
            reefFaces[3].getRotation()),
        21),
    BACKRIGHT(
        new Pose2d(
            Math.cos(reefFaces[4].getRotation().getRadians()) * distanceAway
                + reefFaces[4].getTranslation().getX(),
            Math.sin(reefFaces[4].getRotation().getRadians()) * distanceAway
                + reefFaces[4].getTranslation().getY(),
            reefFaces[4].getRotation()),
        22),
    FRONTRIGHT(
        new Pose2d(
            Math.cos(reefFaces[5].getRotation().getRadians()) * distanceAway
                + reefFaces[5].getTranslation().getX(),
            Math.sin(reefFaces[5].getRotation().getRadians()) * distanceAway
                + reefFaces[5].getTranslation().getY(),
            reefFaces[5].getRotation()),
        17),
    RIGHTSOURCE(
        new Pose2d(
            Math.cos(reefFaces[6].getRotation().getRadians()) * distanceAway
                + reefFaces[6].getTranslation().getX(),
            Math.sin(reefFaces[6].getRotation().getRadians()) * distanceAway
                + reefFaces[6].getTranslation().getY(),
            reefFaces[6].getRotation()),
        12),
    LEFTSOURCE(
        new Pose2d(
            Math.cos(reefFaces[7].getRotation().getRadians()) * distanceAway
                + reefFaces[7].getTranslation().getX(),
            Math.sin(reefFaces[7].getRotation().getRadians()) * distanceAway
                + reefFaces[7].getTranslation().getY(),
            reefFaces[7].getRotation()),
        13),
    PROCESSOR(
        new Pose2d(
            Math.cos(reefFaces[8].getRotation().getRadians()) * distanceAway
                + reefFaces[8].getTranslation().getX(),
            Math.sin(reefFaces[8].getRotation().getRadians()) * distanceAway
                + reefFaces[8].getTranslation().getY(),
            reefFaces[8].getRotation()),
        16);

    public Pose2d scorePosition;
    public int aprilTagID;

    private ReefScorePositions(Pose2d pose, int aprilTagID) {
      this.scorePosition = pose;
      this.aprilTagID = aprilTagID;
    }
  }

  // Subsystems
  private final Drive drive;
  private final Intake intake;
  private final Vision vision;
  private final Elevator elevator;
  private final Leds leds = Leds.getInstance();
  private final Climber climber;
  private final Wrist wrist;

  private Boolean slowed = false;
  private Boolean farEnoughAwayFromReefToMoveArm = true;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  private final CommandXboxController reefSelector = new CommandXboxController(2);

  // Manual elevator toggle (incase things go wrong)
  boolean manualElevatorToggle = false;

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
        intake = new Intake(new IntakeIOReal());
        elevator = new Elevator(new ElevatorIOReal());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(camera0Name, drive::getRotation),
                new VisionIOLimelight(camera1Name, drive::getRotation));
        climber = new Climber(new ClimberIOReal());
        wrist = new Wrist(new WristIOReal());

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
        intake = new Intake(new IntakeIOSim());
        elevator = new Elevator(new ElevatorIOSim());
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        climber = new Climber(new ClimberIO() {});
        wrist = new Wrist(new WristIOSim());
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
        intake = new Intake(new IntakeIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        climber = new Climber(new ClimberIO() {});
        wrist = new Wrist(new WristIO() {});
        break;
    }

    // Register NamedCommands for use in PathPlanner // TAKE INTAKE COMMAND TIMEOUT OUT (FOR SIM)
    NamedCommands.registerCommand(
        "Intake", intake.intakeCoral().until(() -> intake.coralPresent()));
    NamedCommands.registerCommand("Outtake", intake.outtakeCoralBack().withTimeout(1.0));
    NamedCommands.registerCommand(
        "OuttakeFront", intake.outtakeCoralFront().until(() -> !intake.coralPresent()));
    NamedCommands.registerCommand("StopIntake", Commands.runOnce(() -> intake.setOutput(0)));
    NamedCommands.registerCommand(
        "L1", Commands.runOnce(() -> setMechanismSetpoint(ElevatorPosition.L1), elevator, wrist));
    NamedCommands.registerCommand(
        "L2",
        Commands.runOnce(() -> setMechanismSetpoint(ElevatorPosition.L2BACK), elevator, wrist));
    NamedCommands.registerCommand(
        "L3",
        Commands.runOnce(() -> setMechanismSetpoint(ElevatorPosition.L3BACK), elevator, wrist));
    NamedCommands.registerCommand(
        "Stow", Commands.runOnce(() -> setMechanismSetpoint(ElevatorPosition.STOW), elevator));
    NamedCommands.registerCommand(
        "Ground", Commands.runOnce(() -> setMechanismSetpoint(ElevatorPosition.GROUND), elevator));
    NamedCommands.registerCommand(
        "BlockTagUpdate", Commands.runOnce(() -> vision.blockTagUpdate(), vision));
    NamedCommands.registerCommand(
        "UnblockTagUpdate", Commands.runOnce(() -> vision.unblockTagUpdate(), vision));
    NamedCommands.registerCommand(
        "WaitForArm", Commands.waitUntil(() -> elevator.mechanismAtSetpoint()));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    /*
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",2
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse)); */
    autoChooser.addOption(
        "LeaveAndStop",
        Commands.run(() -> drive.runVelocity(new ChassisSpeeds(-0.25, 0, 0)), drive)
            .withTimeout(4));

    SmartDashboard.putData(CommandScheduler.getInstance());

    // Configure the button bindings
    configureButtonBindings();

    RobotVisualizer.initialize(elevator, wrist);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> driverController.getLeftY(),
            () -> driverController.getLeftX(),
            () -> -driverController.getRightX(),
            () -> (slowed)));

    // Default Commands

    // Triggers
    Trigger coralIntakeTrigger = new Trigger(() -> intake.coralPresent());
    coralIntakeTrigger.onTrue(
        driverRumbleCommand()
            .withTimeout(1.0)
            .alongWith(Commands.runOnce(() -> Leds.getInstance().coralPresent = true)));

    coralIntakeTrigger.onFalse(Commands.runOnce(() -> Leds.getInstance().coralPresent = false));

    Trigger L1PositionTrigger =
        new Trigger(() -> elevator.getElevatorPosition().equals(ElevatorPosition.L1));

    Trigger L3PositionTrigger =
        new Trigger(() -> elevator.getElevatorPosition().equals(ElevatorPosition.L3BACK));

    Trigger L3FrontPositionTrigger =
        new Trigger(() -> elevator.getElevatorPosition().equals(ElevatorPosition.L3FRONT));

    Trigger L3AlgaeTrigger =
        new Trigger(() -> elevator.getElevatorPosition().equals(ElevatorPosition.L3ALGAE));

    Trigger L2PositionTrigger =
        new Trigger(() -> elevator.getElevatorPosition().equals(ElevatorPosition.L2BACK));

    Trigger L2FrontPositionTrigger =
        new Trigger(() -> elevator.getElevatorPosition().equals(ElevatorPosition.L2FRONT));

    Trigger L2AlgaeTrigger =
        new Trigger(() -> elevator.getElevatorPosition().equals(ElevatorPosition.L2ALGAE));

    Trigger groundPositionTrigger =
        new Trigger(() -> elevator.getElevatorPosition().equals(ElevatorPosition.GROUND));

    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= 2
                    && elevator.getElevatorPosition().equals(ElevatorPosition.CLIMBPREP))
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> climber.setClimberSetpoint(0.18), climber),
                Commands.waitUntil(() -> climber.climberAtSetpoint(0.05)),
                Commands.runOnce(
                    () -> setMechanismSetpoint(ElevatorPosition.CLIMBFULL), elevator)));

    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= 20)
        .onTrue(
            driverRumbleCommand()
                .withTimeout(0.5)
                .beforeStarting(() -> leds.endgameAlert = true)
                .finallyDo(() -> leds.endgameAlert = false)
                .withName("Controller Endgame Al3rt"));

    Trigger farEnoughMoveArm =
        new Trigger(
                () -> {
                  Pose2d centerRobot = drive.getPose();
                  double centerDistance =
                      centerRobot.getTranslation().getDistance(AllianceFlipUtil.apply(Reef.center));

                  return centerDistance > 1.6;
                })
            .onTrue(Commands.runOnce(() -> farEnoughAwayFromReefToMoveArm = true))
            .onFalse(Commands.runOnce(() -> farEnoughAwayFromReefToMoveArm = false));

    groundPositionTrigger
        .and(coralIntakeTrigger)
        .and(() -> !DriverStation.isAutonomous())
        .onTrue(
            Commands.runOnce(() -> setMechanismSetpoint(ElevatorPosition.STOW), elevator, wrist));

    Trigger climbTrigger =
        new Trigger(
            () ->
                elevator.getElevatorPosition().equals(ElevatorPosition.CLIMBPREP)
                    || elevator.getElevatorPosition().equals(ElevatorPosition.CLIMBFULL));

    groundPositionTrigger
        .or(L3PositionTrigger)
        .or(L3AlgaeTrigger)
        .or(L3FrontPositionTrigger)
        .or(L2AlgaeTrigger)
        .or(L2FrontPositionTrigger)
        .or(L2PositionTrigger)
        .or(climbTrigger)
        .onTrue(Commands.runOnce((() -> slowed = true)))
        .onFalse(Commands.runOnce((() -> slowed = false)));

    // Driver Controls

    // Reset gyro to 0° when RS and LS are pressed
    driverController
        .rightStick()
        .and(driverController.leftStick())
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // Toggle coral intake when LT is pressed
    driverController
        .leftTrigger()
        .onTrue(
            Commands.sequence(
                    Commands.runOnce(() -> setMechanismSetpoint(ElevatorPosition.STOW), elevator),
                    Commands.waitUntil(() -> elevator.mechanismAtSetpoint()))
                .onlyIf(L3PositionTrigger.or(L3AlgaeTrigger))
                .finallyDo(() -> setMechanismSetpoint(ElevatorPosition.GROUND)))
        .toggleOnTrue(intake.intakeCoral().until(() -> intake.coralPresent()));

    // Outtake coral while RT is held, separate logic for L1
    driverController
        .rightTrigger()
        .and(L1PositionTrigger.negate())
        .whileTrue(new AutoOuttakeDirection(intake, drive::getPose));
    driverController.rightTrigger().and(L1PositionTrigger).whileTrue(intake.outtakeCoralBack());

    // Run lineup sequence when B is held
    driverController
        .b()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> elevator.requestScoreLevel(ScoreLevel.L2)),
                Commands.runOnce(
                    () -> {
                      Pose2d centerRobot = drive.getPose();
                      Transform2d forwardOffset =
                          new Transform2d(new Translation2d(-0.38, 0.0), new Rotation2d());
                      Pose2d frontRobot = centerRobot.transformBy(forwardOffset);

                      double centerDistance =
                          centerRobot
                              .getTranslation()
                              .getDistance(AllianceFlipUtil.apply(Reef.center));
                      double backDistance =
                          frontRobot
                              .getTranslation()
                              .getDistance(AllianceFlipUtil.apply(Reef.center));

                      // SCORE OUT BACK
                      if (centerDistance > 1.6) {
                        if (centerDistance <= backDistance) {
                          setMechanismSetpoint(ElevatorPosition.L2FRONT);
                        }
                        // SCORE OUT FRONT
                        else {
                          setMechanismSetpoint(ElevatorPosition.L2BACK);
                        }
                      }
                    })));

    // Run automated scoring when LB is held
    driverController
        .rightBumper()
        .whileTrue(
            Commands.sequence(
                Commands.runOnce(() -> leds.autoScoring = true),
                Commands.runOnce(
                    () ->
                        drive.setAutoAlignOffsetX(
                            Units.inchesToMeters(Constants.SCORING_POSITION_OFFSET))),
                new DriveToPose(drive, elevator, wrist)))
        .onFalse(
            Commands.sequence(
                Commands.runOnce(() -> leds.autoScoring = false),
                Commands.runOnce(() -> drive.stop(), drive)));

    driverController
        .leftBumper()
        .whileTrue(
            Commands.sequence(
                Commands.runOnce(() -> leds.autoScoring = true),
                Commands.runOnce(
                    () ->
                        drive.setAutoAlignOffsetX(
                            Units.inchesToMeters(-Constants.SCORING_POSITION_OFFSET))),
                new DriveToPose(drive, elevator, wrist)))
        .onFalse(
            Commands.sequence(
                Commands.runOnce(() -> leds.autoScoring = false),
                Commands.runOnce(() -> drive.stop(), drive)));
    // Commands.runOnce(() -> elevator.setSetpoint(ElevatorPosition.STOW), elevator)));

    // Pathfind to processor when X is held
    driverController
        .x()
        .onTrue(
            Commands.sequence(
                    Commands.runOnce(() -> setMechanismSetpoint(ElevatorPosition.L2BACK), elevator),
                    Commands.waitUntil(() -> elevator.mechanismAtSetpoint()))
                .onlyIf(L3PositionTrigger.or(L3AlgaeTrigger))
                .finallyDo(() -> setMechanismSetpoint(ElevatorPosition.STOW))
                .onlyIf(
                    () -> {
                      Pose2d centerRobot = drive.getPose();
                      double centerDistance =
                          centerRobot
                              .getTranslation()
                              .getDistance(AllianceFlipUtil.apply(Reef.center));

                      return centerDistance > 1.6;
                    }));

    driverController
        .a()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> elevator.requestScoreLevel(ScoreLevel.L1)),
                Commands.runOnce(
                    () -> setMechanismSetpoint(ElevatorPosition.L1), elevator, wrist)));

    driverController
        .y()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> elevator.requestScoreLevel(ScoreLevel.L3)),
                Commands.runOnce(
                    () -> {
                      Pose2d centerRobot = drive.getPose();
                      Transform2d forwardOffset =
                          new Transform2d(new Translation2d(-0.38, 0.0), new Rotation2d());
                      Pose2d frontRobot = centerRobot.transformBy(forwardOffset);

                      double centerDistance =
                          centerRobot
                              .getTranslation()
                              .getDistance(AllianceFlipUtil.apply(Reef.center));
                      double backDistance =
                          frontRobot
                              .getTranslation()
                              .getDistance(AllianceFlipUtil.apply(Reef.center));

                      // SCORE OUT BACK
                      if (centerDistance > 1.6) {
                        if (centerDistance <= backDistance) {
                          setMechanismSetpoint(ElevatorPosition.L3FRONT);
                        }
                        // SCORE OUT FRONT
                        else {
                          setMechanismSetpoint(ElevatorPosition.L3BACK);
                        }
                      }
                    })));

    // Set elevator to ALGAEINTAKE when DPAD RIGHT is pressed
    driverController
        .povRight()
        .whileTrue(Commands.run(() -> climber.setOutput(0.6), climber))
        .onFalse(Commands.runOnce(() -> climber.setOutput(0), climber));

    driverController
        .povLeft()
        .whileTrue(Commands.run(() -> climber.setOutput(-0.75), climber))
        .onFalse(Commands.runOnce(() -> climber.setOutput(0), climber));

    driverController
        .povUp()
        .onTrue(
            Commands.runOnce(() -> setMechanismSetpoint(ElevatorPosition.CLIMBPREP), elevator)
                .alongWith(Commands.runOnce(() -> climber.setClimberSetpoint(0.425), climber)));

    driverController
        .povDown()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> climber.setClimberSetpoint(0.18), climber),
                Commands.waitUntil(() -> climber.climberAtSetpoint(0.05)),
                Commands.runOnce(
                    () -> setMechanismSetpoint(ElevatorPosition.CLIMBFULL), elevator)));

    // Operator Controls

    // Toggle manual elevator control when BACK is pressed
    // OUTDATED (MAYBE SHOULDNT USE SOON)

    // Move elevator to stow when X is pressed
    operatorController
        .x()
        .onTrue(
            Commands.sequence(
                    Commands.runOnce(() -> setMechanismSetpoint(ElevatorPosition.L2BACK), elevator),
                    Commands.waitUntil(() -> elevator.mechanismAtSetpoint()))
                .onlyIf(L3PositionTrigger.or(L3AlgaeTrigger))
                .finallyDo(() -> setMechanismSetpoint(ElevatorPosition.STOW)));

    // Move elevator to L3 Algae removal when RT is pressed
    operatorController
        .rightBumper()
        .onTrue(Commands.runOnce(() -> setMechanismSetpoint(ElevatorPosition.L3ALGAE), elevator));

    // Move elevator to L2 Algae removal when LT is pressed
    operatorController
        .leftBumper()
        .onTrue(Commands.runOnce(() -> setMechanismSetpoint(ElevatorPosition.L2ALGAE), elevator));

    // Move elevator to L1 position when A is pressed
    operatorController
        .a()
        .onTrue(Commands.runOnce(() -> setMechanismSetpoint(ElevatorPosition.L1), elevator, wrist));

    // Move elevator to L2 position when B is pressed
    operatorController
        .b()
        .onTrue(
            Commands.runOnce(() -> setMechanismSetpoint(ElevatorPosition.L2BACK), elevator, wrist));

    // Move elevator to L3 position when Y is pressed
    operatorController
        .y()
        .onTrue(
            Commands.runOnce(() -> setMechanismSetpoint(ElevatorPosition.L3BACK), elevator, wrist));

    operatorController
        .rightTrigger()
        .whileTrue(Commands.run(() -> intake.setOutput(-1)))
        .onFalse(Commands.runOnce(() -> intake.setOutput(0), intake));

    operatorController
        .leftTrigger()
        .whileTrue(Commands.run(() -> intake.setOutput(-0.2)))
        .onFalse(Commands.runOnce(() -> intake.setOutput(0), intake));

    // REEF SELECTION USING KEYBOARD

    // Set lineup position to I/J
    reefSelector
        .button(5)
        .onTrue(
            Commands.runOnce(() -> drive.setSelectedScorePosition(ReefScorePositions.BACKLEFT))
                .ignoringDisable(true));

    // Set lineup position to E/F
    reefSelector
        .button(3)
        .onTrue(
            Commands.runOnce(() -> drive.setSelectedScorePosition(ReefScorePositions.BACKRIGHT))
                .ignoringDisable(true));

    // Set lineup position to K/L
    reefSelector
        .button(6)
        .onTrue(
            Commands.runOnce(() -> drive.setSelectedScorePosition(ReefScorePositions.FRONTLEFT))
                .ignoringDisable(true));

    // HI MANBIR :D

    // Set lineup position to C/D
    reefSelector
        .button(2)
        .onTrue(
            Commands.runOnce(() -> drive.setSelectedScorePosition(ReefScorePositions.FRONTRIGHT))
                .ignoringDisable(true));

    // Set lineup position to G/H
    reefSelector
        .button(4)
        .onTrue(
            Commands.runOnce(() -> drive.setSelectedScorePosition(ReefScorePositions.BACK))
                .ignoringDisable(true));

    // Set lineup position to A/B
    reefSelector
        .button(1)
        .onTrue(
            Commands.runOnce(() -> drive.setSelectedScorePosition(ReefScorePositions.FRONT))
                .ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  private void setMechanismSetpoint(ElevatorPosition position) {
    elevator.setSetpoint(position);
    wrist.setSetpoint(position);
  }

  @AutoLogOutput(key = "TEST/Score back?")
  private boolean scoreBack() {
    return Math.abs(
                drive.getSelectedPose().getRotation().getDegrees()
                    - drive.getRotation().getDegrees())
            < 90
        && Math.abs(
                drive.getSelectedPose().getRotation().getDegrees()
                    - drive.getRotation().getDegrees())
            <= 270;
  }

  private boolean mechanismAtSetpoint() {
    return elevator.elevatorAtSetpoint() && wrist.wristAtSetpoint(2);
  }

  private Command driverRumbleCommand() {
    return Commands.startEnd(
        () -> {
          driverController.getHID().setRumble(RumbleType.kBothRumble, 1.0);
        },
        () -> {
          driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
        });
  }
}
