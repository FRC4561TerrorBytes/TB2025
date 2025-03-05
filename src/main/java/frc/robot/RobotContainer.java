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
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.SingleTagAlign;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.algaeManipulator.AlgaeManipulator;
import frc.robot.subsystems.algaeManipulator.AlgaeManipulatorIO;
import frc.robot.subsystems.algaeManipulator.AlgaeManipulatorIOReal;
import frc.robot.subsystems.algaeManipulator.AlgaeManipulatorIOSim;
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
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants.ReefLevel;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public enum ElevatorPosition {
    STOW(0, 20.0),
    SOURCE(0.1, 46),
    CLIMBPREP(0.45, 50.0),
    CLIMBFULL(0.2, 10),
    ALGAEINTAKE(0.1, 15),
    L1(0.1, 20.0),
    L2(0.0, 85.0),
    L2ALGAE(0, 90),
    L3(0.55, 90.0),
    L3ALGAE(0.4, 90),
    L3RETURN(0.55, 65),
    L3RETURN2(0.2, 65),
    L4(0.5, 90.0);

    public double extensionPosition;
    public double pivotPosition;

    private ElevatorPosition(double extensionPosition, double pivotPosition) {
      this.extensionPosition = extensionPosition;
      this.pivotPosition = pivotPosition;
    }
  }

  private static double distanceAway = 1;

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
    PROCESSER(
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
  private final AlgaeManipulator algaeManipulator;
  private final Leds leds = Leds.getInstance();

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

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
        algaeManipulator = new AlgaeManipulator(new AlgaeManipulatorIOReal());
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
        algaeManipulator = new AlgaeManipulator(new AlgaeManipulatorIOSim());
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

        algaeManipulator = new AlgaeManipulator(new AlgaeManipulatorIO() {});
        break;
    }

    // Register NamedCommands for use in PathPlanner
    NamedCommands.registerCommand(
        "Intake",
        intake
            .intakeCoral()
            .withTimeout(0.5)); // REMOVE WITH TIMEOUT ADDED FOR SIM TESTING PURPOSES
    NamedCommands.registerCommand("Outtake", intake.outtakeCoral().withTimeout(0.5));
    NamedCommands.registerCommand(
        "L1", Commands.runOnce(() -> elevator.setSetpoint(ElevatorPosition.L1), elevator));
    NamedCommands.registerCommand(
        "L2", Commands.runOnce(() -> elevator.setSetpoint(ElevatorPosition.L2), elevator));
    NamedCommands.registerCommand(
        "L3", Commands.runOnce(() -> elevator.setSetpoint(ElevatorPosition.L3), elevator));
    NamedCommands.registerCommand(
        "Source", Commands.runOnce(() -> elevator.setSetpoint(ElevatorPosition.SOURCE), elevator));
    NamedCommands.registerCommand(
        "Stow",
        Commands.sequence(
                Commands.runOnce(() -> elevator.setSetpoint(ElevatorPosition.L2), elevator),
                Commands.waitUntil(() -> elevator.mechanismAtSetpoint()))
            .onlyIf(() -> elevator.getElevatorPosition().equals(ElevatorPosition.L3))
            .andThen(
                Commands.runOnce(() -> elevator.setSetpoint(ElevatorPosition.STOW), elevator)));
    NamedCommands.registerCommand(
        "L3Algae",
        Commands.runOnce(() -> elevator.setSetpoint(ElevatorPosition.L3ALGAE), elevator));
    NamedCommands.registerCommand(
        "WaitForArm", Commands.waitUntil(() -> elevator.mechanismAtSetpoint()));
    NamedCommands.registerCommand(
        "L2Algae",
        Commands.runOnce(() -> elevator.setSetpoint(ElevatorPosition.L2ALGAE), elevator));
    NamedCommands.registerCommand(
        "SpinAlgae",
        Commands.run(() -> algaeManipulator.setOutput(1.0), algaeManipulator)
            // .withTimeout(2.5)
            .andThen(Commands.runOnce(() -> algaeManipulator.setOutput(0), algaeManipulator)));
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

    SmartDashboard.putData(CommandScheduler.getInstance());

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
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    // Default Commands

    intake.setDefaultCommand(new RunCommand(() -> intake.setOutput(0.0), intake));
    algaeManipulator.setDefaultCommand(algaeManipulator.stopAlgaeManipulator());

    // Triggers
    Trigger coralIntakeTrigger = new Trigger(() -> intake.coralPresent());
    coralIntakeTrigger.onTrue(driverRumbleCommand().withTimeout(1.0));

    Trigger algaePositionTrigger =
        new Trigger(() -> elevator.getElevatorPosition().equals(ElevatorPosition.ALGAEINTAKE));

    Trigger L1PositionTrigger =
        new Trigger(() -> elevator.getElevatorPosition().equals(ElevatorPosition.L1));

    Trigger L3PositionTrigger =
        new Trigger(() -> elevator.getElevatorPosition().equals(ElevatorPosition.L3));
    Trigger L3AlgaeTrigger =
        new Trigger(() -> elevator.getElevatorPosition().equals(ElevatorPosition.L3ALGAE));

    // Driver Controls

    // Reset gyro to 0° when B button is pressed
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

    // Toggle coral intake when left bumper is pressed
    driverController
        .leftTrigger()
        .and(algaePositionTrigger.negate())
        .onTrue(Commands.runOnce(() -> elevator.setSetpoint(ElevatorPosition.SOURCE), elevator))
        .toggleOnTrue(intake.intakeCoral());

    driverController
        .leftTrigger()
        .and(algaePositionTrigger)
        .toggleOnTrue(Commands.run(() -> intake.setOutput(-0.75), intake));

    // Outtake coral while right bumper is held
    driverController
        .rightTrigger()
        .and(L1PositionTrigger.negate())
        .whileTrue(intake.outtakeCoral());

    driverController.rightTrigger().and(L1PositionTrigger).whileTrue(intake.outtakeL1Coral());

    driverController
        .leftBumper()
        .whileTrue(
            AutoBuilder.pathfindToPose(
                AllianceFlipUtil.apply(ReefScorePositions.LEFTSOURCE.scorePosition),
                new PathConstraints(
                    1, 1, Units.degreesToRadians(360), Units.degreesToRadians(360))))
        .onFalse(Commands.runOnce(() -> drive.stop(), drive));

    driverController
        .rightBumper()
        .whileTrue(
            AutoBuilder.pathfindToPose(
                AllianceFlipUtil.apply(ReefScorePositions.RIGHTSOURCE.scorePosition),
                new PathConstraints(
                    1, 1, Units.degreesToRadians(360), Units.degreesToRadians(360))))
        .onFalse(Commands.runOnce(() -> drive.stop(), drive));

    // Run lineup sequence when B is held
    driverController
        .b()
        .whileTrue(
            Commands.sequence(new DriveToPose(drive, vision), new SingleTagAlign(drive, vision)))
        .onFalse(Commands.runOnce(() -> drive.stop(), drive));

    // Run automated scoring when A is held
    driverController
        .a()
        .whileTrue(
            Commands.sequence(
                Commands.runOnce(() -> leds.autoScoring = true),
                new DriveToPose(drive, vision),
                Commands.parallel(
                    new SingleTagAlign(drive, vision),
                    Commands.runOnce(
                        () -> elevator.setSetpoint(elevator.getRequestedElevatorPosition()),
                        elevator)),
                Commands.waitUntil(() -> elevator.mechanismAtSetpoint()),
                intake.outtakeCoral().withTimeout(0.5)))
        .onFalse(
            Commands.sequence(
                    Commands.runOnce(
                        () -> elevator.setSetpoint(ElevatorPosition.L3RETURN), elevator),
                    Commands.waitUntil(() -> elevator.mechanismAtSetpoint()),
                    Commands.runOnce(
                        () -> elevator.setSetpoint(ElevatorPosition.L3RETURN2), elevator),
                    Commands.waitUntil(() -> elevator.mechanismAtSetpoint()))
                .onlyIf(L3PositionTrigger.or(L3AlgaeTrigger))
                .alongWith(Commands.runOnce(() -> drive.stop(), drive))
                .andThen(
                    Commands.runOnce(() -> elevator.setSetpoint(ElevatorPosition.STOW), elevator))
                .alongWith(Commands.runOnce(() -> leds.autoScoring = false)));

    // Run algae bar when Y is held
    driverController
        .y()
        .whileTrue(new RunCommand(() -> algaeManipulator.setOutput(1), algaeManipulator));

    driverController
        .x()
        .whileTrue(
            AutoBuilder.pathfindToPose(
                AllianceFlipUtil.apply(ReefScorePositions.PROCESSER.scorePosition),
                new PathConstraints(
                    1, 1, Units.degreesToRadians(360), Units.degreesToRadians(360))))
        .onFalse(Commands.runOnce(() -> drive.stop(), drive));

    driverController
        .povRight()
        .onTrue(
            Commands.runOnce(() -> elevator.setSetpoint(ElevatorPosition.ALGAEINTAKE), elevator));

    driverController
        .povUp()
        .onTrue(Commands.runOnce(() -> elevator.setSetpoint(ElevatorPosition.CLIMBPREP), elevator));

    driverController
        .povDown()
        .onTrue(Commands.runOnce(() -> elevator.setSetpoint(ElevatorPosition.CLIMBFULL), elevator));

    // Operator Controls

    // Set lineup position to I/J
    operatorController
        .povUpLeft()
        .onTrue(Commands.runOnce(() -> drive.setSelectedScorePosition(ReefScorePositions.BACKLEFT)))
        .debounce(0.5, DebounceType.kFalling);

    // Set lineup position to E/F
    operatorController
        .povUpRight()
        .onTrue(
            Commands.runOnce(() -> drive.setSelectedScorePosition(ReefScorePositions.BACKRIGHT)))
        .debounce(0.5, DebounceType.kFalling);

    // Set lineup position to K/L
    operatorController
        .povDownLeft()
        .onTrue(
            Commands.runOnce(() -> drive.setSelectedScorePosition(ReefScorePositions.FRONTLEFT)))
        .debounce(0.5, DebounceType.kFalling);

    // Set lineup position to C/D
    operatorController
        .povDownRight()
        .onTrue(
            Commands.runOnce(() -> drive.setSelectedScorePosition(ReefScorePositions.FRONTRIGHT)))
        .debounce(0.5, DebounceType.kFalling);

    // Set lineup position to G/H
    operatorController
        .povUp()
        .onTrue(Commands.runOnce(() -> drive.setSelectedScorePosition(ReefScorePositions.BACK)))
        .debounce(0.5, DebounceType.kFalling);

    // Set lineup position to A/B
    operatorController
        .povDown()
        .onTrue(Commands.runOnce(() -> drive.setSelectedScorePosition(ReefScorePositions.FRONT)))
        .debounce(0.5, DebounceType.kFalling);

    // Adjust scoring position to left side
    operatorController
        .leftBumper()
        .onTrue(
            new InstantCommand(
                () -> {
                  drive.setAutoAlignOffsetX(
                      Units.inchesToMeters(-Constants.SCORING_POSITION_OFFSET));
                }));

    // Adjust scoring position to right side
    operatorController
        .rightBumper()
        .onTrue(
            new InstantCommand(
                () -> {
                  drive.setAutoAlignOffsetX(
                      Units.inchesToMeters(Constants.SCORING_POSITION_OFFSET));
                }));

    // Toggle manual elevator control when BACK is pressed
    operatorController
        .back()
        .toggleOnTrue(
            Commands.runOnce(() -> manualElevatorToggle = !manualElevatorToggle)
                .alongWith(Commands.runOnce(() -> leds.manualElevator = !leds.manualElevator)));

    // Move elevator to stow when X is pressed
    operatorController
        .x()
        .onTrue(
            Commands.sequence(
                    Commands.runOnce(() -> elevator.setSetpoint(ElevatorPosition.L2), elevator),
                    Commands.waitUntil(() -> elevator.mechanismAtSetpoint()))
                .onlyIf(L3PositionTrigger.or(L3AlgaeTrigger))
                .finallyDo(() -> elevator.setSetpoint(ElevatorPosition.STOW)));

    // operatorController
    //     .x()
    //     .and(L3PositionTrigger.negate())
    //     .onTrue(Commands.runOnce(() -> elevator.setSetpoint(ElevatorPosition.STOW), elevator));

    // Move elevator to L3 Algae removal when RT is pressed
    operatorController
        .rightTrigger()
        .onTrue(Commands.runOnce(() -> elevator.setSetpoint(ElevatorPosition.L3ALGAE), elevator));

    // Move elevator to L2 Algae removal when LT is pressed
    operatorController
        .leftTrigger()
        .onTrue(Commands.runOnce(() -> elevator.setSetpoint(ElevatorPosition.L2ALGAE), elevator));

    // Move elevator to L1 position when A is pressed and Manual mode is TRUE
    operatorController
        .a()
        .and(() -> manualElevatorToggle)
        .onTrue(new InstantCommand(() -> elevator.setSetpoint(ElevatorPosition.L1)));

    // Move elevator to L2 position when B is pressed and Manual mode is TRUE
    operatorController
        .b()
        .and(() -> manualElevatorToggle)
        .onTrue(new InstantCommand(() -> elevator.setSetpoint(ElevatorPosition.L2)));

    // Move elevator to L3 position when Y is pressed and Manual mode is TRUE
    operatorController
        .y()
        .and(() -> manualElevatorToggle)
        .onTrue(Commands.runOnce(() -> elevator.setSetpoint(ElevatorPosition.L3), elevator));

    // Request L1 automated score position when A is pressed and Manual mode is FALSE
    operatorController
        .a()
        .and(() -> !manualElevatorToggle)
        .onTrue(
            Commands.runOnce(() -> elevator.requestElevatorPosition(ElevatorPosition.L1), elevator)
                .alongWith(Commands.runOnce(() -> leds.autoScoringLevel = ReefLevel.L1)));

    // Request L2 automated score position when B is pressed and Manual mode is FALSE
    operatorController
        .b()
        .and(() -> !manualElevatorToggle)
        .onTrue(
            Commands.runOnce(() -> elevator.requestElevatorPosition(ElevatorPosition.L2), elevator)
                .alongWith(Commands.runOnce(() -> leds.autoScoringLevel = ReefLevel.L2)));

    // Request L3 automated score position when Y is pressed and Manual mode is FALSE
    operatorController
        .y()
        .and(() -> !manualElevatorToggle)
        .onTrue(
            Commands.runOnce(() -> elevator.requestElevatorPosition(ElevatorPosition.L3), elevator)
                .alongWith(Commands.runOnce(() -> leds.autoScoringLevel = ReefLevel.L3)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
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
