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
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera0;
import static frc.robot.subsystems.vision.VisionConstants.robotToCamera1;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.AutomatedScore;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.goToPose;
import frc.robot.generated.TunerConstants;
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
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public enum ElevatorPosition {
    STOW(0, 0.0),
    SOURCE(0.15, 47),
    L1(0, 120.0),
    L2(0.1, 100.0),
    L3(0.45, 92.0),
    L4(0.5, 10.0);

    public double extensionPosition;
    public double pivotPosition;

    private ElevatorPosition(double extensionPosition, double pivotPosition) {
      this.extensionPosition = extensionPosition;
      this.pivotPosition = pivotPosition;
    }
  }

  private static double distanceAway = 1;

  private static Pose2d[] reefFaces = {
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
        Units.inchesToMeters(160.375), Units.inchesToMeters(130.144), Rotation2d.fromDegrees(-120))
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
        17);
    public Pose2d scorePosition;
    public int aprilTagID;

    private ReefScorePositions(Pose2d pose, int aprilTagID) {
      this.scorePosition = pose;
      this.aprilTagID = aprilTagID;
    }
  }

  // Subsystems
  private final Drive drive;
  private final Elevator elevator;
  private final Vision vision;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

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
                new VisionIOLimelight(camera0Name, drive::getRotation),
                new VisionIOLimelight(camera1Name, drive::getRotation));

        elevator = new Elevator(new ElevatorIOReal());
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
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));

        elevator = new Elevator(new ElevatorIOSim());
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
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        elevator = new Elevator(new ElevatorIO() {});
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

    // Lock to 0° when A button is held
    // driverController
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -driverController.getLeftY(),
    //             () -> -driverController.getLeftX(),
    //             () -> new Rotation2d()));

    // Switch to X pattern when X button is pressed
    // driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    // driverController
    //     .y()
    //     .whileTrue(new goToPose(drive, vision, new Pose2d(3, 3, Rotation2d.fromDegrees(30))));

    driverController
        .a()
        .whileTrue(
            Commands.sequence(
                new DriveToPose(drive, vision),
                Commands.parallel(
                    new AutomatedScore(elevator, vision),
                    new goToPose(drive, vision, drive.getPose()))))
        .onFalse(Commands.runOnce(() -> drive.stop(), drive));

    driverController
        .b()
        .whileTrue(new DriveToPose(drive, vision))
        .onFalse(Commands.runOnce(() -> drive.stop(), drive));

    driverController
        .povDown()
        .whileTrue(new RunCommand(() -> drive.runVelocity(new ChassisSpeeds(-1, 0, 0))));
    driverController
        .povUp()
        .whileTrue(new RunCommand(() -> drive.runVelocity(new ChassisSpeeds(1, 0, 0))));
    driverController
        .povLeft()
        .whileTrue(new RunCommand(() -> drive.runVelocity(new ChassisSpeeds(0, 1, 0))));
    driverController
        .povRight()
        .whileTrue(new RunCommand(() -> drive.runVelocity(new ChassisSpeeds(0, -1, 0))));

    // Reset gyro to 0° when B button is pressed
    // driverController
    //     .b()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     drive.setPose(
    //                         new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
    //                 drive)
    //             .ignoringDisable(true));

    operatorController
        .povUpLeft()
        .onTrue(
            Commands.runOnce(() -> drive.setSelectedScorePosition(ReefScorePositions.BACKLEFT)));
    operatorController
        .povUpRight()
        .onTrue(
            Commands.runOnce(() -> drive.setSelectedScorePosition(ReefScorePositions.BACKRIGHT)));
    operatorController
        .povDownLeft()
        .onTrue(
            Commands.runOnce(() -> drive.setSelectedScorePosition(ReefScorePositions.FRONTLEFT)));
    operatorController
        .povDownRight()
        .onTrue(
            Commands.runOnce(() -> drive.setSelectedScorePosition(ReefScorePositions.FRONTRIGHT)));
    operatorController
        .povUp()
        .onTrue(Commands.runOnce(() -> drive.setSelectedScorePosition(ReefScorePositions.BACK)));
    operatorController
        .povDown()
        .onTrue(Commands.runOnce(() -> drive.setSelectedScorePosition(ReefScorePositions.FRONT)));

    operatorController
        .a()
        .onTrue(
            Commands.runOnce(
                () -> elevator.requestElevatorPosition(ElevatorPosition.L1), elevator));

    operatorController
        .b()
        .onTrue(
            Commands.runOnce(
                () -> elevator.requestElevatorPosition(ElevatorPosition.L2), elevator));

    operatorController
        .y()
        .onTrue(
            Commands.runOnce(
                () -> elevator.requestElevatorPosition(ElevatorPosition.L3), elevator));

    operatorController
        .x()
        .onTrue(Commands.runOnce(() -> elevator.setSetpoint(ElevatorPosition.STOW), elevator));
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
