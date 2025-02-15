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

import com.pathplanner.lib.auto.AutoBuilder;
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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOReal;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public enum ElevatorPosition {
    STOW(0, 5.0),
    SOURCE(0.15, 47),
    L1(0, 90.0),
    L2(0.1, 90.0),
    L3(0.45, 90.0),
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
  private final Pivot pivot;
  private final Intake intake;
  private final Vision vision;
  private final AlgaeManipulator algaeManipulator;

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

        pivot = new Pivot(new PivotIOReal());
        intake = new Intake(new IntakeIOReal());
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(camera0Name, drive::getRotation));
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

        pivot = new Pivot(null);
        intake = new Intake(null);
        vision = null;
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

        pivot = new Pivot(new PivotIO() {});
        intake = new Intake(new IntakeIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        algaeManipulator = new AlgaeManipulator(new AlgaeManipulatorIO() {});
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

    intake.setDefaultCommand(new RunCommand(() -> intake.setOutput(0.0), intake));
    algaeManipulator.setDefaultCommand(algaeManipulator.stopAlgaeManipulator());

    // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

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

    driverController.leftBumper().whileTrue(new RunCommand(() -> intake.setOutput(0.3), intake));
    driverController.rightBumper().whileTrue(new RunCommand(() -> intake.setOutput(-0.3), intake));

    driverController
        .a()
        .whileTrue(
            Commands.sequence(new DriveToPose(drive, vision), new SingleTagAlign(drive, vision)))
        .onFalse(Commands.runOnce(() -> drive.stop(), drive));

    driverController
        .b()
        .whileTrue(new SingleTagAlign(drive, vision))
        .onFalse(Commands.runOnce(() -> drive.stop(), drive));

    // driverController.y().onTrue(Commands.runOnce(() -> pivot.setPivotPosition(57), pivot));
    // driverController.b().onTrue(Commands.runOnce(() -> pivot.setPivotPosition(25), pivot));
    driverController.x().onTrue(Commands.runOnce(() -> pivot.setPivotPosition(5), pivot));
    driverController.y().onTrue(Commands.runOnce(() -> pivot.setPivotPosition(90), pivot));

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

    operatorController.leftTrigger().whileTrue(intake.intakeCoral());
    operatorController.rightTrigger().whileTrue(intake.outtakeCoral());
    operatorController
        .rightStick()
        .whileTrue(Commands.run(() -> algaeManipulator.setOutput(1), algaeManipulator));
    operatorController
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
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
