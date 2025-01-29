// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class goToPose extends Command {

  private Drive drive;
  private Vision vision;

  private Pose2d targetPose;
  private double DRIVE_KP = 4;
  private double DRIVE_KD = 0;
  private double DRIVE_MAX_VELOCITY = 3;
  private double DRIVE_MAX_ACCELERATION = 3;

  private double ROTATION_KP = 7;
  private double ROTATION_KD = 2;
  private double ROTATION_MAX_VELOCITY = 30;
  private double ROTATION_MAX_ACCELERATION = 15;

  private boolean useSingleTag = false;

  private Pose2d robotPose;

  private ProfiledPIDController drivePIDController =
      new ProfiledPIDController(
          DRIVE_KP,
          0.0,
          DRIVE_KD,
          new TrapezoidProfile.Constraints(DRIVE_MAX_VELOCITY, DRIVE_MAX_ACCELERATION));

  private ProfiledPIDController rotationPIDController =
      new ProfiledPIDController(
          ROTATION_KP,
          0.0,
          ROTATION_KD,
          new TrapezoidProfile.Constraints(ROTATION_MAX_VELOCITY, ROTATION_MAX_ACCELERATION));
  /** Creates a new goToPose. */
  public goToPose(Drive drive, Vision vision, Pose2d target) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.vision = vision;
    targetPose = target;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (drive.getPose().getTranslation().getDistance(targetPose.getTranslation()) < 1.5
        && !useSingleTag) {
      useSingleTag = true;
    }
    if (!useSingleTag) {
      robotPose = drive.getPose();
    } else {
      // convert local to field
      // vision.getFieldPoseUsingTag(0, 0, 20);
      robotPose = drive.getPose();
    }

    // drive.runVelocity(
    //     new ChassisSpeeds(
    //         drivePIDController.calculate(robotPose.getX(), targetPose.getX()),
    //         drivePIDController.calculate(robotPose.getY(), targetPose.getY()),
    //         rotationPIDController.calculate(
    //             robotPose.getRotation().getDegrees(), targetPose.getRotation().getDegrees())));
    // drive.runVelocity(
    //     ChassisSpeeds.fromFieldRelativeSpeeds(
    //         drivePIDController.calculate(robotPose.getX(), targetPose.getX()),
    //         drivePIDController.calculate(robotPose.getY(), targetPose.getY()),
    //         Units.degreesToRadians(
    //             rotationPIDController.calculate(
    //                 robotPose.getRotation().getDegrees(), targetPose.getRotation().getDegrees())),
    //         robotPose.getRocity(
    //     ChassisSpeeds.fromFieldRelativeSpeeds(
    //         drivePIDController.calculate(robotPose.getX(), targetPose.getX()),
    //         drivePIDController.calculate(robotPose.getY(), targetPose.getY()),
    //         Units.degreesToRadians(
    //             rotationPIDController.calculate(
    //                 robotPose.getRotation().getDegrees(), targetPose.getRotation().getDegrees())),
    //         robotPose.getRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drive.getPose().getTranslation().getDistance(targetPose.getTranslation()) < 0.1
        && Math.abs(drive.getRotation().getDegrees() - targetPose.getRotation().getDegrees()) < 0.5;
  }
}
