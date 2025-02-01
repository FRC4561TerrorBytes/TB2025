// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SingleTagAlign extends Command {

  private Drive drive;
  private Vision vision;

  private Command pathCommand;

  private Pose2d targetPose;
  private Pose2d robotPose;

  /** Creates a new goToPose. */
  public SingleTagAlign(Drive drive, Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.vision = vision;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetPose =
        Vision.tagPoses2d.get(18).transformBy(new Transform2d(0.49, 0, Rotation2d.fromDegrees(0)));

    Logger.recordOutput("AutoLineup/Target Pose", targetPose);

    robotPose = vision.getFieldPoseUsingTag(0, drive.getPose().getRotation());
    Logger.recordOutput("AutoLineup/robotPose", robotPose);

    drive.setPose(robotPose);
    pathCommand = AutoBuilder.pathfindToPose(targetPose, new PathConstraints(1, 1, 360, 360));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
    //                 robotPose.getRotation().getDegrees(),
    // targetPose.getRotation().getDegrees())),
    //         robotPose.getRotation()));
    pathCommand.schedule();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
    pathCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathCommand
        .isFinished(); // drive.getPose().getTranslation().getDistance(targetPose.getTranslation())
    // < 0.1
    //     && Math.abs(drive.getRotation().getDegrees() - targetPose.getRotation().getDegrees()) <
    // 0.5;
  }
}
