// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
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
  private double distanceAway = -0.55;

  /** Creates a new goToPose. */
  public SingleTagAlign(Drive drive, Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.vision = vision;

    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d selectedPosition = drive.getSelectedScorePosition().scorePosition;

    targetPose =
        new Pose2d(
            Math.cos(selectedPosition.getRotation().getRadians()) * distanceAway
                + selectedPosition.getTranslation().getX(),
            Math.sin(selectedPosition.getRotation().getRadians()) * distanceAway
                + selectedPosition.getTranslation().getY(),
            selectedPosition.getRotation());

    Logger.recordOutput("AutoLineup/Target Pose", targetPose);

    robotPose = vision.getFieldPoseUsingTag(0, drive.getPose().getRotation());
    Logger.recordOutput("AutoLineup/robotPose", robotPose);

    drive.setPose(robotPose);
    pathCommand = AutoBuilder.pathfindToPose(targetPose, new PathConstraints(1, 1, 180, 180));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pathCommand.schedule();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pathCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathCommand.isFinished();
  }
}
