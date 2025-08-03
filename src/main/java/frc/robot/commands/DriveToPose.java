// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPose extends Command {

  private Drive drive;
  private Vision vision;
  private int endTagId;
  private boolean seenEndTag;
  private Command pathCommand;

  private Pose2d targetPose;
  private boolean scoreBack = true;
  private double distanceAway = Units.inchesToMeters(-25.654);

  /** Creates a new DriveToPose. */
  public DriveToPose(Drive drive, Vision vision) {
    this.drive = drive;
    this.vision = vision;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.stop();
    drive.setPathFinished(false);

    targetPose = drive.getFinalTargetPose();

    Logger.recordOutput("Auto Lineup/Target Pose", targetPose);

    pathCommand = AutoBuilder.pathfindToPose(targetPose, new PathConstraints(3.5, 3, 360, 360), 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // seenEndTag = vision.seenTagId(endTagId, 0);

    pathCommand.withName("DriveToPose").schedule();

    Logger.recordOutput("TEST/score back", scoreBack);
    Logger.recordOutput(
        "TEST/target dist",
        drive.getPose().getTranslation().getDistance(targetPose.getTranslation()));
    Logger.recordOutput("Auto Lineup/Seen Tag", seenEndTag);
    Logger.recordOutput("Auto Lineup/Tag ID", endTagId);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
    drive.setPathFinished(true);
    pathCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    switch (Constants.currentMode) {
      case REAL:
        return pathCommand.isFinished();
      case SIM:
        return pathCommand.isFinished();
      case REPLAY:
        return true;
    }
    return true;
  }
}
