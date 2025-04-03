// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer.ReefScorePositions;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPose extends Command {

  private Drive drive;
  private Vision vision;
  private int endTagId;
  private boolean seenEndTag;
  private Command pathCommand;

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
    seenEndTag = false;
    endTagId = drive.getSelectedScorePosition().aprilTagID;

    if (AllianceFlipUtil.shouldFlip()) {
      if (drive.getSelectedScorePosition().equals(ReefScorePositions.FRONT)
          || drive.getSelectedScorePosition().equals(ReefScorePositions.BACK)
          || drive.getSelectedScorePosition().equals(ReefScorePositions.LEFTSOURCE)
          || drive.getSelectedScorePosition().equals(ReefScorePositions.RIGHTSOURCE)) {
        endTagId -= 11;
      } else if (drive.getSelectedScorePosition().equals(ReefScorePositions.BACKLEFT)
          || drive.getSelectedScorePosition().equals(ReefScorePositions.FRONTRIGHT)) {
        endTagId -= 9;
      } else if (drive.getSelectedScorePosition().equals(ReefScorePositions.BACKRIGHT)
          || drive.getSelectedScorePosition().equals(ReefScorePositions.FRONTLEFT)
          || drive.getSelectedScorePosition().equals(ReefScorePositions.PROCESSER)) {
        endTagId -= 13;
      }
    }

    Pose2d targetPosition = drive.getSelectedPose();

    Logger.recordOutput("Auto Lineup/Target Pose", targetPosition);

    pathCommand =
        AutoBuilder.pathfindToPose(targetPosition, new PathConstraints(4.7, 3.5, 360, 360));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    seenEndTag = vision.seenTagId(endTagId, 0);

    pathCommand.withName("DriveToPose").schedule();

    Logger.recordOutput("Auto Lineup/Seen Tag", seenEndTag);
    Logger.recordOutput("Auto Lineup/Tag ID", endTagId);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pathCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    switch (Constants.currentMode) {
      case REAL:
        return pathCommand.isFinished() || (seenEndTag && vision.getDistanceToTag(0) < 1);
      case SIM:
        return pathCommand.isFinished();
      case REPLAY:
        return true;
    }
    return true;
  }
}
