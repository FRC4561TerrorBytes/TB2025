// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
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

  /** Creates a new DriveToPose. */
  public DriveToPose(Drive drive, Vision vision, int endTagId) {
    this.drive = drive;
    this.vision = vision;
    this.endTagId = endTagId;

    addRequirements(vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.stop();
    seenEndTag = false;

    pathCommand =
        AutoBuilder.pathfindToPose(drive.getSelectedPose(), new PathConstraints(2.5, 3, 360, 360));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    seenEndTag = vision.seenTagId(endTagId);

    pathCommand.schedule();
    Logger.recordOutput("Auto Lineup/Seen Tag", seenEndTag);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pathCommand.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    switch (Constants.currentMode) {
      case REAL:
        return pathCommand.isFinished() || (seenEndTag && vision.getDistanceToTag(1) < 1);
      case SIM:
        return pathCommand.isFinished() || seenEndTag;
      case REPLAY:
        return true;
    }
    return true;
  }
}
