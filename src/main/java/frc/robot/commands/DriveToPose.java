// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer.ElevatorPosition;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.wrist.Wrist;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPose extends Command {

  private Drive drive;
  private Vision vision;
  private int endTagId;
  private boolean seenEndTag;
  private Command pathCommand;
  private Elevator elevator;
  private Wrist wrist;

  private Pose2d targetPose;
  private boolean scoreBack = true;
  private double distanceAway = Units.inchesToMeters(-25.654);

  /** Creates a new DriveToPose. */
  public DriveToPose(Drive drive, Vision vision, Elevator elevator, Wrist wrist) {
    this.drive = drive;
    this.vision = vision;
    this.elevator = elevator;
    this.wrist = wrist;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.stop();

    Pose2d selectedPosition = drive.getSelectedPose();

    targetPose =
        new Pose2d(
            Math.cos(selectedPosition.getRotation().getRadians()) * distanceAway
                - Math.sin(selectedPosition.getRotation().getRadians())
                    * drive.getAutoAlignOffsetX()
                + selectedPosition.getTranslation().getX(),
            Math.sin(selectedPosition.getRotation().getRadians()) * distanceAway
                + Math.cos(selectedPosition.getRotation().getRadians())
                    * drive.getAutoAlignOffsetX()
                + selectedPosition.getTranslation().getY(),
            selectedPosition.getRotation());

    if (Math.abs(targetPose.getRotation().getDegrees() - drive.getRotation().getDegrees()) > 90
        && Math.abs(targetPose.getRotation().getDegrees() - drive.getRotation().getDegrees())
            <= 270) {
      targetPose = targetPose.rotateAround(targetPose.getTranslation(), Rotation2d.k180deg);
      scoreBack = false;
    } else {
      scoreBack = true;
    }

    Logger.recordOutput("Auto Lineup/Target Pose", targetPose);

    pathCommand = AutoBuilder.pathfindToPose(targetPose, new PathConstraints(4.7, 3.5, 360, 360));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // seenEndTag = vision.seenTagId(endTagId, 0);

    pathCommand.withName("DriveToPose").schedule();

    Logger.recordOutput("TEST/score back", scoreBack);
    Logger.recordOutput("TEST/elevator thing", elevator.getRequestedElevatorPosition(scoreBack));
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
    pathCommand.end(interrupted);
    setMechanismSetpoint(elevator.getRequestedElevatorPosition(scoreBack));
  }

  private void setMechanismSetpoint(ElevatorPosition position) {
    elevator.setSetpoint(position);
    wrist.setSetpoint(position);
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
