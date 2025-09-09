// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer.ElevatorPosition;
import frc.robot.RobotContainer.ScoreLevel;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants.Reef;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPose extends Command {

  private Drive drive;
  private Elevator elevator;
  private Wrist wrist;
  private int endTagId;
  private boolean seenEndTag;
  private Command pathCommand;

  private Pose2d targetPose;
  private boolean scoreBack = true;

  ProfiledPIDController xController =
      new ProfiledPIDController(15, 0, 0, new TrapezoidProfile.Constraints(2, 2));
  ProfiledPIDController yController =
      new ProfiledPIDController(15, 0, 0, new TrapezoidProfile.Constraints(2, 2));
  ProfiledPIDController thetaController =
      new ProfiledPIDController(
          15, 0, 0, new TrapezoidProfile.Constraints(Math.PI, Math.PI));

  /** Creates a new DriveToPose. */
  public DriveToPose(Drive drive, Elevator elevator, Wrist wrist) {
    this.drive = drive;
    this.elevator = elevator;
    this.wrist = wrist;

    addRequirements(drive, elevator, wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.stop();
    drive.setPathFinished(false);

    targetPose = drive.getFinalTargetPose();
    Pose2d centerRobot = drive.getPose();
    Transform2d forwardOffset = new Transform2d(new Translation2d(-0.38, 0.0), new Rotation2d());
    Pose2d frontRobot = centerRobot.transformBy(forwardOffset);

    double centerDistance =
        centerRobot.getTranslation().getDistance(AllianceFlipUtil.apply(Reef.center));
    double backDistance =
        frontRobot.getTranslation().getDistance(AllianceFlipUtil.apply(Reef.center));

    // SCORE OUT BACK
    if (centerDistance > 1.6) { // only moving arm if far enough away from reef
      if (centerDistance <= backDistance) {
        // SCORE L2
        if (elevator.getRequestedScoreLevel().equals(ScoreLevel.L2)) {
          elevator.setSetpoint(ElevatorPosition.L2FRONT);
        }
        // SCORE L3
        else if (elevator.getRequestedScoreLevel().equals(ScoreLevel.L3)) {
          elevator.setSetpoint(ElevatorPosition.L3FRONT);
        }
      }
      // SCORE OUT FRONT
      else {
        // SCORE L2
        if (elevator.getRequestedScoreLevel().equals(ScoreLevel.L2)) {
          elevator.setSetpoint(ElevatorPosition.L2BACK);
        }
        // SCORE L3
        else if (elevator.getRequestedScoreLevel().equals(ScoreLevel.L3)) {
          elevator.setSetpoint(ElevatorPosition.L3BACK);
        }
      }
    }

    Logger.recordOutput("Auto Lineup/Target Pose", targetPose);
    Logger.recordOutput("Auto Lineup/CenterDistanceAway", centerDistance);

    if (centerDistance > 2) {
      pathCommand =
          AutoBuilder.pathfindToPose(targetPose, new PathConstraints(3.5, 3, Math.PI, Math.PI), 0);

      pathCommand.withName("DriveToPose").schedule();
    } else {
      pathCommand = null;

      xController.setGoal(targetPose.getX());
      xController.setTolerance(0.07);
      xController.reset(drive.getPose().getX());
      yController.setGoal(targetPose.getY());
      yController.setTolerance(0.07);
      yController.reset(drive.getPose().getY());
      thetaController.setGoal(targetPose.getRotation().getRadians());
      thetaController.setTolerance(Units.degreesToRadians(2));
      thetaController.reset(drive.getPose().getRotation().getRadians());
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (pathCommand == null) {
      double xSpeed = xController.calculate(drive.getPose().getX());
      double ySpeed = yController.calculate(drive.getPose().getY());
      thetaController.enableContinuousInput(-Math.PI, Math.PI);
      double rotSpeed = thetaController.calculate(drive.getPose().getRotation().getRadians());

      Logger.recordOutput("Auto Lineup/RunXVelocity", xSpeed);
      Logger.recordOutput("Auto Lineup/TargetPoseX", targetPose.getX());
      Logger.recordOutput("Auto Lineup/RunYVelocity", ySpeed);
      Logger.recordOutput("Auto Lineup/TargetPoseY", targetPose.getY());
      Logger.recordOutput("Auto Lineup/RunThetaVelocity", rotSpeed);
      //Hi Ethan :)

      // feed to chassis
      ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, drive.getPose().getRotation());
      drive.runVelocity(speeds);
    }
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
    if (pathCommand != null) {
      pathCommand.end(interrupted);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (pathCommand != null) {
      switch (Constants.currentMode) {
        case REAL:
          return pathCommand.isFinished();
        case SIM:
          return pathCommand.isFinished();
        case REPLAY:
          return true;
        default:
          return true;
      }
    } else {
      return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
    }
  }
}
