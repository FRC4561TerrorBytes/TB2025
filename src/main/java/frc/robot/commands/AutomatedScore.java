// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer.ElevatorPosition;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.vision.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutomatedScore extends Command {

  private Elevator elevator;
  private Vision vision;
  // private Intake intake;
  private ElevatorPosition scoringPosition;

  /** Creates a new ElevatorScoringPosition. */
  public AutomatedScore(Elevator elevator, Vision vision) {
    this.elevator = elevator;
    this.vision = vision;

    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    scoringPosition = elevator.getRequestedElevatorPosition();

    elevator.setSetpoint(scoringPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (vision.scoreReady() && elevator.elevatorAtSetpoint() && elevator.pivotAtSetpoint()) {
      // outtake coral
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.setSetpoint(ElevatorPosition.STOW);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false; // return !intake.coralInRobot();
  }
}
