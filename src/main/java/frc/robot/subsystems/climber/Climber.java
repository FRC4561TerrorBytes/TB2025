// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private ClimberIO io;

  public Climber(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber/IO", inputs);
  }

  public void setClimberSpeed(double speed) {
    io.setClimberSpeed(speed);
  }

  public void stopClimber() {
    io.stopClimber();
  }

  // This method will be called once per scheduler run
}
