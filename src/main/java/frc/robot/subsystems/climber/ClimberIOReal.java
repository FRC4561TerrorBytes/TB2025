// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.PWM;
import frc.robot.Constants;

public class ClimberIOReal implements ClimberIO {

  private PWM climberServo =
      new PWM(Constants.CLIMBER_CHANNEL); // needs to be either pwm or servo need to test either

  /** Creates a new ClimberIOReal. */
  public ClimberIOReal() {
    climberServo.setBoundsMicroseconds(3, 3, 2, 1, 1); // need to be determined
  }

  @Override
  public void setClimberSetPoint(double pos) {
    climberServo.setPosition(pos);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.climberSpeed = climberServo.getSpeed();
    inputs.climberPosition = climberServo.getPosition();
    inputs.climberPWM = climberServo.getPulseTimeMicroseconds();
  }
}
