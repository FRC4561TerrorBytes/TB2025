package frc.robot.subsystems.endEffector;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends SubsystemBase {
  private EndEffectorIO io;
  private EndEffectorIOInputsAutoLogged inputs = new EndEffectorIOInputsAutoLogged();

  public EndEffector(EndEffectorIO io) {
    this.io = io;


  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("EndEffector/IO", inputs);
  }

  public void setEndEffectorSpeed(double speed) {
    io.setEndEffectorSpeed(speed);
  }

  public void setEndEffectorVoltage(double voltage) {
    io.setEndEffectorVoltage(voltage);
  }

 
}
