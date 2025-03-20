package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private ClimberIO io;
  private ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private final Alert climberDisconnectedAlert;

  public Climber(ClimberIO io) {
    this.io = io;
    climberDisconnectedAlert = new Alert("Climber motor disconnected", AlertType.kError);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber/Io", inputs);
    climberDisconnectedAlert.set(!inputs.climberMotorConnected);
  }

  public void setOutput(double speed) {
    io.setOutput(speed);
  }

  public void setClimberSetpoint(double position) {
    io.setSetpoint(position);
  }

  public boolean climberAtSetpoint(double tolerance) {
    if (Math.abs(inputs.climberAngle - inputs.climberSetpoint) > tolerance) return false;
    else return true;
  }
}
