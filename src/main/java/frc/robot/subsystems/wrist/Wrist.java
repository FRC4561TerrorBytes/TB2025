package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer.ElevatorPosition;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private WristIO io;
  private WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
  private final Alert WristDisconnectedAlert;

  public Wrist(WristIO io) {
    this.io = io;
    WristDisconnectedAlert = new Alert("Wrist motor disconnected", AlertType.kError);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Wrist/Io", inputs);
    WristDisconnectedAlert.set(!inputs.wristMotorConnected);
  }

  public void setOutput(double speed) {
    io.setOutput(speed);
  }

  public void setSetpoint(ElevatorPosition position) {
    io.setSetpoint(position.wristPosition);
  }

  public boolean wristAtSetpoint(double tolerance) {
    if (Math.abs(inputs.wristAngle - inputs.wristSetpoint) > tolerance) return false;
    else return true;
  }
}
