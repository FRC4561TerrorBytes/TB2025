package frc.robot.subsystems.pivot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer.ElevatorPosition;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  private PivotIO io;
  private PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
  private final Alert pivotDisconnectedAlert;

  private ElevatorPosition selectedScorePosition = ElevatorPosition.STOW;

  public Pivot(PivotIO io) {
    this.io = io;

    pivotDisconnectedAlert = new Alert("Pivot motor disconnected", AlertType.kError);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Pivot/IO", inputs);
    pivotDisconnectedAlert.set(!inputs.pivotMotorOneConnected);
  }

  public void setPivotPosition(double angle) {
    io.setPivotPosition(angle);
  }

  public void setPivotVoltage(double voltage) {
    io.setPivotVoltage(voltage);
  }

  public double getCurrentSetpoint() {
    return inputs.pivotAngle;
  }

  public void requestScorePosition(ElevatorPosition position) {
    selectedScorePosition = position;
  }

  @AutoLogOutput(key = "Pivot/Selected Position")
  public ElevatorPosition getRequestedScorePosition() {
    return selectedScorePosition;
  }
}
