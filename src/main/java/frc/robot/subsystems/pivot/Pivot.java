package frc.robot.subsystems.pivot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer.ElevatorPosition;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  private PivotIO io;
  private PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

  private ElevatorPosition selectedScorePosition = ElevatorPosition.STOW;

  public Pivot(PivotIO io) {
    this.io = io;

    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        break;
      case SIM:
        break;
      default:
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Pivot/IO", inputs);
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
