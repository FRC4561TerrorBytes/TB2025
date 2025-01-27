package frc.robot.subsystems.pivot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  private PivotIO io;
  private PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

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
}
