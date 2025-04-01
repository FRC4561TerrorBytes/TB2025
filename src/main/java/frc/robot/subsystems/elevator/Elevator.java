package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer.ElevatorPosition;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private ElevatorPosition selectedElevatorPosition = ElevatorPosition.STOW;
  private ElevatorPosition lastElevatorPosition = ElevatorPosition.STOW;

  public Elevator(ElevatorIO io) {
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

    Logger.recordOutput(
        "FinalComponentPoses",
        new Pose3d[] {
          new Pose3d(
              0.03 - 0.2782,
              0,
              0.37 - 0.095,
              new Rotation3d(0, Units.rotationsToRadians(inputs.pivotAngle), 0)),
          new Pose3d(
              0.03
                  - 0.2782
                  + Math.cos(Units.rotationsToRadians(inputs.pivotAngle))
                      * (inputs.extensionHeight),
              0,
              0.37
                  - 0.095
                  - Math.sin(Units.rotationsToRadians(inputs.pivotAngle))
                      * (inputs.extensionHeight),
              new Rotation3d(0, Units.rotationsToRadians(inputs.pivotAngle), 0))
        });

    SmartDashboard.putString("Auto Lineup/Score Level", selectedElevatorPosition.toString());
  }

  public void setSetpoint(ElevatorPosition position) {
    lastElevatorPosition = position;
    io.setTargetPosition(position);
  }

  public ElevatorPosition getElevatorPosition() {
    return lastElevatorPosition;
  }

  public void setPivotPosition(double angle) {
    io.setPivotPosition(angle);
  }

  public void setPivotVoltage(double voltage) {
    io.setPivotVoltage(voltage);
  }

  public void setExtensionPosition(double angle) {
    io.setExtensionPosition(angle);
  }

  public void setExtensionVoltage(double voltage) {
    io.setExtensionVoltage(voltage);
  }

  public double getCurrentSetpoint() {
    return inputs.pivotAngle;
  }

  public void requestElevatorPosition(ElevatorPosition position) {
    selectedElevatorPosition = position;
  }

  @AutoLogOutput(key = "Elevator/Selected Position")
  public ElevatorPosition getRequestedElevatorPosition(boolean scoreBack) {
    if (!scoreBack) {
      if (selectedElevatorPosition.equals(ElevatorPosition.L2BACKAUTOALIGN)) selectedElevatorPosition = ElevatorPosition.L2FRONTAUTOALIGN;
      else selectedElevatorPosition = ElevatorPosition.L3FRONTAUTOALIGN;
    }

    return selectedElevatorPosition;
  }

  @AutoLogOutput(key = "Elevator/At setpoint")
  public boolean mechanismAtSetpoint() {
    if (Math.abs(inputs.extensionHeight - inputs.extensionSetpoint) <= 0.075
        && Math.abs(inputs.pivotAngle - inputs.pivotSetpoint) <= Units.degreesToRotations(2.0))
      return true;
    else return false;
  }

  @AutoLogOutput(key = "Elevator/Elevator At Setpoint")
  public boolean elevatorAtSetpoint() {
    if (inputs.extensionHeight == inputs.extensionSetpoint) return true;
    else return false;
  }

  @AutoLogOutput(key = "Elevator/Pivot At Setpoint")
  public boolean pivotAtSetpoint() {
    if (inputs.pivotAngle == inputs.pivotSetpoint) return true;
    else return false;
  }
}
