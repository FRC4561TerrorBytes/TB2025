package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer.ElevatorPosition;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

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
              new Rotation3d(0, Units.degreesToRadians(inputs.pivotAngle), 0)),
          new Pose3d(
              0.03
                  - 0.2782
                  + Math.cos(Units.degreesToRadians(inputs.pivotAngle)) * (inputs.extensionHeight),
              0,
              0.37
                  - 0.095
                  - Math.sin(Units.degreesToRadians(inputs.pivotAngle)) * (inputs.extensionHeight),
              new Rotation3d(0, Units.degreesToRadians(inputs.pivotAngle), 0))
        });
  }

  public void setSetpoint(ElevatorPosition position) {
    io.setTargetPosition(position);
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
}
