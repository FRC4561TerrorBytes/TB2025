package frc.robot.subsystems.elevator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer.ElevatorPosition;
import frc.robot.RobotContainer.ScoreLevel;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private ElevatorPosition selectedElevatorPosition = ElevatorPosition.STOW;
  private ElevatorPosition lastElevatorPosition = ElevatorPosition.STOW;

  private ScoreLevel requestedScoreLevel = ScoreLevel.L2;

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

    SmartDashboard.putString("Auto Lineup/Score Level", selectedElevatorPosition.toString());
  }

  public Pose3d getPivotPose() {
    return new Pose3d(
        0.03 - 0.2782,
        0,
        0.37 - 0.095,
        new Rotation3d(0, Units.rotationsToRadians(-inputs.pivotAngle), 0));
  }

  public Pose3d getExtensionPose() {
    return new Pose3d(
        0.03
            - 0.2782
            + Math.cos(Units.rotationsToRadians(-inputs.pivotAngle)) * (inputs.extensionHeight),
        0,
        0.37
            - 0.095
            - Math.sin(Units.rotationsToRadians(-inputs.pivotAngle)) * (inputs.extensionHeight),
        new Rotation3d(0, Units.rotationsToRadians(-inputs.pivotAngle), 0));
  }

  public double getPivotPosition() {
    return inputs.pivotAngle;
  }

  public double getExtensionPosition() {
    return inputs.extensionHeight;
  }

  public void setSetpoint(ElevatorPosition position) {
    lastElevatorPosition = position;
    io.setTargetPosition(position);
  }

  @AutoLogOutput(key = "Elevator/Elevator Position")
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

  public void requestScoreLevel(ScoreLevel level) {
    requestedScoreLevel = level;
  }

  @AutoLogOutput(key = "Elevator/Score Level")
  public ScoreLevel getRequestedScoreLevel() {
    return requestedScoreLevel;
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
