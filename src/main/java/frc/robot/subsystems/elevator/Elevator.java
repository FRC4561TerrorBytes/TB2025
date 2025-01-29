// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer.ElevatorPosition;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final Alert pivotMotorOneDisconnectedAlert;
  private final Alert pivotMotorTwoDisconnectedAlert;
  private final Alert pivotMotorThreeDisconnectedAlert;
  private final Alert pivotMotorFourDisconnectedAlert;
  private final Alert extensionMotorDisconnectedAlert;

  private ElevatorPosition selectedElevatorPosition = ElevatorPosition.STOW;

  /** Creates a new Elevator. */
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

    SignalLogger.setPath("/media/sda1/");

    pivotMotorOneDisconnectedAlert = new Alert("Pivot Motor One Disconnected", AlertType.kError);

    pivotMotorTwoDisconnectedAlert = new Alert("Pivot Motor Two Disconnected", AlertType.kError);

    pivotMotorThreeDisconnectedAlert =
        new Alert("Pivot Motor Three Disconnected", AlertType.kError);

    pivotMotorFourDisconnectedAlert = new Alert("Pivot Motor Four Disconnected", AlertType.kError);

    extensionMotorDisconnectedAlert = new Alert("Extension Motor Disconnected", AlertType.kError);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Elevator/IO", inputs);

    pivotMotorOneDisconnectedAlert.set(!inputs.pivotMotorOneConnected);
    pivotMotorTwoDisconnectedAlert.set(!inputs.pivotMotorTwoConnected);
    pivotMotorThreeDisconnectedAlert.set(!inputs.pivotMotorThreeConnected);
    pivotMotorFourDisconnectedAlert.set(!inputs.pivotMotorFourConnected);

    extensionMotorDisconnectedAlert.set(!inputs.extensionMotorConnected);

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

    Logger.recordOutput("ZeroesComponentPoses", new Pose3d[] {new Pose3d(), new Pose3d()});
  }

  public void setSetpoint(ElevatorPosition position) {
    io.setTargetPosition(position);
  }

  public void getCurrentSetpoint() {
    io.getCurrentSetpoint();
  }

  public void setPivotVoltage(double voltage) {
    io.setPivotVoltage(voltage);
  }

  public double getPivotAngle() {
    return inputs.pivotAngle;
  }

  public void setExtensionVoltage(double voltage) {
    io.setExtensionVoltage(voltage);
  }

  public double getExtensionHeight() {
    return inputs.extensionHeight;
  }

  public void requestElevatorPosition(ElevatorPosition position) {
    selectedElevatorPosition = position;
  }

  @AutoLogOutput(key = "Elevator/Selected Position")
  public ElevatorPosition getRequestedElevatorPosition() {
    return selectedElevatorPosition;
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
