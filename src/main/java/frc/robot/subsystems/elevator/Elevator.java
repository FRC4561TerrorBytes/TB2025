// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private ElevatorIO io;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private final Alert pivotMotorOneDisconnectedAlert;
  private final Alert pivotMotorTwoDisconnectedAlert;
  private final Alert pivotMotorThreeDisconnectedAlert;
  private final Alert pivotMotorFourDisconnectedAlert;
  private final Alert extensionMotorDisconnectedAlert;

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
  }

  public void setSetpoint(double extensionSetpoint, double pivotSetpoint) {
    io.setTargetPosition(extensionSetpoint, pivotSetpoint);
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
}
