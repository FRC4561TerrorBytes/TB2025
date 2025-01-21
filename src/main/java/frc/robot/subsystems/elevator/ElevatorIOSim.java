// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOSim implements ElevatorIO {
  /** Creates a new ElevatorIOSim. */
  private static final double LOOP_PERIOD_SECS = 0.02;

  private static final double PIVOT_KP = 12;
  private static final double PIVOT_KD = 14;

  private static final double EXTENSION_KP = 12;
  private static final double EXTENSION_KD = 3;

  private static final DCMotor PIVOT_MOTOR = DCMotor.getFalcon500(4);
  private static final DCMotor EXTENSION_MOTOR = DCMotor.getKrakenX60(1);

  private DCMotorSim pivotMotorOneSim;
  private DCMotorSim pivotMotorTwoSim;
  private DCMotorSim pivotMotorThreeSim;
  private DCMotorSim pivotMotorFourSim;
  private DCMotorSim extensionMotorSim;

  private boolean pivotClosedLoop = false;
  private boolean extensionClosedLoop = false;
  private PIDController pivotController = new PIDController(PIVOT_KP, 0, PIVOT_KD);
  private PIDController extensionController = new PIDController(EXTENSION_KP, 0, EXTENSION_KD);
  private double pivotAppliedVolts = 0.0;
  private double extensionAppliedVolts = 0.0;

  private double pivotSetpoint = 0.0;
  private double extensionSetpoint = 0.0;

  public ElevatorIOSim() {
    pivotMotorOneSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                PIVOT_MOTOR, 0.001, 1.0 / Constants.PIVOT_GEAR_RATIO),
            PIVOT_MOTOR);

    pivotMotorTwoSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                PIVOT_MOTOR, 0.001, 1.0 / Constants.PIVOT_GEAR_RATIO),
            PIVOT_MOTOR);

    pivotMotorThreeSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                PIVOT_MOTOR, 0.001, 1.0 / Constants.PIVOT_GEAR_RATIO),
            PIVOT_MOTOR);

    pivotMotorFourSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                PIVOT_MOTOR, 0.001, 1.0 / Constants.PIVOT_GEAR_RATIO),
            PIVOT_MOTOR);

    extensionMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                EXTENSION_MOTOR, 0.001, Constants.EXTENSION_GEAR_RATIO),
            EXTENSION_MOTOR);
    setTargetPosition(0.5, -70);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (pivotClosedLoop) {
      pivotAppliedVolts = pivotController.calculate(inputs.pivotAngle, pivotSetpoint);
    } else {
      pivotController.reset();
    }

    if (extensionClosedLoop) {
      extensionAppliedVolts =
          extensionController.calculate(inputs.extensionHeight, extensionSetpoint);
    } else {
      extensionController.reset();
    }

    pivotMotorOneSim.setInputVoltage(MathUtil.clamp(pivotAppliedVolts, -12.0, 12.0));
    pivotMotorTwoSim.setInputVoltage(MathUtil.clamp(pivotAppliedVolts, -12.0, 12.0));
    pivotMotorThreeSim.setInputVoltage(MathUtil.clamp(pivotAppliedVolts, -12.0, 12.0));
    pivotMotorFourSim.setInputVoltage(MathUtil.clamp(pivotAppliedVolts, -12.0, 12.0));
    extensionMotorSim.setInputVoltage(MathUtil.clamp(extensionAppliedVolts, -12.0, 12.0));

    pivotMotorOneSim.update(LOOP_PERIOD_SECS);
    pivotMotorTwoSim.update(LOOP_PERIOD_SECS);
    pivotMotorThreeSim.update(LOOP_PERIOD_SECS);
    pivotMotorFourSim.update(LOOP_PERIOD_SECS);
    extensionMotorSim.update(LOOP_PERIOD_SECS);

    inputs.pivotAngle =
        Units.rotationsToDegrees(
            pivotMotorOneSim.getAngularPositionRotations() / Constants.PIVOT_GEAR_RATIO);
    inputs.extensionHeight =
        extensionMotorSim.getAngularPositionRotations() / Constants.EXTENSION_GEAR_RATIO;
    Logger.recordOutput(
        "FinalComponentPoses",
        new Pose3d[] {
          new Pose3d(),
          new Pose3d(
              0.05 - 0.2964,
              0,
              0.38 - 0.092,
              new Rotation3d(0, Units.degreesToRadians(inputs.pivotAngle), 0)),
          new Pose3d(
              -0.5
                  + 0.27
                  + Math.cos(Units.degreesToRadians(inputs.pivotAngle)) * inputs.extensionHeight,
              0,
              -0.37
                  + 0.64
                  - Math.sin(Units.degreesToRadians(inputs.pivotAngle)) * inputs.extensionHeight,
              new Rotation3d(0, Units.degreesToRadians(inputs.pivotAngle), 0))
        });
    // json was -0.05, 0, -0.38
    inputs.pivotMotorOneConnected = true;
    inputs.pivotMotorTwoConnected = true;
    inputs.pivotMotorThreeConnected = true;
    inputs.pivotMotorFourConnected = true;
    inputs.extensionMotorConnected = true;
    inputs.pivotSetpoint = this.pivotSetpoint;
    inputs.extensionSetpoint = this.extensionSetpoint;
    inputs.pivotVoltage = pivotAppliedVolts;
    inputs.extensionVoltage = extensionAppliedVolts;

    // this is for testing remove later
  }

  @Override
  public void setTargetPosition(double extenstionPosition, double pivotPosition) {
    extensionClosedLoop = true;
    extensionSetpoint = extenstionPosition;
    pivotClosedLoop = true;
    pivotSetpoint = pivotPosition;
  }

  @Override
  public void setExtensionVoltage(double voltage) {
    extensionClosedLoop = false;
    extensionAppliedVolts = voltage;
  }

  @Override
  public void setPivotVoltage(double voltage) {
    pivotClosedLoop = false;
    pivotAppliedVolts = voltage;
  }
}
