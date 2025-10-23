// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class WristIOSim implements WristIO {
  private static final double LOOP_PERIOD_SECS = 0.02;

  private static final double WRIST_KP = 1.8;
  private static final double WRIST_KD = 0;

  private static final DCMotor WRIST_MOTOR = DCMotor.getKrakenX60(1);

  private DCMotorSim wristMotorSim;

  private boolean closedLoop = true;
  private ProfiledPIDController wristController =
      new ProfiledPIDController(WRIST_KP, 0.0, WRIST_KD, new Constraints(3, 3));

  private double wristSetpoint;
  private double wristAppliedVolts;

  public WristIOSim() {
    wristMotorSim =
        new DCMotorSim(LinearSystemId.createDCMotorSystem(WRIST_MOTOR, 0.000001, 1), WRIST_MOTOR);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    if (closedLoop) {
      wristAppliedVolts = wristController.calculate(inputs.wristAngle, wristSetpoint);
    }

    wristMotorSim.setInputVoltage(MathUtil.clamp(wristAppliedVolts, -12.0, 12.0));
    wristMotorSim.update(LOOP_PERIOD_SECS);

    inputs.wristAngle = wristMotorSim.getAngularPositionRotations() / 1.0;
    inputs.wristMotorConnected = true;
    inputs.wristSetpoint = this.wristSetpoint;
    inputs.wristVoltage = this.wristAppliedVolts;
  }

  @Override
  public void setSetpoint(double position) {
    closedLoop = true;
    this.wristSetpoint = Units.degreesToRotations(position);
  }

  @Override
  public void setOutput(double voltage) {
    closedLoop = false;
    wristAppliedVolts = voltage;
  }
}
