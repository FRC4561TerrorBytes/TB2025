package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {

  private static final double LOOP_PERIOD_SECS = 0.02;

  private double intakeAppliedVolts = 0.0;

  private DCMotorSim intakeMotor =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), 0.01, 1.0),
          DCMotor.getNeo550(1));

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    intakeMotor.update(LOOP_PERIOD_SECS);
    inputs.intakeVelocity = Units.radiansToDegrees(intakeMotor.getAngularVelocityRadPerSec());
    inputs.intakeVoltage = intakeAppliedVolts;
    inputs.intakeCurrentAmps = Math.abs(intakeMotor.getCurrentDrawAmps());
  }

  public void setOutput(double speed) {
    intakeAppliedVolts = MathUtil.clamp(speed * 12, -12, 12);
    intakeMotor.setInputVoltage(intakeAppliedVolts);
  }
}
