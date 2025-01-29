package frc.robot.subsystems.endEffector;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class EndEffectorIOReal implements EndEffectorIO {

  private SparkMax spark;


  public EndEffectorIOReal(){
    endEffectorMotor = new SparkMax(Constants.END_EFFECTOR_MOTOR_ID, MotorType.kBrushless)

var endEffectorconfig = new SparkMaxConfig();
endEffectorConfig
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(driveMotorCurrentLeft)
    .voltageCompensation(12.0)
    .appliedOutputPeriodMs(20)
    .busVoltagePeriodMs(20)
    .outputCurrentPeriodMs(20);

    tryUntilOk(endEffectorMotor, 5, () -> endEffectorMotor.configure(endEffectorConfig, ResetMode.kResetSafeParameters, ResetMode.kPersistParameters))

  }

  public void updateInputs(PivotIOInputs inputs) {



    inputs.endEffectorMotorOneConnected = pivotOneStatus.getStickyFaults();
    inputs.endEffectorStatorCurrent = endEffectorMotor.getOutputCurrent();
    inputs.endEffectorSpeed = endEffectorMotor.get();
    inputs.endEffectorVoltage = pivotMotor.getBusVoltage();
    inputs.endEffectorMotorOneTemp = endEffectorMotor.getMotorTemperature();

  }

  public void setEndEffectorSpeed(double speed) {
    endEffectorMotor.set(speed);

  }

  public void stopEndEffector(){
    setEndEffectorSpeed(0);
  }

  public void setPivotVoltage(double voltage) {
    endEffectorMotor.setVoltage(voltage);
  }
}
