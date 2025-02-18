package frc.robot.io.implementations.motor;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;

/** A "SparkMax" controller implementation of a MotorIO */
public class MotorIOSparkMax extends MotorIOBase {
  public static class SparkMaxSettings {
    public int canId = 0;
  }

  MotorIOBaseSettings motorSettings;

  // SparkMax Controller
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController pid;
  SparkMaxConfig motorConfig = new SparkMaxConfig();

  DigitalInput reverseLimitSwitch = null;
  DigitalInput forwardLimitSwitch = null;

  public MotorIOSparkMax(MotorIOBaseSettings motorSettings, SparkMaxSettings sparkMaxSettings) {
    super(motorSettings);
    this.motorSettings = motorSettings;

    motor = new SparkMax(sparkMaxSettings.canId, MotorType.kBrushless);
    // Set motor to brake mode so shooter stops spinning immediately

    motorConfig
        .inverted(motorSettings.motor.inverted)
        .smartCurrentLimit(60, 40)
        .secondaryCurrentLimit(0)
        .idleMode(SparkBaseConfig.IdleMode.kBrake);

    motorConfig.closedLoop.pid(
        motorSettings.pid.getP(), motorSettings.pid.getI(), motorSettings.pid.getD());

    // Last thing we do is save all settings to flash on sparkmax
    motor.configure(
        motorConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    encoder = motor.getEncoder();
    pid = motor.getClosedLoopController();

    if (motorSettings.reverseLimitChannel > -1) {
      reverseLimitSwitch = new DigitalInput(motorSettings.reverseLimitChannel);
    }
    if (motorSettings.forwardLimitChannel > -1) {
      forwardLimitSwitch = new DigitalInput(motorSettings.forwardLimitChannel);
    }
  }

  @Override
  public void updateInputs(MotorIOInputs inputs) {
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = motor.getOutputCurrent();
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(
            encoder.getVelocity() / motorSettings.motor.gearing);
    inputs.positionRad =
        Units.rotationsToRadians(encoder.getPosition() / motorSettings.motor.gearing);

    inputs.velocityMetersPerSec = inputs.velocityRadPerSec * motorSettings.motor.drumRadiusMeters;
    inputs.positionMeters = inputs.positionRad * motorSettings.motor.drumRadiusMeters;
    inputs.forwardLimit = getForwardLimitSwitch();
    inputs.reverseLimit = getReverseLimitSwitch();
    super.updateInputs(inputs);
  }

  @Override
  public void setVoltage(double volts) {
    // Set the voltage output for the top motor
    motor.setVoltage(calculateSafeVoltage(volts));
  }

  @Override
  public boolean setVelocity(double velocityRadPerSec, double ffVolts) {
    REVLibError result;

    result =
        pid.setReference(
            Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec),
            ControlType.kMAXMotionVelocityControl,
            ClosedLoopSlot.kSlot0,
            ffVolts);
    return result == REVLibError.kOk;
  }

  @Override
  public boolean setPosition(double positionRad, double ffVolts) {
    REVLibError result;
    result =
        pid.setReference(
            Units.radiansToRotations(positionRad),
            ControlType.kMAXMotionPositionControl,
            ClosedLoopSlot.kSlot0,
            ffVolts);
    return result == REVLibError.kOk;
  }

  @Override
  public void resetEncoder(double positionRad) {
    encoder.setPosition(Units.radiansToRotations(positionRad));
  }

  private boolean getForwardLimitSwitch() {
    if (forwardLimitSwitch == null) {
      return false;
    }
    // TODO: check if fwd limit switch is configured/plugged directly into spark max
    // motor.getForwardLimitSwitch()
    boolean limit = forwardLimitSwitch.get();
    if (motorSettings.forwardLimitNegate) {
      return !limit;
    }
    return limit;
  }

  private boolean getReverseLimitSwitch() {
    if (reverseLimitSwitch == null) {
      return false;
    }
    // TODO: check if reverse limit switch is configured/plugged directly into spark max
    // motor.getReverseLimitSwitch()
    boolean limit = reverseLimitSwitch.get();
    if (motorSettings.reverseLimitNegate) {
      return !limit;
    }
    return limit;
  }
}
