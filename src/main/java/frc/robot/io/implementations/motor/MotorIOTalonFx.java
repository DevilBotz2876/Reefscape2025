package frc.robot.io.implementations.motor;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;

public class MotorIOTalonFx extends MotorIOBase {
  public static class TalonFxSettings {
    public int canId = 0;
  }

  MotorIOBaseSettings motorSettings;

  private final TalonFX motorFx;

  public MotorIOTalonFx(MotorIOBaseSettings motorSettings, TalonFxSettings talonFxSettings) {
    super(motorSettings);
    this.motorSettings = motorSettings;
    motorFx = new TalonFX(talonFxSettings.canId);
    motorFx.setNeutralMode(NeutralModeValue.Brake);

    TalonFXConfiguration toConfigure = new TalonFXConfiguration();
    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();

    /* Configure the Talon to use a supply limit of 70 A, and
     * lower to 40 A if we're at 70 A for over 1 second */

    currentLimitsConfigs
        .withSupplyCurrentLowerLimit(Units.Amps.of(70)) // Default limit of 70 A
        .withSupplyCurrentLimit(
            Units.Amps.of(40)) // Reduce the limit to 40 A if we've limited to 70 A...
        .withSupplyCurrentLowerTime(Units.Seconds.of(1.0)) // ...for at least 1 second
        .withSupplyCurrentLimitEnable(true); // And enable it

    currentLimitsConfigs
        .withStatorCurrentLimit(Units.Amps.of(120)) // Limit stator current to 120 A
        .withStatorCurrentLimitEnable(true); // And enable it

    // // Peak output of 8 V
    toConfigure
        .Voltage
        .withPeakForwardVoltage(Units.Volts.of(8))
        .withPeakReverseVoltage(Units.Volts.of(-8));

    toConfigure.CurrentLimits = currentLimitsConfigs;

    toConfigure.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    if (motorSettings.motor.inverted) {
      toConfigure.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    }

    motorFx.getConfigurator().apply(toConfigure);
    // motorFx.setpo(0);
    // motorFx.setPosition(null)
  }

  @Override
  public void setVoltage(double volts) {
    volts = MathUtil.clamp(volts, -12, 12);
    motorFx.setVoltage(volts);
  }

  @Override
  public void updateInputs(MotorIOInputs inputs) {
    inputs.appliedVolts = motorFx.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = motorFx.getSupplyCurrent().getValueAsDouble();
    // inputs.currentStatorAmps = motorFx.getStatorCurrent().getValueAsDouble();
    inputs.positionRad =
        edu.wpi.first.math.util.Units.rotationsToRadians(
            motorFx.getPosition().getValueAsDouble() / motorSettings.motor.gearing);

    super.updateInputs(inputs);
  }

  @Override
  public void resetEncoder(double positionRad) {
    motorFx.setPosition(edu.wpi.first.math.util.Units.radiansToRotations(positionRad));
  }
}
