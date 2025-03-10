package frc.robot.io.implementations.motor;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

    // // Peak output of 12 V
    toConfigure
        .Voltage
        .withPeakForwardVoltage(Units.Volts.of(12))
        .withPeakReverseVoltage(Units.Volts.of(-12));

    toConfigure.CurrentLimits = currentLimitsConfigs;

    toConfigure.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    if (motorSettings.motor.inverted) {
      toConfigure.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    }
    toConfigure.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    motorFx.getConfigurator().apply(toConfigure);
  }

  @Override
  public void setVoltage(double volts) {
    // Inversion is handled in the Talon FX Controller config, always set it to false
    motorFx.setVoltage(calculateSafeVoltage(volts, false));
  }

  @Override
  public void updateInputs(MotorIOInputs inputs) {
    inputs.appliedVolts = motorFx.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = motorFx.getSupplyCurrent().getValueAsDouble();
    // inputs.currentStatorAmps = motorFx.getStatorCurrent().getValueAsDouble();

    inputs.positionRad =
        edu.wpi.first.math.util.Units.rotationsToRadians(
            motorFx.getPosition().getValueAsDouble() / motorSettings.motor.gearing);
    inputs.velocityRadPerSec =
        edu.wpi.first.math.util.Units.rotationsToRadians(
            motorFx.getVelocity().getValueAsDouble() / motorSettings.motor.gearing);

    super.updateInputs(inputs);
  }

  @Override
  public void resetEncoder(double positionRad) {
    motorFx.setPosition(
        edu.wpi.first.math.util.Units.radiansToRotations(
            positionRad * motorSettings.motor.gearing));
  }
}
