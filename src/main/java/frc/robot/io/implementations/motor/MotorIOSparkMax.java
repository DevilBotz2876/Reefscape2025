package frc.robot.io.implementations.motor;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import frc.robot.io.interfaces.MotorIO;

/** A "SparkMax" controller implementation of a MotorIO */
public class MotorIOSparkMax implements MotorIO {
  // define the 1 SparkMax Controllers
  private final SparkMax motor;

  // Gets the NEO encoder
  private final RelativeEncoder encoder;

  SparkMaxConfig motorConfig = new SparkMaxConfig();

  public MotorIOSparkMax(int id) {
    motor = new SparkMax(id, MotorType.kBrushless);
    // Set motor to brake mode so shooter stops spinning immediately

    motorConfig
        .inverted(false)
        .smartCurrentLimit(60, 40)
        .secondaryCurrentLimit(20000)
        .idleMode(SparkBaseConfig.IdleMode.kBrake);

    // Last thing we do is save all settings to flash on sparkmax
    motor.configure(
        motorConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
    encoder = motor.getEncoder();
  }

  @Override
  public void updateInputs(MotorIOInputs inputs) {
    // Get applied voltage from the motor
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();

    // Get applied voltage from the top motor
    inputs.currentAmps = motor.getOutputCurrent();

    // Set velocityRadPerSec to the encoder velocity(rotationsPerMinute) divided by the gear ratio
    // and converted into Radians Per Second
    inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());

    // Set position to the encoder position(rotations) converted into Radians Per Second
    inputs.positionRad = Units.rotationsToRadians(encoder.getPosition());
  }

  @Override
  public void setVoltage(double volts) {
    // Set the voltage output for the top motor
    motor.setVoltage(volts);
  }

  /* TODO: Add setVelocity using SparkMax's Hardware PID */
  /* TODO: Add setPosition using SparkMax's Hardware PID */
}
