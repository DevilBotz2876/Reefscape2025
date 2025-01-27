package frc.robot.io.interfaces;

import org.littletonrobotics.junction.AutoLog;

/**
 * This is a generic definition for the Motor IO interface. All motors (SparkMax, Talon, Kraken,
 * etc) are expected to have this basic functionality
 */
public interface MotorIO {

  /* MotorIOInputs defines the status/sensor values that are typically reported by a motor */
  @AutoLog
  public class MotorIOInputs {
    public double appliedVolts = 0.0; // how many volts are actually being applied (in volts)
    public double currentAmps = 0.0; // how much current is being used (in amps)
    public double velocityRadPerSec = 0.0; // how fast the motor shaft is spinning (in radians/sec)
    public double positionRad = 0.0; // the current position of the motor shaft (in radians)
  }

  /**
   * Updates the set of loggable inputs. Called by periodically (e.g. in the subsystem's periodic()
   * function)
   */
  public default void updateInputs(MotorIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /**
   * Optional: Run closed loop (using a PID) at a specific velocity
   *
   * @param velocityRadPerSec desired velocity
   * @param ffVolts feedforward voltage which depends on the static friction of what is connected to
   *     the motor
   * @return true if the motor is capable of automatically maintaining a velocity
   */
  public default boolean setVelocity(double velocityRadPerSec, double ffVolts) {
    return false;
  }

  /**
   * Optional: Run closed loop (using a PID) to reach a specific position
   *
   * @param positionRad desired position
   * @return true if the motor is capable of automatically moving and staying at a specific position
   */
  public default boolean setPosition(double positionRad) {
    return false;
  }
}
