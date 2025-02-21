package frc.robot.io.interfaces;

import edu.wpi.first.math.controller.PIDController;
import org.littletonrobotics.junction.AutoLog;

/**
 * This is a generic definition for the Motor IO interface. All motor controllers (SparkMax, Talon,
 * Kraken, etc) are expected to provide this basic functionality
 */
public interface MotorIO {

  public static class MotorIOSettings {
    public boolean inverted =
        false; // Set this based on the desired direction of movement when voltage is positive vs
    // negative
    public double minVoltage = -12.0;
    public double maxVoltage = 12.0;
    public double gearing =
        1.0; // Set this to match any gearboxes attached the shaft *after* the encoder
    public double drumRadiusMeters =
        1.0; // If the motor is connected to a linear system (e.g. elevator), indicate the radius of
    // the spool/gear here
  }

  /* MotorIOInputs defines the status/sensor values that are typically reported by a motor */
  @AutoLog
  public class MotorIOInputs {
    /* Required to be updated by the motor implementation. The Motor IO native units are assumed to be
     *  voltage: volts
     *  amperage: amps
     *  velocity: radians/sec
     *  acceleration: radians/sec^2
     *  position: radians
     */
    public double appliedVolts = 0.0; // how many volts are actually being applied (in volts)
    public double currentAmps = 0.0; // how much current is being used (in amps)
    public double velocityRadPerSec = 0.0; // how fast the motor shaft is spinning (in radians/sec)
    public double accelerationRadPerSecSq =
        0.0; // how fast the motor shaft is accelerating (in radians/sec^2)
    public double positionRad = 0.0; // the current position of the motor shaft (in radians)

    /* The following are conversions for convenience only and will be auto populated if left at zero */
    public double velocityDegreesPerSec =
        0.0; // how fast the motor shaft is spinning (in degrees/sec)
    public double velocityMetersPerSec =
        0.0; // how fast the linear mechanism connected to the motor shaft (in meters/sec)
    public double velocityRPMs =
        0.0; // how fast the linear mechanism connected to the motor shaft (in rotations/minute)
    public double positionDegrees = 0.0; // the current position of the motor shaft (in degrees)
    public double positionMeters =
        0.0; // the current position of the linear mechanism connected to the motor shaft (in
    // meters)

    public boolean forwardLimit = false;
    public boolean reverseLimit = false;
  }

  /**
   * Updates the set of loggable inputs. Called by periodically (e.g. in the subsystem's periodic()
   * function)
   */
  public void updateInputs(MotorIOInputs inputs);

  /**
   * Run open loop at the specified voltage.
   *
   * @param volts desired voltage
   */
  public void setVoltage(double volts);

  /**
   * Optional: Run closed loop (using a PID) at a specific velocity
   *
   * @param velocityRadPerSec desired velocity
   * @param ffVolts feedforward voltage which depends on the static friction of what is connected to
   *     the motor
   * @return true if the motor is capable of automatically maintaining a velocity
   */
  public boolean setVelocity(double velocityRadPerSec, double ffVolts);

  /**
   * Takes in a position in meters or radians and converts it to radians
   *
   * @param position in radians or meters
   * @return position in radians
   */
  public double normalizePositionToRad(double position);

  /**
   * Takes in a position in radians and converts it to meters
   *
   * @param position in radians
   * @return position in meters
   */
  public double normalizePositionToMeters(double positionRad);

  /**
   * Optional: Run closed loop (using a PID) to reach a specific position
   *
   * @param position desired position (in radians or meters if drumRadiusMeters != 0)
   * @param ffVolts feedforward voltage which depends on the static friction of what is connected to
   *     the motor
   * @return true if the motor is capable of automatically moving and staying at a specific position
   */
  public boolean setPosition(double positionRad, double ffVolts);

  /**
   * Returns the current PID Controller object for this motor
   *
   * @return
   */
  public PIDController getPid();

  /**
   * Resets the relative encoder to the specified position
   *
   * @position the current position in radians
   */
  public void resetEncoder(double positionRad);

  /** Disables the software PID loop */
  public void disablePid();
}
