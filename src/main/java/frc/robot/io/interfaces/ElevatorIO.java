package frc.robot.io.interfaces;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double positionMeters = 0.0;
    public double velocityInMetersPerSecond = 0.0;

    public boolean upperLimit = false;
    public boolean lowerLimit = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {}

  /** Run the elevator motor(s) at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** sets of the position of the elevator in meters */
  public default void setPosition(double meters, double ffVolts) {}
}
