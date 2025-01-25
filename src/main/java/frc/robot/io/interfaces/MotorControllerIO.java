package frc.robot.io.interfaces;

import org.littletonrobotics.junction.AutoLog;

public interface MotorControllerIO {
      @AutoLog
  public class MotorControllerIOInputs {
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double positionRad = 0.0;
    public double current = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(MotorControllerIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Run closed loop at the specified velocity */
  public default void setVelocity(double velocityRadPerSec, double ffVolts) {}

  public default boolean supportsHardwarePid() {
    return false;
  }
}
