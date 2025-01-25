package frc.robot.subsystems.interfaces;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface Elevator extends Subsystem {
  public static class Constants {
    public static double maxPositionInMeters = 1.25;
    public static double minPositionInMeters = 0.0;

    public static double maxVelocityInDegreesPerSecond = 45;
    public static double maxAccelerationInDegreesPerSecondSquared = 120;

    public static double pidKp = 0.1;
    public static double pidKi = 0.0;
    public static double pidKd = 0.0;
    public static double pidTimeoutInSeconds = 3.0;

    public static double ffKs = 0.0;
    public static double ffKv = 0.0;
    public static double ffKa = 0.0;
    public static double ffKg = 0.1;
  }

  /** runs the elevator at a voltage. */
  public void runVoltage(double volts);

  /** Returns the current postition of the elevator in meters. */
  public double getPosition();

  /** Returns the target postition of the elevator in meters. */
  public double getTargetPosition();

  /** Returns true if the elevator is at the maximum limit. */
  public boolean isAtUpperLimit();

  /** Returns true if the elevator is at the minimum limit. */
  public boolean isAtLowerLimit();

  /** Returns velocity in meters per second of the elevator. */
  public double getVelocityPerSecond();

  /** Set the postition of the elevator in meters. */
  public void setPosition(double meters);

  /** Returns true if the elevator is at the setpoint. */
  public boolean isAtSetpoint();

  public default void add2dSim(Mechanism2d mech2d) {}
}
