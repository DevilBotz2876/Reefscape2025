package frc.robot.subsystems.interfaces;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public interface Elevator extends Subsystem {
  public static class Constants {
    public static double maxPositionInMeters = 1.25;
    public static double minPositionInMeters = 0.0;

    public static double maxVelocityInDegreesPerSecond = 45;
    public static double maxAccelerationInDegreesPerSecondSquared = 120;

    // PID and FF values from the wpi elevatorsim example
    public static double maxVelocity = 2.45;
    public static double maxAcceleration = 2.45;
    public static double pidDt = 0.02;

    public static double pidKp = 5.0;
    public static double pidKi = 0.0;
    public static double pidKd = 0.0;
    public static double pidTimeoutInSeconds = 3.0;
    public static double pidErrorInMeters = 2.0;

    public static double ffKs = 0.0;
    public static double ffKv = 0.762;
    public static double ffKa = 0.0;
    public static double ffKg = 0.762;
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

  public default void setLigament(MechanismLigament2d ligament2d) {}
  ;

  public default SysIdRoutine createSystemIdRoutine(String subsystemName) {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            null, null, null, (state) -> Logger.recordOutput(subsystemName, state.toString())),
        new SysIdRoutine.Mechanism((voltage) -> runVoltage(voltage.in(Volts)), null, this));
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public default Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return new InstantCommand();
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public default Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return new InstantCommand();
  }
}
