package frc.robot.subsystems.interfaces;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;

public interface ClimberPrototype {
  public static class Constants {

  }

  /**
   * Set the voltage of motors for the shooter.
   *
   * @param volts The volts to set. Value should be between -12.0 and 12.0.
   */

  public default void setVolts(double Volts) {}

  public double getSpeed();

  public default void turnOff() {
    setVolts(0);
  }

  public Command getTurnOffCommand();
  
  public default void add2dSim(Mechanism2d mech2d) {}
  ;
}
