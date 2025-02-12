package frc.robot.subsystems.interfaces;

public interface Motor {
  /**
   * Runs the motor at the specified voltage. Typically used only for bring-up.
   *
   * @param volts
   */
  public void runVoltage(double volts);
}
