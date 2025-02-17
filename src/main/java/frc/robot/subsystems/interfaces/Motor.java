package frc.robot.subsystems.interfaces;

public interface Motor {
  /**
   * Runs the motor at the specified voltage. Typically used only for bring-up.
   *
   * @param volts
   */
  public void runVoltage(double volts);

  /**
   * Returns the current used by the motor
   *
   * @return returns the instantaneous current usage in amps
   */
  public double getCurrent();

  /**
   * Resets the relative encoder to the specified value (in radians)
   *
   * @param positonRad the current position in radians
   */
  public void resetEncoder(double positionRad);

  /**
   * Returns status of the max limit switch
   *
   * @return returns true if limit switch is triggered, otherwise false.
   */
  public boolean maxLimitTriggered();

  /**
   * Returns status of the min limit switch
   *
   * @return returns true if limit switch is triggered, otherwise false.
   */
  public boolean minLimitTriggered();
}
