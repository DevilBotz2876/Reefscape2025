package frc.robot.subsystems.interfaces;

public interface LimitSwitch {

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
