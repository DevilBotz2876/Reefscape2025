package frc.robot.commands.common.motor;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.interfaces.Motor;
import java.util.function.BooleanSupplier;

public class ElevatorPitCalibrationCommand extends Command {
  private final Motor motor;
  private final MotorAutoResetEncoderSettings settings;
  private boolean done = false;
  private int state = 0;

  public static class MotorAutoResetEncoderSettings {
    // Voltage to apply to the mechanism to reach the limit/hard stop\
    public double voltage = 2.0;

    // any current detected above this value will trigger the encoder reset
    public double minResetCurrent = 5;

    // the relative encoder value to set when limit is detected. Units are
    // whatever is native to the resetEncoder function
    public double resetPositionRad = 0;

    public double initialReverseDuration = 0;

    public double currentReverseDuration = 0;

    // Optional: If there's a limit switch, this will return true when the limit switch is
    // triggered. Otherwise, minCurrentAtReset will be used to dectect the limit
    public BooleanSupplier limitTriggered = null;
  }

  public ElevatorPitCalibrationCommand(Motor arm, MotorAutoResetEncoderSettings settings) {
    this.motor = arm;
    this.settings = settings;
  }

  @Override
  public void initialize() {
    done = false;
    state = 1;
    settings.currentReverseDuration = settings.initialReverseDuration;
  }

  @Override
  public void execute() {
    /* State list:
     * 1: Elevator is moving down to the reverse limit switch
     * - Checks if the elevator activated the reverse limit switch
     * 2: Elevator motor is spinning in the opposite direction to eliminate any slack in the rope
     * - Checks if the current has spiked in the motor
     * 3: Current has spiked
     * - Stops motor and returns done
     * Default: Happens if an error occurs
     */
    switch(state)
    {
      case 1: // Wait for reverse limit switch to trigger
        motor.runVoltage(-settings.voltage);
        if (motor.getReverseLimit()){
          state = 2;
        }
        break;
      case 2: // Limit has been triggered, now reverse direction and wait for current to spike
        motor.runVoltage(settings.voltage);
        if (motor.getCurrent() > settings.minResetCurrent){
          state = 3;
        }
        break;
      case 3: // Current Spiked
        motor.runVoltage(0);
        state = 0;
        done = true;
        break;
      default:
      System.out.println("Elevator calibration failed");
      state = 0;
        break;
    }
  }

  @Override
  public boolean isFinished() {
    return done;
  }

  @Override
  public void end(boolean interrupted) {
    if (false == interrupted) {
      motor.resetEncoder(settings.resetPositionRad);
      motor.runVoltage(0);
    }
  }
}
