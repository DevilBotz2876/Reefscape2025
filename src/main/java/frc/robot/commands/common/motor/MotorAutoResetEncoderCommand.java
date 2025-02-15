package frc.robot.commands.common.motor;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.interfaces.Motor;
import java.util.function.BooleanSupplier;

public class MotorAutoResetEncoderCommand extends Command {
  private final Motor motor;
  private final MotorAutoResetEncoderSettings settings;
  private boolean done = false;

  public static class MotorAutoResetEncoderSettings {
    // Voltage to apply to the mechanism to reach the limit/hard stop\
    public double voltage = 2.0;

    // any current detected above this value will trigger the encoder reset
    public double minResetCurrent = 5;

    // the relative encoder value to set when limit is detected. Units are
    // whatever is native to the resetEncoder function
    public double resetPositionRad = 0;

    // Optional: If there's a limit switch, this will return true when the limit switch is
    // triggered. Otherwise, minCurrentAtReset will be used to dectect the limit
    public BooleanSupplier limitTriggered = null;
  }

  public MotorAutoResetEncoderCommand(Motor arm, MotorAutoResetEncoderSettings settings) {
    this.motor = arm;
    this.settings = settings;
  }

  @Override
  public void initialize() {
    done = false;
  }

  @Override
  public void execute() {
    motor.runVoltage(settings.voltage);

    /* TODO: Add code to use limit switch if one exists */
    if (settings.voltage > 0.0 && motor.getForwardLimit()) {
      motor.runVoltage(0);
      done = true;
    }
    if (settings.voltage < 0.0 && motor.getReverseLimit()) {
      motor.runVoltage(0);
      done = true;
    }

    if (motor.getCurrent() > settings.minResetCurrent) {
      motor.runVoltage(0);
      done = true;
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
    }
  }
}
