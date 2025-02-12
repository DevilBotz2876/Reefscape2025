package frc.robot.commands.common.motor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.interfaces.Motor;
import java.util.function.DoubleSupplier;

public class MotorBringUpCommand extends Command {
  Motor motor;
  DoubleSupplier voltage;
  double currentVoltage;

  public MotorBringUpCommand(Motor motor, DoubleSupplier voltage) {
    this.motor = motor;
    this.voltage = voltage;

    addRequirements((Subsystem) motor);
  }

  @Override
  public void initialize() {
    currentVoltage = 0;
  }

  @Override
  public void execute() {
    currentVoltage += voltage.getAsDouble() / 10;
    motor.runVoltage(currentVoltage);
  }
}
