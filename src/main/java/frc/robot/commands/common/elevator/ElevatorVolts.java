package frc.robot.commands.common.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.interfaces.Elevator;
import java.util.function.DoubleSupplier;

public class ElevatorVolts extends Command {
  private final Elevator elevator;
  private DoubleSupplier speed;

  public ElevatorVolts(Elevator elevator, DoubleSupplier speed) {
    this.elevator = elevator;
    this.speed = speed;

    addRequirements((Subsystem) elevator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double volts = speed.getAsDouble() * 5;
    elevator.runVoltage(volts);
  }
}
