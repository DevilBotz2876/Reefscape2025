package frc.robot.commands.common.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.interfaces.Elevator;
import java.util.function.DoubleSupplier;

public class ElevatorVolts extends Command {
  private final Elevator elevator;
  private DoubleSupplier volts;

  public ElevatorVolts(Elevator elevator, DoubleSupplier volts) {
    this.elevator = elevator;
    this.volts = volts;

    addRequirements((Subsystem) elevator);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double volts = this.volts.getAsDouble();
    // if(volts != 0.0) {
      elevator.runVoltage(volts);
    // }
  }
}
