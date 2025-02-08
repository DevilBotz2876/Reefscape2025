package frc.robot.commands.common.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.interfaces.Elevator;

public class ElevatorOpenLoop extends Command {
  private final Elevator elevator;
  private double volts;

  public ElevatorOpenLoop(Elevator elevator, double volts) {
    this.elevator = elevator;
    this.volts = volts;

    addRequirements((Subsystem) elevator);
  }

  @Override
  public void initialize() {
    elevator.disableClosedLoop();
  }

  @Override
  public void execute() {
    elevator.runVoltage(volts);
  }
}
