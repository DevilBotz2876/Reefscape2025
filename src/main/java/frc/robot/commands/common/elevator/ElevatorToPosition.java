package frc.robot.commands.common.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.Elevator;
import java.util.function.DoubleSupplier;

public class ElevatorToPosition extends Command {
  Elevator elevator;
  DoubleSupplier positionMeters;
  double targetPositionMeters;

  public ElevatorToPosition(Elevator elevator, DoubleSupplier positionMeters) {
    this.elevator = elevator;
    this.positionMeters = positionMeters;

    addRequirements((SubsystemBase) elevator);
  }

  @Override
  public void initialize() {
    targetPositionMeters = positionMeters.getAsDouble();

    elevator.setPosition(targetPositionMeters);
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return elevator.isAtSetpoint();
  }

  @Override
  public void end(boolean interrupted) {}
}
