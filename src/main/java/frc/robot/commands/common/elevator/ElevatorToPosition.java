package frc.robot.commands.common.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.interfaces.Elevator;
import java.util.function.DoubleSupplier;

public class ElevatorToPosition extends Command {
  Elevator elevator;
  DoubleSupplier positionDegrees;
  double targetPositionMeters;

  public ElevatorToPosition(Elevator elevator, DoubleSupplier positionMeters) {
    this.elevator = elevator;
    this.positionDegrees = positionMeters;

    addRequirements((SubsystemBase) elevator);
  }

  @Override
  public void initialize() {
    targetPositionMeters = positionDegrees.getAsDouble();

    if (Constants.debugCommands) {
      System.out.println(
          "START: " + this.getClass().getSimpleName() + " position: " + targetPositionMeters);
    }

    elevator.setTargetHeight(targetPositionMeters);
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return elevator.isAtSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      System.err.println("INTERRUPTED: " + this.getClass().getSimpleName());
    }

    if (Constants.debugCommands) {
      System.out.println("  END: " + this.getClass().getSimpleName());
    }
  }
}
