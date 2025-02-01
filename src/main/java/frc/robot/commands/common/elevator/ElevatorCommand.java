package frc.robot.commands.common.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.interfaces.Elevator;
import java.util.function.DoubleSupplier;

public class ElevatorCommand extends Command {
  private final Elevator elevator;
  private DoubleSupplier speed;
  double targetPosition;
  double maxElevatorVelocity = Elevator.Constants.maxVelocity;

  public ElevatorCommand(Elevator elevator, DoubleSupplier speed) {
    this.elevator = elevator;
    this.speed = speed;

    addRequirements((Subsystem) elevator);
  }

  @Override
  public void initialize() {
    targetPosition = elevator.getPosition();
  }

  @Override
  public void execute() {
    double currentSpeed = speed.getAsDouble();

    targetPosition += currentSpeed * maxElevatorVelocity / 50;
    targetPosition =
        MathUtil.clamp(
            targetPosition,
            Elevator.Constants.minPositionInMeters,
            Elevator.Constants.maxPositionInMeters);

    elevator.setPosition(targetPosition);
  }
}
