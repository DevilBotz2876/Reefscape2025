package frc.robot.commands.common.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.interfaces.ElevatorV2;
import java.util.function.DoubleSupplier;

public class ElevatorCommandV2 extends Command {
  ElevatorV2 elevator;
  DoubleSupplier speed;
  double targetPosition;

  public ElevatorCommandV2(ElevatorV2 elevator, DoubleSupplier speed) {
    this.elevator = elevator;
    this.speed = speed;

    addRequirements((Subsystem) elevator);
  }

  @Override
  public void initialize() {
    targetPosition = elevator.getCurrentHeight();
  }

  @Override
  public void execute() {
    double currentSpeed = speed.getAsDouble();

    targetPosition += currentSpeed * elevator.getSettings().maxVelocityInMetersPerSecond / 50;
    targetPosition =
        MathUtil.clamp(
            targetPosition,
            elevator.getSettings().minHeightInMeters,
            elevator.getSettings().maxHeightInMeters);

    elevator.setTargetHeight(targetPosition);
  }
}
