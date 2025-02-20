package frc.robot.commands.common.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.interfaces.Arm;
import java.util.function.DoubleSupplier;

public class ArmCommand extends Command {
  Arm arm;
  DoubleSupplier speed;
  double targetPosition;

  public ArmCommand(Arm arm, DoubleSupplier speed) {
    this.arm = arm;
    this.speed = speed;

    addRequirements((Subsystem) arm);
  }

  @Override
  public void initialize() {
    targetPosition = arm.getCurrentAngle();
  }

  @Override
  public void execute() {
    double currentSpeed = speed.getAsDouble();
    if (0 != currentSpeed) {
      targetPosition += currentSpeed * arm.getSettings().maxVelocityInDegreesPerSecond / 50;
      targetPosition =
          MathUtil.clamp(
              targetPosition,
              arm.getSettings().minAngleInDegrees,
              arm.getSettings().maxAngleInDegrees);

      arm.setTargetAngle(targetPosition);
    }
  }
}
