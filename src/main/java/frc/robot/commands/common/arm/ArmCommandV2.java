package frc.robot.commands.common.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.interfaces.ArmV2;
import java.util.function.DoubleSupplier;

public class ArmCommandV2 extends Command {
  ArmV2 arm;
  DoubleSupplier speed;
  double targetPosition;

  public ArmCommandV2(ArmV2 arm, DoubleSupplier speed) {
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

    targetPosition += currentSpeed * arm.getSettings().maxVelocityInDegreesPerSecond / 50;
    targetPosition =
        MathUtil.clamp(
            targetPosition,
            arm.getSettings().minAngleInDegrees,
            arm.getSettings().maxAngleInDegrees);

    arm.setTargetAngle(targetPosition);
  }
}
