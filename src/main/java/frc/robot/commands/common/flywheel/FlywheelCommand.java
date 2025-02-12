package frc.robot.commands.common.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.interfaces.Flywheel;
import java.util.function.DoubleSupplier;

public class FlywheelCommand extends Command {
  Flywheel flywheel;
  DoubleSupplier speed;
  double targetVelocity;

  public FlywheelCommand(Flywheel flywheel, DoubleSupplier speed) {
    this.flywheel = flywheel;
    this.speed = speed;

    addRequirements((Subsystem) flywheel);
  }

  @Override
  public void initialize() {
    targetVelocity = flywheel.getCurrentVelocity();
  }

  @Override
  public void execute() {
    double currentSpeed = speed.getAsDouble();

    targetVelocity += currentSpeed * flywheel.getSettings().maxVelocityInRPMs / 50;
    targetVelocity = MathUtil.clamp(targetVelocity, 0, flywheel.getSettings().maxVelocityInRPMs);

    flywheel.setTargetVelocity(targetVelocity);
  }
}
