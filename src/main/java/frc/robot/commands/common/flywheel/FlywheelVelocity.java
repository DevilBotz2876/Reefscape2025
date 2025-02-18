package frc.robot.commands.common.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.interfaces.Flywheel;
import java.util.function.DoubleSupplier;

public class FlywheelVelocity extends Command {
  Flywheel flywheel;
  DoubleSupplier velocityRPM;
  double targetVelocityRPM;

  public FlywheelVelocity(Flywheel flywheel, DoubleSupplier velocityRPM) {
    this.flywheel = flywheel;
    this.velocityRPM = velocityRPM;
    addRequirements((SubsystemBase) flywheel);
  }

  @Override
  public void initialize() {
    if (Constants.debugCommands) {
      System.out.println(
          "START: " + this.getClass().getSimpleName() + " velocity: " + velocityRPM.getAsDouble());
    }
    targetVelocityRPM = velocityRPM.getAsDouble();
    flywheel.setTargetVelocity(targetVelocityRPM);
  }

  @Override
  public boolean isFinished() {
    return flywheel.isAtSetpoint();
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
