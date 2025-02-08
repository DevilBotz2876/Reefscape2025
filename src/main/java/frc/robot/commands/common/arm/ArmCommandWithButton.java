package frc.robot.commands.common.arm;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.interfaces.Arm;
import java.util.function.DoubleSupplier;

public class ArmCommandWithButton extends Command {
  Arm arm;
  DoubleSupplier speed;
  double targetPosition;
  double targetFinalPosition;
  double maxArmVelocityInDegreesPerSec = Arm.Constants.maxVelocityInDegreesPerSecond;

  public ArmCommandWithButton(Arm arm, Double targetFinalPosition) {
    this.arm = arm;
    this.targetPosition = targetFinalPosition;

    addRequirements((Subsystem) arm);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    arm.setAngle(targetPosition);
  }

  // @Override
  // public boolean isFinished(){
  //   return arm.isAtTargetPosition();
  // }
}
