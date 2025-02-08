package frc.robot.commands.common.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.interfaces.Arm;
import swervelib.encoders.ThriftyNovaEncoderSwerve;

import java.util.function.DoubleSupplier;

import javax.lang.model.util.ElementScanner14;

public class ArmCommand extends Command {
  Arm arm;
  DoubleSupplier speed;
  double targetPosition;
  double currentAngle;
  double maxArmVelocityInDegreesPerSec = Arm.Constants.maxVelocityInDegreesPerSecond;

  public ArmCommand(Arm arm, DoubleSupplier speed) {
    this.arm = arm;
    this.speed = speed;

    addRequirements((Subsystem) arm);
  }

  @Override
  public void initialize() {
    targetPosition = arm.getAngle();
    currentAngle = arm.getAngle();
  }

  @Override
  public void execute() {
    double currentSpeed = speed.getAsDouble();

    targetPosition += currentSpeed * maxArmVelocityInDegreesPerSec / 50;
    targetPosition =
        MathUtil.clamp(
            targetPosition, Arm.Constants.minAngleInDegrees, Arm.Constants.maxAngleInDegrees);

    arm.setAngle(targetPosition);
  }
  
 public void positionArm(double targetFinalPosition){
 if (currentAngle >= 0 ){

  double currentSpeed = speed.getAsDouble();
  
  if (targetFinalPosition < currentAngle || currentSpeed == 0){
      targetPosition += -currentSpeed * maxArmVelocityInDegreesPerSec / 50;
      arm.setAngle(targetPosition);
    } else {
      System.out.println("CLIMBER MUST NOT BE MOVING TO EXECUTE THIS COMMAND");
  }
  if (targetFinalPosition > currentAngle || currentSpeed == 0){
      targetPosition += currentSpeed * maxArmVelocityInDegreesPerSec / 50;
      arm.setAngle(targetPosition);
    } else {
      System.out.println("CLIMBER MUST NOT BE MOVING TO EXECUTE THIS COMMAND");
}
}
}
}

