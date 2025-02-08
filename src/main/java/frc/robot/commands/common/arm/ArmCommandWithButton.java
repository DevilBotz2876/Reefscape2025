package frc.robot.commands.common.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.interfaces.Arm;
import swervelib.encoders.ThriftyNovaEncoderSwerve;

import java.util.function.DoubleSupplier;

import javax.lang.model.util.ElementScanner14;

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