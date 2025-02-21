package frc.robot.subsystems.controls.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.common.motor.MotorBringUpCommand;
import frc.robot.subsystems.interfaces.Motor;

public class AlgaeArmControls {
  // RIGHT TRIGGER = up arm
  // LEFT TRIGGER = down arm
  public static void setupController(Motor motor, CommandXboxController controller) {
    SubsystemBase motorSubsystem = (SubsystemBase) motor;
    motorSubsystem.setDefaultCommand(
        new MotorBringUpCommand(
            motor,
            () -> {
              return (controller.getRightTriggerAxis() - controller.getLeftTriggerAxis());
            }));
  }
}
