package frc.robot.subsystems.controls.arm;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.common.motor.MotorBringUpCommand;
import frc.robot.subsystems.interfaces.Motor;

public class CoralArmControls {
  // RIGHT POV = up arm
  // LEFT POV = down arm
  public static void setupController(Motor motor, CommandXboxController controller) {
    SubsystemBase motorSubsystem = (SubsystemBase) motor;
    motorSubsystem.setDefaultCommand(
        new MotorBringUpCommand(
            motor,
            () -> {
              if (controller.povRight().getAsBoolean()) {
                return 1.0;
              } else if (controller.povLeft().getAsBoolean()) {
                return -1.0;
              }
              return 0.0;
            }));
  }
}
