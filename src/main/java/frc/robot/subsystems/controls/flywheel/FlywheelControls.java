package frc.robot.subsystems.controls.flywheel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.common.motor.MotorBringUpCommand;
import frc.robot.subsystems.interfaces.Motor;

public class FlywheelControls {
  // Right Bumper = Increase Rotation CW
  // Left Bumper = Increase Rotation CCW
  public static void setupController(Motor motor, CommandXboxController controller) {
    SubsystemBase motorSubsystem = (SubsystemBase) motor;
    motorSubsystem.setDefaultCommand(
        new MotorBringUpCommand(
            motor,
            () -> {
              if (controller.rightBumper().getAsBoolean()) {
                return 1.0;
              } else if (controller.leftBumper().getAsBoolean()) {
                return -1.0;
              }
              return 0.0;
            }));
  }
}
