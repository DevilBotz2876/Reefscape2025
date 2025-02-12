package frc.robot.subsystems.controls.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.common.motor.MotorBringUpCommand;
import frc.robot.subsystems.interfaces.Motor;

public class ElevatorControlsV2 {
  // UP POV = Up elevator
  // DOWN POV = Down elevator
  public static void setupController(Motor motor, CommandXboxController controller) {
    SubsystemBase motorSubsystem = (SubsystemBase) motor;
    motorSubsystem.setDefaultCommand(
        new MotorBringUpCommand(
            motor,
            () -> {
              if (controller.pov(0).getAsBoolean()) {
                return 1.0;
              } else if (controller.pov(180).getAsBoolean()) {
                return -1.0;
              }
              return 0.0;
            }));
  }
}
