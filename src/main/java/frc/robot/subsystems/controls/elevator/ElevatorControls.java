package frc.robot.subsystems.controls.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.common.elevator.ElevatorVolts;
import frc.robot.subsystems.interfaces.Elevator;

public class ElevatorControls {
  public static void setupController(Elevator elevator, CommandXboxController controller) {
    SubsystemBase elevatorSubsystem = (SubsystemBase) elevator;
    elevatorSubsystem.setDefaultCommand(
        new ElevatorVolts(elevator, () -> {
          if(controller.pov(0).getAsBoolean()) {
            return 1.0;
          } else if(controller.pov(180).getAsBoolean()) {
            return -1.0;
          }
          return 0.0;
        }));
  }
}
