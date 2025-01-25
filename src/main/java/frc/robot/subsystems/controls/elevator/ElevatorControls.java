package frc.robot.subsystems.controls.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.common.elevator.ElevatorCommand;
import frc.robot.subsystems.interfaces.Elevator;

public class ElevatorControls {
  public static void setupController(Elevator elevator, CommandXboxController controller) {
    SubsystemBase elevatorSubsystem = (SubsystemBase) elevator;
    elevatorSubsystem.setDefaultCommand(
        new ElevatorCommand(
            elevator,
            () -> MathUtil.applyDeadband(-controller.getLeftY(), 0.05) // Robot Strafe Front/Back
            ));
  }
}
