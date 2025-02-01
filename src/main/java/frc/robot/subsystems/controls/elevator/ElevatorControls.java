package frc.robot.subsystems.controls.elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.interfaces.Elevator;

public class ElevatorControls {
  public static void setupController(Elevator elevator, CommandXboxController controller) {
    SubsystemBase elevatorSubsystem = (SubsystemBase) elevator;
    controller
        .povUp()
        .onTrue(
            new InstantCommand(
                () -> {
                  elevator.runVoltage(6.0);
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  if (controller.povCenter().getAsBoolean()) {
                    elevator.runVoltage(0.0);
                  }
                }));
    ;

    controller
        .povDown()
        .onTrue(
            new InstantCommand(
                () -> {
                  elevator.runVoltage(-1.0);
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  if (controller.povCenter().getAsBoolean()) {
                    elevator.runVoltage(0.0);
                  }
                }));
  }
}
