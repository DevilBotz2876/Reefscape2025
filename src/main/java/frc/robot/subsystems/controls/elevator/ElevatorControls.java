package frc.robot.subsystems.controls.elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.interfaces.Elevator;

public class ElevatorControls {
  public static void setupController(Elevator elevator, CommandXboxController controller) {
    SubsystemBase elevatorSubsystem = (SubsystemBase) elevator;
    Trigger elevatorUp = new Trigger(() -> controller.pov(0).getAsBoolean());
    elevatorUp.onTrue(
        new InstantCommand(
            () -> {
              elevator.runVoltage(6.0);
            }));

    Trigger elevatorDown = new Trigger(() -> controller.pov(180).getAsBoolean());
    elevatorDown.onTrue(
        new InstantCommand(
            () -> {
              elevator.runVoltage(-1.0);
            }));

    Trigger elevatorNothing = new Trigger(() -> controller.povCenter().getAsBoolean());
    elevatorNothing.onTrue(
        new InstantCommand(
            () -> {
              elevator.runVoltage(0.0);
            }));
  }
}
