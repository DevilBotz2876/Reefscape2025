package frc.robot.subsystems.controls.combination;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.common.arm.ArmToPosition;
import frc.robot.commands.common.elevator.ElevatorToPosition;
import frc.robot.subsystems.interfaces.Arm;
import frc.robot.subsystems.interfaces.Elevator;

public class DriverControls {
  public static void setupController(
      Elevator elevator,
      Arm arm,
      CommandXboxController controller,
      SendableChooser<Command> prepareScoreChooser) {
    Trigger ableToIntake =
        new Trigger(
            () -> {
              if (elevator.getCurrentHeight() > 0.5) {
                return true;
              }
              return false;
            });
    controller
        .a()
        .and(ableToIntake)
        .onTrue(
            new SequentialCommandGroup(
                new ArmToPosition(arm, () -> -90).withTimeout(0),
                new ElevatorToPosition(elevator, () -> 0.8)));

    controller
        .leftTrigger()
        .onTrue(new SequentialCommandGroup(new ElevatorToPosition(elevator, () -> 0.3)));

    controller.rightTrigger().onTrue(new SequentialCommandGroup(new ArmToPosition(arm, () -> 0)));

    controller.y().onTrue(prepareScoreChooser.getSelected());
    prepareScoreChooser.onChange(
        (selected) -> {
          controller.y().onTrue(prepareScoreChooser.getSelected());
        });
  }
}
