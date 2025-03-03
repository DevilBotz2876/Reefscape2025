package frc.robot.subsystems.controls.combination;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.common.arm.ArmToPosition;
import frc.robot.commands.common.elevator.ElevatorToPosition;
import frc.robot.subsystems.interfaces.Arm;
import frc.robot.subsystems.interfaces.Elevator;

public class DriverControls {
  public static class Choosers {
    public static SendableChooser<Command> prepareScoreChooser = new SendableChooser<>();
  }

  public static void setupController(Elevator elevator, Arm arm, CommandXboxController controller) {
    Trigger ableToIntake =
        new Trigger(
            () -> {
              if (elevator.getCurrentHeight() > 0.5) {
                return true;
              }
              return false;
            });

    Command prepareIntakeCoralCommand =
        new SequentialCommandGroup(
            new ArmToPosition(arm, () -> -90).withTimeout(0),
            new ElevatorToPosition(elevator, () -> 0.8));
    controller.a().and(ableToIntake).onTrue(prepareIntakeCoralCommand);

    Command intakeCoralCommand = new ElevatorToPosition(elevator, () -> 0.4);
    controller.leftTrigger().onTrue(intakeCoralCommand);

    Command score = new ArmToPosition(arm, () -> 0);
    controller.rightTrigger().onTrue(score);

    DriverControls.Choosers.prepareScoreChooser.setDefaultOption(
        "L2",
        new SequentialCommandGroup(
            new ElevatorToPosition(elevator, () -> 0.53), new ArmToPosition(arm, () -> 75)));

    DriverControls.Choosers.prepareScoreChooser.addOption(
        "L3",
        new SequentialCommandGroup(
            new ElevatorToPosition(elevator, () -> 1.0), new ArmToPosition(arm, () -> 75)));

    DriverControls.Choosers.prepareScoreChooser.addOption(
        "L4",
        new SequentialCommandGroup(
            new ElevatorToPosition(elevator, () -> 0.6),
            new ParallelCommandGroup(
                new ArmToPosition(arm, () -> 75), new ElevatorToPosition(elevator, () -> 1.553))));

    SmartDashboard.putData(
        "Driver " + "/Commands/Prepare To Score Command",
        DriverControls.Choosers.prepareScoreChooser.getSelected());

    SmartDashboard.putData(
        "Driver " + "/Commands/Prepare To Score Chooser",
        DriverControls.Choosers.prepareScoreChooser);

    controller.y().onTrue(DriverControls.Choosers.prepareScoreChooser.getSelected());
    DriverControls.Choosers.prepareScoreChooser.onChange(
        (selected) -> {
          SmartDashboard.putData("Driver " + "/Commands/Prepare To Score Command", selected);
          controller.y().onTrue(selected);
        });
  }
}
