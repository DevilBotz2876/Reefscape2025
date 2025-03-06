package frc.robot.subsystems.controls.combination;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.common.arm.ArmToPosition;
import frc.robot.commands.common.elevator.ElevatorToPosition;
import frc.robot.subsystems.interfaces.Arm;
import frc.robot.subsystems.interfaces.Elevator;
import java.util.Map;

public class DriverControls {
  public static class Constants {
    public static SendableChooser<Integer> prepareScoreChooser = new SendableChooser<>();
    public static Integer prepareScoreSelctedIndex = 2;
    public static Command prepareScoreCommand = new InstantCommand();
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

    Constants.prepareScoreChooser.setDefaultOption("L2", 2);

    Constants.prepareScoreChooser.addOption("L3", 3);

    Constants.prepareScoreChooser.addOption("L4", 4);

    Command prepareScoreControllerCommand =
        new SelectCommand<>(
            Map.ofEntries(
                Map.entry(
                    2,
                    new SequentialCommandGroup(
                            new ElevatorToPosition(elevator, () -> 0.63),
                            new ArmToPosition(arm, () -> 75))
                        .withTimeout(5.0)),
                Map.entry(
                    3,
                    new SequentialCommandGroup(
                            new ElevatorToPosition(elevator, () -> 1.0),
                            new ArmToPosition(arm, () -> 75))
                        .withTimeout(5.0)),
                Map.entry(
                    4,
                    new SequentialCommandGroup(
                            new ElevatorToPosition(elevator, () -> 0.6),
                            new ParallelCommandGroup(
                                new ArmToPosition(arm, () -> 75).withTimeout(1.0),
                                new ElevatorToPosition(elevator, () -> 1.553)))
                        .withTimeout(5.0))),
            () -> {
              return Constants.prepareScoreSelctedIndex;
            });

    Constants.prepareScoreCommand =
        new SelectCommand<>(
            Map.ofEntries(
                Map.entry(
                    2,
                    new SequentialCommandGroup(
                            new ElevatorToPosition(elevator, () -> 0.63),
                            new ArmToPosition(arm, () -> 75))
                        .withTimeout(5.0)),
                Map.entry(
                    3,
                    new SequentialCommandGroup(
                            new ElevatorToPosition(elevator, () -> 1.0),
                            new ArmToPosition(arm, () -> 75))
                        .withTimeout(5.0)),
                Map.entry(
                    4,
                    new SequentialCommandGroup(
                            new ElevatorToPosition(elevator, () -> 0.6),
                            new ParallelCommandGroup(
                                new ArmToPosition(arm, () -> 75).withTimeout(1.0),
                                new ElevatorToPosition(elevator, () -> 1.553)))
                        .withTimeout(5.0))),
            () -> {
              return Constants.prepareScoreSelctedIndex;
            });

    SmartDashboard.putNumber(
        "Driver " + "/Commands/Prepare To Score Selection",
        Constants.prepareScoreChooser.getSelected());

    SmartDashboard.putData(
        "Driver " + "/Commands/Prepare To Score Command", prepareScoreControllerCommand);

    SmartDashboard.putData(
        "Driver " + "/Commands/Prepare To Score Chooser", Constants.prepareScoreChooser);

    controller.y().onTrue(prepareScoreControllerCommand);
  }

  public static Command getPrepareToScoreCommand(Elevator elevator, Arm arm) {
    return new SelectCommand<>(
        Map.ofEntries(
            Map.entry(
                2,
                new SequentialCommandGroup(
                        new ElevatorToPosition(elevator, () -> 0.63),
                        new ArmToPosition(arm, () -> 75))
                    .withTimeout(5.0)),
            Map.entry(
                3,
                new SequentialCommandGroup(
                        new ElevatorToPosition(elevator, () -> 1.0),
                        new ArmToPosition(arm, () -> 75))
                    .withTimeout(5.0)),
            Map.entry(
                4,
                new SequentialCommandGroup(
                        new ElevatorToPosition(elevator, () -> 0.6),
                        new ParallelCommandGroup(
                            new ArmToPosition(arm, () -> 75).withTimeout(1.0),
                            new ElevatorToPosition(elevator, () -> 1.553)))
                    .withTimeout(5.0))),
        () -> {
          return Constants.prepareScoreSelctedIndex;
        });
  }
}
