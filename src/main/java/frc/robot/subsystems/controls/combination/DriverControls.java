package frc.robot.subsystems.controls.combination;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.common.arm.ArmCommand;
import frc.robot.commands.common.arm.ArmToPosition;
import frc.robot.commands.common.elevator.ElevatorCommand;
import frc.robot.commands.common.elevator.ElevatorToPosition;
import frc.robot.subsystems.interfaces.Arm;
import frc.robot.subsystems.interfaces.Elevator;
import frc.robot.subsystems.interfaces.SimpleMotor;
import java.util.Map;

public class DriverControls {
    //prob wrong name not constants
    public static class Constants {
        public static SendableChooser<Integer> prepareChooser = new SendableChooser<>();
        public static Integer prepareScoreSelctedIndex = 2;
      }

  public static void setupController(
      Elevator elevator, Arm coralArm, SimpleMotor climber, CommandXboxController controller) {
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
            new ArmToPosition(coralArm, () -> -90).withTimeout(0),
            new ElevatorToPosition(elevator, () -> 0.8));
    controller.b().onTrue(prepareIntakeCoralCommand.andThen(
        new InstantCommand(
        () -> {
          
          DriverControls.Constants.prepareScoreSelctedIndex = 1;
          SmartDashboard.putNumber(
              "Driver " + "/Misc/Prepare Selection",
              DriverControls.Constants.prepareScoreSelctedIndex);
        })));

    Command intakeCoralCommand = new ElevatorToPosition(elevator, () -> 0.35);
    Trigger scoreMode = new Trigger(() -> Constants.prepareScoreSelctedIndex >= 2);
    controller.leftTrigger().and(scoreMode.negate()).and(ableToIntake).onTrue(intakeCoralCommand);

    Command score = new ArmToPosition(coralArm, () -> 0);
    controller.rightBumper().onTrue(score);

    Constants.prepareChooser.setDefaultOption("L2", 2);

    Constants.prepareChooser.addOption("L3", 3);

    Constants.prepareChooser.addOption("L4", 4);
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
