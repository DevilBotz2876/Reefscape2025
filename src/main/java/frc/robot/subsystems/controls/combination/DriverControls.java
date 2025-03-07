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
import frc.robot.commands.common.arm.ArmCommand;
import frc.robot.commands.common.arm.ArmToPosition;
import frc.robot.commands.common.elevator.ElevatorCommand;
import frc.robot.commands.common.elevator.ElevatorToPosition;
import frc.robot.subsystems.interfaces.Arm;
import frc.robot.subsystems.interfaces.Elevator;
import frc.robot.subsystems.interfaces.SimpleMotor;
import java.util.Map;

public class DriverControls {
  // prob wrong name not constants
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
    controller
        .b()
        .onTrue(
            prepareIntakeCoralCommand.andThen(
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

    controller.leftTrigger().and(scoreMode).onTrue(getPrepareToScoreCommand(elevator, coralArm));

    controller
        .y()
        .onTrue(
            new InstantCommand(
                () -> {
                  DriverControls.Constants.prepareScoreSelctedIndex = 4;
                  SmartDashboard.putNumber(
                      "Driver " + "/Misc/Prepare Selection",
                      DriverControls.Constants.prepareScoreSelctedIndex);
                }));

    controller
        .x()
        .onTrue(
            new InstantCommand(
                () -> {
                  DriverControls.Constants.prepareScoreSelctedIndex = 3;
                  SmartDashboard.putNumber(
                      "Driver " + "/Misc/Prepare Selection",
                      DriverControls.Constants.prepareScoreSelctedIndex);
                }));

    controller
        .a()
        .onTrue(
            new InstantCommand(
                () -> {
                  DriverControls.Constants.prepareScoreSelctedIndex = 2;
                  SmartDashboard.putNumber(
                      "Driver " + "/Misc/Prepare Selection",
                      DriverControls.Constants.prepareScoreSelctedIndex);
                }));

    DriverControls.Constants.prepareChooser.onChange(
        (index) -> {
          DriverControls.Constants.prepareScoreSelctedIndex = index;
          SmartDashboard.putNumber("Driver " + "/Misc/Prepare To Score Selection", index);
        });

    SmartDashboard.putNumber(
        "Driver " + "/Misc/Prepare To Score Selection", Constants.prepareChooser.getSelected());

    SmartDashboard.putData(
        "Driver " + "/Commands/Prepare To Score Command",
        getPrepareToScoreCommand(elevator, coralArm));

    SmartDashboard.putData("Driver " + "/Misc/Prepare To Score Chooser", Constants.prepareChooser);

    // climber
    // SubsystemBase climberSubsystem = (SubsystemBase) climber;
    // // prepare to climb
    // controller
    //     .leftBumper()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> climber.setTargetPosition(climber.getSettings().maxPositionInRads),
    //             climberSubsystem));

    // // climb
    // controller
    //     .rightBumper()
    //     .onTrue(
    //         new InstantCommand(
    //             () -> climber.setTargetPosition(climber.getSettings().minPositionInRads),
    //             climberSubsystem));

    // multi controll not workking in each subsystem inde
    controller.povUp().whileTrue(new ElevatorCommand(elevator, () -> 0.2));

    controller.povDown().whileTrue(new ElevatorCommand(elevator, () -> -0.2));

    controller.povRight().whileTrue(new ArmCommand(coralArm, () -> 0.1));

    controller.povLeft().whileTrue(new ArmCommand(coralArm, () -> -0.1));
  }

  public static Command getPrepareToScoreCommand(Elevator elevator, Arm coralArm) {
    return new SelectCommand<>(
        Map.ofEntries(
            Map.entry(1, new InstantCommand(() -> System.out.println("Not scoring mode selected"))),
            Map.entry(
                2,
                new SequentialCommandGroup(
                        new ElevatorToPosition(elevator, () -> 0.63),
                        new ArmToPosition(coralArm, () -> 47))
                    .withTimeout(5.0)),
            Map.entry(
                3,
                new SequentialCommandGroup(
                        new ElevatorToPosition(elevator, () -> 1.0),
                        new ArmToPosition(coralArm, () -> 47))
                    .withTimeout(5.0)),
            Map.entry(
                4,
                new SequentialCommandGroup(
                        new ElevatorToPosition(elevator, () -> 0.6),
                        new ParallelCommandGroup(
                            new ArmToPosition(coralArm, () -> 48).withTimeout(1.0),
                            new ElevatorToPosition(elevator, () -> 1.553)))
                    .withTimeout(5.0))),
        () -> {
          return Constants.prepareScoreSelctedIndex;
        });
  }
}
