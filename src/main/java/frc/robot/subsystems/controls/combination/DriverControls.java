package frc.robot.subsystems.controls.combination;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.common.arm.ArmToPosition;
import frc.robot.commands.common.elevator.ElevatorToPosition;
import frc.robot.subsystems.interfaces.Arm;
import frc.robot.subsystems.interfaces.Elevator;

public class DriverControls {
  public static void setupController(Elevator elevator, Arm arm, CommandXboxController controller) {


    controller.a().onTrue(new SequentialCommandGroup(
        new ArmToPosition(arm, () -> -90).withTimeout(0),
        new ElevatorToPosition(elevator, () -> 0.8)));

    controller. leftTrigger().onTrue(new SequentialCommandGroup(new ElevatorToPosition(elevator, () -> 0.4)));

    controller.rightTrigger().onTrue(new SequentialCommandGroup(new ArmToPosition(arm, () -> 0)));

    controller.y().onTrue(new SequentialCommandGroup(
        new ElevatorToPosition(elevator, () -> 1.0),
        new ArmToPosition(arm, () -> 75),
        new ElevatorToPosition(elevator, () -> 1.553)));
  }
  
}
