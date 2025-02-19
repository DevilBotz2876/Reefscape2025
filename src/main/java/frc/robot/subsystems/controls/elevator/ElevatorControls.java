package frc.robot.subsystems.controls.elevator;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.common.elevator.ElevatorCommand;
import frc.robot.commands.common.elevator.ElevatorToPosition;
import frc.robot.commands.common.motor.MotorAutoResetEncoderCommand;
import frc.robot.commands.common.motor.MotorAutoResetEncoderCommand.MotorAutoResetEncoderSettings;
import frc.robot.subsystems.interfaces.Elevator;
import frc.robot.subsystems.interfaces.Motor;

public class ElevatorControls {
  public static class Constants {
    public static MotorAutoResetEncoderSettings autoZeroSettings =
        new MotorAutoResetEncoderSettings();
  }

  // UP POV = Up elevator
  // DOWN POV = Down elevator
  public static void setupController(Elevator elevator, CommandXboxController controller) {
    SubsystemBase subsystem = (SubsystemBase) elevator;
    subsystem.setDefaultCommand(
        // TODO: Change this to use the elevator command (see commented lines below) after initial
        // bringup and Feedforward/PID tuning
        // new MotorBringUpCommand(
        // (Motor) elevator,
        new ElevatorCommand(
            elevator,
            () -> {
              if (controller.pov(0).getAsBoolean()) {
                return 0.1;
              } else if (controller.pov(180).getAsBoolean()) {
                return -0.1;
              }
              return 0.0;
            }));

    /* Add Auto Zero */
    Command autoCalibrateCommand =
        new MotorAutoResetEncoderCommand((Motor) elevator, Constants.autoZeroSettings);
    SmartDashboard.putData(
        subsystem.getName() + "/Commands/Auto Calibrate Elevator", autoCalibrateCommand);

    SmartDashboard.putData(
        subsystem.getName() + "/Commands/Elevator To 0\"",
        new ElevatorToPosition(elevator, () -> Units.inchesToMeters(0)));
    SmartDashboard.putData(
        subsystem.getName() + "/Commands/Elevator To 12\"",
        new ElevatorToPosition(elevator, () -> Units.inchesToMeters(12)));
    SmartDashboard.putData(
        subsystem.getName() + "/Commands/Elevator To 24\"",
        new ElevatorToPosition(elevator, () -> Units.inchesToMeters(24)));
    SmartDashboard.putData(
        subsystem.getName() + "/Commands/Elevator To 36\"",
        new ElevatorToPosition(elevator, () -> Units.inchesToMeters(36)));
    SmartDashboard.putData(
        subsystem.getName() + "/Commands/Elevator To 48\"",
        new ElevatorToPosition(elevator, () -> Units.inchesToMeters(48)));
  }
}
