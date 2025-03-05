package frc.robot.subsystems.controls.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.common.motor.MotorAutoResetEncoderCommand;
import frc.robot.commands.common.motor.MotorAutoResetEncoderCommand.MotorAutoResetEncoderSettings;
import frc.robot.subsystems.interfaces.Motor;
import frc.robot.subsystems.interfaces.SimpleMotor;

public class ClimberArmControls {
  public static class Constants {
    public static MotorAutoResetEncoderSettings autoZeroSettings =
        new MotorAutoResetEncoderSettings();
  }

  // Y-button = up arm
  // A-button = down arm
  public static void setupController(SimpleMotor arm, CommandXboxController controller) {

    SubsystemBase armSubsystem = (SubsystemBase) arm;
    // armSubsystem.setDefaultCommand(
    //     new MotorBringUpCommand(
    //         (Motor) arm,
    //         () -> {
    //           if (controller.y().getAsBoolean()) {
    //             return 0.2;
    //           } else if (controller.a().getAsBoolean()) {
    //             return -0.2;
    //           }
    //           return 0.0;
    //         }));

    Command autoCalibrateCommand =
        new MotorAutoResetEncoderCommand((Motor) arm, Constants.autoZeroSettings);
    SmartDashboard.putData(
        armSubsystem.getName() + "/Commands/Climber Auto Calibrate ", autoCalibrateCommand);

    SmartDashboard.putData(
        armSubsystem.getName() + "/Commands/Climber to Start Position",
        new InstantCommand(
            () -> arm.setTargetPosition(arm.getSettings().startingPositionInRads), armSubsystem));
    SmartDashboard.putData(
        armSubsystem.getName() + "/Commands/Prepare to Climb",
        new InstantCommand(
            () -> arm.setTargetPosition(arm.getSettings().maxPositionInRads), armSubsystem));
    SmartDashboard.putData(
        armSubsystem.getName() + "/Commands/Climb",
        new InstantCommand(
            () -> arm.setTargetPosition(arm.getSettings().minPositionInRads), armSubsystem));
  }
}
