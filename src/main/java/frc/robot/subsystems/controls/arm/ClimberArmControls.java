package frc.robot.subsystems.controls.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.common.arm.ArmToPosition;
import frc.robot.commands.common.motor.MotorAutoResetEncoderCommand;
import frc.robot.commands.common.motor.MotorAutoResetEncoderCommand.MotorAutoResetEncoderSettings;
import frc.robot.subsystems.interfaces.Arm;
import frc.robot.subsystems.interfaces.Motor;
import java.util.function.DoubleSupplier;

public class ClimberArmControls {
  public static class Constants {
    public static MotorAutoResetEncoderSettings autoZeroSettings =
        new MotorAutoResetEncoderSettings();
  }

  // Y-button = up arm
  // A-button = down arm
  public static void setupController(Arm arm, CommandXboxController controller) {

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
    // assign to button temporarily to debug.
    // controller.y().onTrue(runCalibration);

    DoubleSupplier endClimbPosition = () -> 100.0;
    Command endClimberPosition = new ArmToPosition(arm, endClimbPosition);
    SmartDashboard.putData(armSubsystem.getName() + "/Commands/Climber To End", endClimberPosition);
    // assign to button temporarily to debug.
    // controller.b().onTrue(endClimberPosition);

    DoubleSupplier startClimbPosition = () -> 0.0;
    Command startClimberPosition = new ArmToPosition(arm, startClimbPosition);
    SmartDashboard.putData(
        armSubsystem.getName() + "/Commands/Climber To Start", startClimberPosition);
    // assign to button temporarily to debug.
    // controller.a().onTrue(startClimberPosition);
  }
}
