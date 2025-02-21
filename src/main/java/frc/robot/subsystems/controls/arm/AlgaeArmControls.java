package frc.robot.subsystems.controls.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.common.arm.ArmToPosition;
import frc.robot.commands.common.motor.MotorAutoResetEncoderCommand;
import frc.robot.commands.common.motor.MotorAutoResetEncoderCommand.MotorAutoResetEncoderSettings;
import frc.robot.commands.common.motor.MotorBringUpCommand;
import frc.robot.subsystems.interfaces.Arm;
import frc.robot.subsystems.interfaces.Motor;

public class AlgaeArmControls {
  public static class Constants {
    public static MotorAutoResetEncoderSettings autoZeroSettings =
        new MotorAutoResetEncoderSettings();
  }

  // RIGHT TRIGGER = up arm
  // LEFT TRIGGER = down arm
  public static void setupController(Arm arm, CommandXboxController controller) {
    SubsystemBase armSubsystem = (SubsystemBase) arm;
    armSubsystem.setDefaultCommand(
        new MotorBringUpCommand(
            (Motor) arm,
            //        new ArmToPosition(
            //            arm,
            () -> {
              return (controller.getRightTriggerAxis() - controller.getLeftTriggerAxis());
            }));

    Command autoCalibrateCommand =
        new MotorAutoResetEncoderCommand((Motor) arm, Constants.autoZeroSettings);
    SmartDashboard.putData(
        armSubsystem.getName() + "/Commands/Algae Arm Auto Calibrate ", autoCalibrateCommand);

    SmartDashboard.putData(
        armSubsystem.getName() + "/Commands/Algae Arm Down",
        new ArmToPosition(arm, () -> arm.getSettings().minAngleInDegrees));

    SmartDashboard.putData(
        armSubsystem.getName() + "/Commands/Algae Arm Up",
        new ArmToPosition(arm, () -> arm.getSettings().maxAngleInDegrees));
  }
}
