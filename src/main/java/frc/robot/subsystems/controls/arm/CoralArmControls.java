package frc.robot.subsystems.controls.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.common.arm.ArmCommand;
import frc.robot.commands.common.arm.ArmToPosition;
import frc.robot.commands.common.motor.MotorAutoResetEncoderCommand;
import frc.robot.commands.common.motor.MotorAutoResetEncoderCommand.MotorAutoResetEncoderSettings;
import frc.robot.commands.common.motor.MotorBringUpCommand;
import frc.robot.subsystems.interfaces.Arm;
import frc.robot.subsystems.interfaces.Motor;

public class CoralArmControls {
  public static class Constants {
    public static MotorAutoResetEncoderSettings autoZeroSettings =
        new MotorAutoResetEncoderSettings();
  }

  // RIGHT POV = up arm
  // LEFT POV = down arm
  public static void setupController(Arm arm, CommandXboxController controller) {
    SubsystemBase motorSubsystem = (SubsystemBase) arm;

    motorSubsystem.setDefaultCommand(
        new ArmCommand(
            arm,
            () -> {
              if (controller.povRight().getAsBoolean()) {
                return 0.1;
              } else if (controller.povLeft().getAsBoolean()) {
                return -0.1;
              }
              return 0.0;
            }));

    /* Add Auto Zero */
    Command autoCalibrateCommand =
        new MotorAutoResetEncoderCommand((Motor) arm, Constants.autoZeroSettings);
    SmartDashboard.putData(
        motorSubsystem.getName() + "/Commands/Auto Calibrate Coral Arm", autoCalibrateCommand);

    SmartDashboard.putData(
        motorSubsystem.getName() + "/Commands/Arm To -90", new ArmToPosition(arm, () -> -90));
    SmartDashboard.putData(
        motorSubsystem.getName() + "/Commands/Arm To -45", new ArmToPosition(arm, () -> -45));
    SmartDashboard.putData(
        motorSubsystem.getName() + "/Commands/Arm To 0", new ArmToPosition(arm, () -> 0));
    SmartDashboard.putData(
        motorSubsystem.getName() + "/Commands/Arm To 15", new ArmToPosition(arm, () -> 15));
    SmartDashboard.putData(
        motorSubsystem.getName() + "/Commands/Arm To 45", new ArmToPosition(arm, () -> 45));
    SmartDashboard.putData(
        motorSubsystem.getName() + "/Commands/Arm To 75", new ArmToPosition(arm, () -> 75));

     SmartDashboard.putData(
      motorSubsystem.getName() + "/Commands/Arm Voltage Up", new MotorBringUpCommand((Motor)arm ,() -> 0.01));
    SmartDashboard.putData(
       motorSubsystem.getName() + "/Commands/Arm Voltage Down", new MotorBringUpCommand((Motor)arm ,() -> -0.01));
       SmartDashboard.putData(
        motorSubsystem.getName() + "/Commands/Arm Voltage 0", new MotorBringUpCommand((Motor)arm ,() -> 0));
  }
}
