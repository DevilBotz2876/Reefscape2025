package frc.robot.subsystems.controls.arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.common.motor.MotorAutoResetEncoderCommand;
import frc.robot.commands.common.motor.MotorAutoResetEncoderCommand.MotorAutoResetEncoderSettings;
import frc.robot.commands.common.motor.MotorBringUpCommand;
import frc.robot.subsystems.interfaces.ArmV2;
import frc.robot.subsystems.interfaces.Motor;

public class CoralArmControls {
  // RIGHT POV = up arm
  // LEFT POV = down arm
  public static void setupController(ArmV2 arm, CommandXboxController controller) {
    SubsystemBase motorSubsystem = (SubsystemBase) arm;
    motorSubsystem.setDefaultCommand(
        new MotorBringUpCommand(
            (Motor) arm,
            () -> {
              if (controller.povRight().getAsBoolean()) {
                return 1.0;
              } else if (controller.povLeft().getAsBoolean()) {
                return -1.0;
              }
              return 0.0;
            }));

    /* Add Auto Zero */
    MotorAutoResetEncoderSettings settings = new MotorAutoResetEncoderSettings();
    settings.voltage = -0.5;
    settings.minResetCurrent = 0.6;
    settings.resetPositionRad = Units.degreesToRadians(arm.getSettings().minAngleInDegrees);
    Command autoCalibrateCommand = new MotorAutoResetEncoderCommand((Motor) arm, settings);
    SmartDashboard.putData("Auto Calibrate Coral Arm", autoCalibrateCommand);
  }
}
