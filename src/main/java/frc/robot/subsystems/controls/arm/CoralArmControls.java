package frc.robot.subsystems.controls.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.common.arm.ArmCommandV2;
import frc.robot.commands.common.motor.MotorAutoResetEncoderCommand;
import frc.robot.commands.common.motor.MotorAutoResetEncoderCommand.MotorAutoResetEncoderSettings;
import frc.robot.subsystems.interfaces.ArmV2;
import frc.robot.subsystems.interfaces.Motor;

public class CoralArmControls {
  // RIGHT POV = up arm
  // LEFT POV = down arm
  public static void setupController(ArmV2 arm, CommandXboxController controller) {
    SubsystemBase armSubsystem = (SubsystemBase) arm;
    armSubsystem.setDefaultCommand(
        new ArmCommandV2(
            arm,
            () -> {
              if (controller.pov(90).getAsBoolean()) {
                return 1.0;
              } else if (controller.pov(270).getAsBoolean()) {
                return -1.0;
              }
              return 0.0;
            }));

    /* Add Auto Zero */
    MotorAutoResetEncoderSettings settings = new MotorAutoResetEncoderSettings();
    settings.voltage = 1.0;
    settings.minResetCurrent = 5.0;
    settings.resetPositionRad = arm.getSettings().maxAngleInDegrees;
    Command autoCalibrateCommand = new MotorAutoResetEncoderCommand((Motor) arm, settings);
    SmartDashboard.putData("Auto Calibrate Coral Arm", autoCalibrateCommand);
  }
}
