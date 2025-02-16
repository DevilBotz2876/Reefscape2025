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

public class ClimberArmControls {
  // Y-button = up arm
  // A-button = down arm
  public static void setupController(ArmV2 arm, CommandXboxController controller) {
    SubsystemBase armSubsystem = (SubsystemBase) arm;
    armSubsystem.setDefaultCommand(
        new MotorBringUpCommand(
            (Motor) arm,
            () -> {
              if (controller.y().getAsBoolean()) {
                return 0.2;
              } else if (controller.a().getAsBoolean()) {
                return -0.2;
              }
              return 0.0;
            }));
    MotorAutoResetEncoderSettings settings = new MotorAutoResetEncoderSettings();
    settings.voltage = -0.5;
    settings.minResetCurrent = 10.0;
    settings.resetPositionRad = Units.degreesToRadians(arm.getSettings().minAngleInDegrees);
    Command autoCalibrateCommand = new MotorAutoResetEncoderCommand((Motor) arm, settings);
    SmartDashboard.putData("Auto Calibrate Climber Arm", autoCalibrateCommand);
  }
}
