package frc.robot.subsystems.controls.arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.common.arm.ArmToPositionV2;
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
                return 0.01;
              } else if (controller.povLeft().getAsBoolean()) {
                return -0.01;
              }
              return 0.0;
            }));

    /* Add Auto Zero */
    MotorAutoResetEncoderSettings settings = new MotorAutoResetEncoderSettings();
    settings.voltage = -0.5;
    settings.minResetCurrent = 0.5;
    settings.resetPositionRad =
        Units.degreesToRadians(
            arm.getSettings().minAngleInDegrees - 10); // We have an offest about 15 degrees
    settings.initialReverseDuration =
        1.0; // Set the seconds of reverse before zero. Set to zero if there shound be no reverse
    Command autoCalibrateCommand = new MotorAutoResetEncoderCommand((Motor) arm, settings);
    SmartDashboard.putData(motorSubsystem.getName() + "/Commands/Auto Calibrate Coral Arm", autoCalibrateCommand);

    Command armToNeg60Command = new ArmToPositionV2(arm, () -> -20);
    SmartDashboard.putData(motorSubsystem.getName() + "/Commands/Arm To -20", armToNeg60Command);
  }
}
