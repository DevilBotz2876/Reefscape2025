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
import java.util.function.DoubleSupplier;

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
    settings.voltage = -1;

    // Set this to something big, we are never going to use stall current to detect if climber has
    // reached it's end of range of motion.
    settings.minResetCurrent = 10.0;

    settings.resetPositionRad = Units.degreesToRadians(arm.getSettings().minAngleInDegrees);

    Command autoCalibrateCommand = new MotorAutoResetEncoderCommand((Motor) arm, settings);
    SmartDashboard.putData(
        armSubsystem.getName() + "/Commands/Climber Auto Calibrate ", autoCalibrateCommand);
    // assign to button temporarily to debug.
    // controller.y().onTrue(runCalibration);

    DoubleSupplier endClimbPosition = () -> 100.0;
    Command endClimberPosition = new ArmToPositionV2(arm, endClimbPosition);
    SmartDashboard.putData(armSubsystem.getName() + "/Commands/Climber To End", endClimberPosition);
    // assign to button temporarily to debug.
    // controller.b().onTrue(endClimberPosition);

    DoubleSupplier startClimbPosition = () -> 0.0;
    Command startClimberPosition = new ArmToPositionV2(arm, startClimbPosition);
    SmartDashboard.putData(
        armSubsystem.getName() + "/Commands/Climber To Start", startClimberPosition);
    // assign to button temporarily to debug.
    // controller.a().onTrue(startClimberPosition);
  }
}
