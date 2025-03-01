package frc.robot.subsystems.controls.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.common.arm.ArmToPosition;
import frc.robot.commands.common.elevator.ElevatorToPosition;
import frc.robot.commands.common.motor.MotorAutoResetEncoderCommand;
import frc.robot.commands.common.motor.MotorAutoResetEncoderCommand.MotorAutoResetEncoderSettings;
import frc.robot.commands.common.motor.MotorBringUpCommand;
import frc.robot.subsystems.interfaces.Arm;
import frc.robot.subsystems.interfaces.Elevator;
import frc.robot.subsystems.interfaces.Motor;
import java.util.function.DoubleSupplier;

public class ClimberArmControls {
  public static class Constants {
    public static MotorAutoResetEncoderSettings autoZeroSettings =
        new MotorAutoResetEncoderSettings();
  }

  // Y-button = up arm
  // A-button = down arm
  public static void setupController(Elevator climber, CommandXboxController controller) {

    SubsystemBase armSubsystem = (SubsystemBase) climber;
    armSubsystem.setDefaultCommand(
        new MotorBringUpCommand(
            (Motor) climber,
            () -> {
              if (controller.y().getAsBoolean()) {
                return 0.2;
              } else if (controller.a().getAsBoolean()) {
                return -0.2;
              }
              return 0.0;
            }));

    Command autoCalibrateCommand =
        new MotorAutoResetEncoderCommand((Motor) climber, Constants.autoZeroSettings);
    SmartDashboard.putData(
        armSubsystem.getName() + "/Commands/Climber Auto Calibrate ", autoCalibrateCommand);
    // assign to button temporarily to debug.
    // controller.y().onTrue(runCalibration);

    DoubleSupplier endClimbPosition = () -> climber.getSettings().maxHeightInMeters;
    Command endClimberPosition = new ElevatorToPosition(climber, endClimbPosition);
    SmartDashboard.putData(armSubsystem.getName() + "/Commands/Climber To End", endClimberPosition);
    // assign to button temporarily to debug.
    // controller.b().onTrue(endClimberPosition);

    DoubleSupplier startClimbPosition = () -> climber.getSettings().minHeightInMeters;
    Command startClimberPosition = new ElevatorToPosition(climber, startClimbPosition);
    SmartDashboard.putData(
        armSubsystem.getName() + "/Commands/Climber To Start", startClimberPosition);
    // assign to button temporarily to debug.
    // controller.a().onTrue(startClimberPosition);
  }
}
