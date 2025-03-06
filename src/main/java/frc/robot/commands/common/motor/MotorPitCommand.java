package frc.robot.commands.common.motor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.interfaces.Motor;

public class MotorPitCommand extends Command {
  Motor motor;
  String key;
  double voltage;

  public MotorPitCommand(Motor motor, String key) {
    this.motor = motor;
    this.key = key;
    SmartDashboard.putNumber(key, voltage);

    addRequirements((Subsystem) motor);
  }

  @Override
  public void execute() {
    voltage = SmartDashboard.getNumber(key, 0);
    motor.runVoltage(voltage);
  }

  @Override
  public void end(boolean interrupted) {
    motor.runVoltage(0);
  }
}
