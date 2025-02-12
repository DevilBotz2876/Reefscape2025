package frc.robot.subsystems.implementations.motor;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.io.interfaces.MotorIO;
import frc.robot.io.interfaces.MotorIOInputsAutoLogged;
import frc.robot.subsystems.interfaces.Motor;
import org.littletonrobotics.junction.Logger;

/**
 * The MotorSubsystem class implements the basic functionality of a generic mechanism that uses a
 * motor. It implements shared sysId routines as well as the Motor interface
 */
public class MotorSubsystem extends SubsystemBase implements Motor {
  protected final MotorIO io;
  protected final MotorIOInputsAutoLogged inputs = new MotorIOInputsAutoLogged();
  private final SysIdRoutine sysId;

  public MotorSubsystem(MotorIO io, String name) {
    this.io = io;
    setName(name);

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput(getName() + "/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVoltage(voltage.in(Volts)), null, this));

    SmartDashboard.putData(getName() + "/PID", io.getPid());

    SmartDashboard.putData(
        getName() + "/Dynamic Forward", sysIdDynamic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData(
        getName() + "/Dynamic Reverse", sysIdDynamic(SysIdRoutine.Direction.kReverse));
    SmartDashboard.putData(
        getName() + "/Quasistatic Forward", sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData(
        getName() + "/Quasistatic Reverse", sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
  }

  @Override
  public void periodic() {
    super.periodic();
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);
  }

  public void runVoltage(double volts) {
    io.setVoltage(volts);
  }

  private Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  private Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }
}
