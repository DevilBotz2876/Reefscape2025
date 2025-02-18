package frc.robot.subsystems.implementations.motor;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.RobotState;
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
  protected TrapezoidProfile motionProfile;
  protected State targetState;
  protected State nextState;
  protected boolean motionProfileEnabled = false;

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

    SmartDashboard.putData(getName() + "/Subsystem", this);
  }

  @Override
  public void periodic() {
    super.periodic();
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);

    if (RobotState.isDisabled()) {
      runVoltage(0);
    }
  }

  @Override
  public void runVoltage(double volts) {
    io.disablePid(); // Disable the PID if we are setting the voltage directly
    motionProfileEnabled = false;
    io.setVoltage(volts);
  }

  @Override
  public double getCurrent() {
    return inputs.currentAmps;
  }

  @Override
  public boolean getForwardLimit() {
    return inputs.forwardLimit;
  }

  @Override
  public boolean getReverseLimit() {
    return inputs.reverseLimit;
  }

  @Override
  public void resetEncoder(double positionRad) {
    io.resetEncoder(positionRad);
  }

  private Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  private Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }
}
