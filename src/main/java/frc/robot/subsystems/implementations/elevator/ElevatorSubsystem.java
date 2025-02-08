package frc.robot.subsystems.implementations.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.io.interfaces.ElevatorIO;
import frc.robot.io.interfaces.ElevatorIOInputsAutoLogged;
import frc.robot.subsystems.interfaces.Elevator;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase implements Elevator {

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private ProfiledPIDController pid = null;
  private boolean softwarePidEnabled = false;
  private double targetMeters = 0;
  private double feedForwardVolts = 0;
  private ElevatorFeedforward elevatorFeedForward = null;

  // Simulation related things
  private MechanismLigament2d elevator2d = null;
  private final double elevatorLigament2dScale = 40;
  private final double elevatorLigament2doffset = 0.05;
  private final SysIdRoutine sysId;

  public ElevatorSubsystem(ElevatorIO io) {
    this(io, null);
  }

  public ElevatorSubsystem(ElevatorIO io, String name) {
    this.io = io;

    if (name != null) {
      setName("Elevator(" + name + ")");
    } else {
      setName("Elevator");
    }

    sysId = createSystemIdRoutine(getName());

    this.elevatorFeedForward =
        new ElevatorFeedforward(
            Elevator.Constants.ffKs,
            Elevator.Constants.ffKg,
            Elevator.Constants.ffKv,
            Elevator.Constants.ffKa);

    this.pid =
        new ProfiledPIDController(
            Elevator.Constants.pidKp,
            Elevator.Constants.pidKi,
            Elevator.Constants.pidKd,
            new TrapezoidProfile.Constraints(
                Elevator.Constants.maxVelocityInDegreesPerSecond,
                Elevator.Constants.maxVelocityInDegreesPerSecond));

    SmartDashboard.putData(this);
    SmartDashboard.putData(this.pid);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(getName(), inputs);

    SmartDashboard.putBoolean(this.getName() + "/pidEnabled", softwarePidEnabled);
    if (softwarePidEnabled) {
      double pidOutput = pid.calculate(inputs.positionMeters, targetMeters);
      feedForwardVolts = elevatorFeedForward.calculate(pid.getSetpoint().velocity);
      double volts = feedForwardVolts + pidOutput * RobotController.getBatteryVoltage();
      SmartDashboard.putNumber(this.getName() + "/pidOutput", pidOutput);
      SmartDashboard.putNumber(this.getName() + "/pidFF", feedForwardVolts);
      SmartDashboard.putNumber(this.getName() + "/pidVolts", volts);

      runVoltage(volts);
    }
  }

  public void simulationPeriodic() {

    if (null != elevator2d) {
      elevator2d.setLength(
          (inputs.positionMeters + elevatorLigament2doffset) * elevatorLigament2dScale);
    }
  }

  @Override
  public double getPosition() {
    return inputs.positionMeters;
  }

  @Override
  public double getTargetPosition() {
    return this.targetMeters;
  }

  @Override
  public boolean isAtUpperLimit() {
    return inputs.upperLimit;
  }

  @Override
  public boolean isAtLowerLimit() {
    return inputs.lowerLimit;
  }

  @Override
  public double getVelocityPerSecond() {
    return inputs.velocityInMetersPerSecond;
  }

  @Override
  public void setPosition(double meters) {
    meters =
        MathUtil.clamp(
            meters, Elevator.Constants.minPositionInMeters, Elevator.Constants.maxPositionInMeters);
    this.targetMeters = meters;

    io.setPosition(meters);
    softwarePidEnabled = true;
  }

  public void disableClosedLoop() {
    softwarePidEnabled = false;
  }

  @Override
  public void runVoltage(double volts) {
    io.setVoltage(volts);
  }

  @Override
  public boolean isAtSetpoint() {
    // return Math.abs(targetMeters - inputs.positionMeters) <= Elevator.Constants.pidErrorInMeters;
    return pid.atSetpoint();
  }

  @Override
  public void setLigament(MechanismLigament2d ligament2d) {
    elevator2d = ligament2d;
  }

  @Override
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  @Override
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }
}
