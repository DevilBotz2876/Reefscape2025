package frc.robot.subsystems.implementations.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
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

  private MechanismLigament2d elevator2d = null;
  private final double elevatorLigament2dScale = 40;
  private final double elevatorLigament2doffset = 0.05;

  private double targetMeters = 0.0;
  private SysIdRoutine sysId;

  public ElevatorSubsystem(ElevatorIO io) {
    this.io = io;

    sysId = createSystemIdRoutine("Elevator/SysIdState");
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

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
  }

  @Override
  public void runVoltage(double volts) {
    io.setVoltage(volts);
  }

  @Override
  public boolean isAtSetpoint() {
    return Math.abs(targetMeters - inputs.positionMeters) <= Elevator.Constants.pidErrorInMeters;
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
