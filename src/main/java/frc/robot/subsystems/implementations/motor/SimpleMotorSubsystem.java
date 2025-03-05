package frc.robot.subsystems.implementations.motor;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.io.interfaces.MotorIO;
import frc.robot.subsystems.interfaces.SimpleMotor;
import org.littletonrobotics.junction.Logger;

public class SimpleMotorSubsystem extends MotorSubsystem implements SimpleMotor {
  private final SimpleMotorSettings settings;
  private double targetPositionRad = 0;

  private TrapezoidProfile.Constraints motionProfileConstraintsRadians = new Constraints(0, 0);

  public SimpleMotorSubsystem(MotorIO io, String name, SimpleMotorSettings settings) {
    super(io, "Position[" + name + "]");
    this.settings = settings;

    resetEncoder(settings.startingPositionInRads);
    setMotionProfileConstraintsRad(
        new Constraints(
            settings.maxVelocityInRadiansPerSecond,
            settings.maxAccelerationInRadiansPerSecondSquared));
  }

  @Override
  public void periodic() {
    if (motionProfileEnabled) {
      io.setPosition(nextState.position, settings.feedforward.calculate(nextState.velocity));

      Logger.recordOutput(getName() + "/targetMotionProfiledPositionRad", nextState.position);
      Logger.recordOutput(getName() + "/targetMotionProfiledVelocityRadPerSec", nextState.velocity);

      nextState = motionProfile.calculate(0.02, nextState, targetState);
    }

    super.periodic();
    Logger.recordOutput(getName() + "/targetPositionRad", targetPositionRad);
  }

  @Override
  public SimpleMotorSettings getSettings() {
    return settings;
  }

  @Override
  public double getCurrentPosition() {
    return inputs.positionRad;
  }

  @Override
  public void setTargetPosition(double positionRad) {
    this.targetPositionRad = positionRad;

    targetState = new State(this.targetPositionRad, 0);
    nextState =
        motionProfile.calculate(
            0.02, new State(inputs.positionRad, inputs.velocityRadPerSec), targetState);
    motionProfileEnabled = true;
  }

  @Override
  public double getTargetPosition() {
    return this.targetPositionRad;
  }

  @Override
  public boolean isAtMaxLimit() {
    return (getTargetPosition() >= settings.maxPositionInRads);
  }

  @Override
  public boolean isAtMinLimit() {
    return (getTargetPosition() <= settings.minPositionInRads);
  }

  @Override
  public boolean isAtSetpoint() {
    return (Math.abs(getCurrentPosition() - this.targetPositionRad)
        <= settings.targetHeightToleranceInRad);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty(
        "motionProfile/maxVelocity (radians per sec)", this::getMaxVelocity, this::setMaxVelocity);
    builder.addDoubleProperty(
        "motionProfile/maxAcceleration (radians per sec^2)",
        this::getMaxAcceleration,
        this::setMaxAcceleration);
    builder.addDoubleProperty(
        "feedforward/Ks", () -> null == settings ? 0 : settings.feedforward.getKs(), this::setFFKs);
    builder.addDoubleProperty(
        "feedforward/Kv", () -> null == settings ? 0 : settings.feedforward.getKv(), this::setFFKv);
    builder.addDoubleProperty(
        "feedforward/Ka", () -> null == settings ? 0 : settings.feedforward.getKa(), this::setFFKa);
  }

  private double getMaxVelocity() {
    if (null != motionProfileConstraintsRadians) {
      return motionProfileConstraintsRadians.maxVelocity;
    } else {
      return 0;
    }
  }

  private void setMaxVelocity(double velocityDegreesPerSec) {
    if (motionProfileConstraintsRadians.maxVelocity != velocityDegreesPerSec)
      setMotionProfileConstraintsRad(
          new Constraints(velocityDegreesPerSec, motionProfileConstraintsRadians.maxAcceleration));
  }

  private double getMaxAcceleration() {
    if (null != motionProfileConstraintsRadians) {
      return motionProfileConstraintsRadians.maxAcceleration;
    } else {
      return 0;
    }
  }

  private void setMaxAcceleration(double accelerationRadiansPerSecSquared) {
    if (motionProfileConstraintsRadians.maxAcceleration != accelerationRadiansPerSecSquared)
      setMotionProfileConstraintsRad(
          new Constraints(
              motionProfileConstraintsRadians.maxVelocity, accelerationRadiansPerSecSquared));
  }

  public void setMotionProfileConstraintsRad(Constraints motionProfileConstraintsRadians) {
    this.motionProfileConstraintsRadians = motionProfileConstraintsRadians;
    super.setMotionProfileConstraintsRad(motionProfileConstraintsRadians);
  }

  private void setFFKs(double Ks) {
    settings.feedforward.setKs(Ks);
  }

  private void setFFKv(double Kv) {
    settings.feedforward.setKv(Kv);
  }

  private void setFFKa(double Ka) {
    settings.feedforward.setKa(Ka);
  }
}
