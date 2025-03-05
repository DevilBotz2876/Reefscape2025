package frc.robot.subsystems.implementations.motor;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.io.interfaces.MotorIO;
import frc.robot.subsystems.interfaces.Position;

import org.littletonrobotics.junction.Logger;

public class PositionMotorSubsystem extends MotorSubsystem implements Position {
  private final PositionSettings settings;
  private double targetPositionRad = 0;

  /* 2D Simulation */
  private final Mechanism2d mech2d;
  private final MechanismLigament2d elevatorSegment;

  private TrapezoidProfile.Constraints motionProfileConstraintsMeters = new Constraints(0, 0);

  public PositionMotorSubsystem(MotorIO io, String name, PositionSettings settings) {
    super(io, "Position[" + name + "]");
    this.settings = settings;

    resetEncoder(settings.startingPositionInRads);
    setMotionProfileConstraintsMeters(
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

    // We get the current linear position of the motor and update the 2D sim accordingly
    elevatorSegment.setLength(inputs.positionMeters * 64);
  }

  @Override
  public PositionSettings getSettings() {
    return settings;
  }

  @Override
  public double getCurrentHeight() {
    return inputs.positionMeters;
  }

  @Override
  public void setTargetHeight(double meters) {
    this.targetHeightMeters = meters;
    this.targetPositionRad = io.normalizePositionToRad(this.targetHeightMeters);

    targetState = new State(this.targetPositionRad, 0);
    nextState =
        motionProfile.calculate(
            0.02, new State(inputs.positionRad, inputs.velocityRadPerSec), targetState);
    motionProfileEnabled = true;
  }

  @Override
  public double getTargetHeight() {
    return this.targetHeightMeters;
  }

  @Override
  public boolean isAtMaxLimit() {
    return (getTargetHeight() >= settings.maxHeightInMeters);
  }

  @Override
  public boolean isAtMinLimit() {
    return (getTargetHeight() <= settings.minHeightInMeters);
  }

  @Override
  public boolean isAtSetpoint() {
    return (Math.abs(getCurrentHeight() - this.targetHeightMeters)
        <= settings.targetHeightToleranceInMeters);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty(
        "motionProfile/maxVelocity (meters per sec)", this::getMaxVelocity, this::setMaxVelocity);
    builder.addDoubleProperty(
        "motionProfile/maxAcceleration (meters per sec^2)",
        this::getMaxAcceleration,
        this::setMaxAcceleration);
    builder.addDoubleProperty(
        "feedforward/Ks", () -> null == settings ? 0 : settings.feedforward.getKs(), this::setFFKs);
    builder.addDoubleProperty(
        "feedforward/Kg", () -> null == settings ? 0 : settings.feedforward.getKg(), this::setFFKg);
    builder.addDoubleProperty(
        "feedforward/Kv", () -> null == settings ? 0 : settings.feedforward.getKv(), this::setFFKv);
    builder.addDoubleProperty(
        "feedforward/Ka", () -> null == settings ? 0 : settings.feedforward.getKa(), this::setFFKa);
  }

  private double getMaxVelocity() {
    if (null != motionProfileConstraintsMeters) {
      return motionProfileConstraintsMeters.maxVelocity;
    } else {
      return 0;
    }
  }

  private void setMaxVelocity(double velocityDegreesPerSec) {
    if (motionProfileConstraintsMeters.maxVelocity != velocityDegreesPerSec)
      setMotionProfileConstraintsMeters(
          new Constraints(velocityDegreesPerSec, motionProfileConstraintsMeters.maxAcceleration));
  }

  private double getMaxAcceleration() {
    if (null != motionProfileConstraintsMeters) {
      return motionProfileConstraintsMeters.maxAcceleration;
    } else {
      return 0;
    }
  }

  private void setMaxAcceleration(double accelerationDegreesPerSecSquared) {
    if (motionProfileConstraintsMeters.maxAcceleration != accelerationDegreesPerSecSquared)
      setMotionProfileConstraintsMeters(
          new Constraints(
              motionProfileConstraintsMeters.maxVelocity, accelerationDegreesPerSecSquared));
  }

  private void setMotionProfileConstraintsMeters(Constraints motionProfileConstraintsDegrees) {
    this.motionProfileConstraintsMeters = motionProfileConstraintsDegrees;
    setMotionProfileConstraintsRad(
        new Constraints(
            io.normalizePositionToRad(motionProfileConstraintsDegrees.maxVelocity),
            io.normalizePositionToRad(motionProfileConstraintsDegrees.maxAcceleration)));
  }

  private void setFFKs(double Ks) {
    settings.feedforward.setKs(Ks);
  }

  private void setFFKg(double Kg) {
    settings.feedforward.setKg(Kg);
  }

  private void setFFKv(double Kv) {
    settings.feedforward.setKv(Kv);
  }

  private void setFFKa(double Ka) {
    settings.feedforward.setKa(Ka);
  }
}
