package frc.robot.subsystems.implementations.motor;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.io.interfaces.MotorIO;
import frc.robot.subsystems.interfaces.Arm;
import org.littletonrobotics.junction.Logger;

public class ArmMotorSubsystem extends MotorSubsystem implements Arm {
  private final ArmSettings settings;
  private double targetAngleDegrees = 0;
  private double targetAngleRad = 0;

  /* 2D Simulation */
  private final Mechanism2d mech2d;
  private final MechanismLigament2d armSegment;

  private TrapezoidProfile.Constraints motionProfileConstraintsDegrees = new Constraints(0, 0);

  public ArmMotorSubsystem(MotorIO io, String name, ArmSettings settings) {
    super(io, "Arm[" + name + "]");
    this.settings = settings;

    resetEncoder(Units.degreesToRadians(settings.startingAngleInDegrees));
    setMotionProfileConstraintsDegrees(
        new Constraints(
            settings.maxVelocityInDegreesPerSecond,
            settings.maxAccelerationInDegreesPerSecondSquared));

    double sim2dSize = settings.armLengthInMeters * 64 * 2;
    mech2d = new Mechanism2d(sim2dSize, sim2dSize);

    MechanismRoot2d armPivot2d = mech2d.getRoot(getName() + "Pivot", sim2dSize / 2, sim2dSize / 2);

    armSegment =
        armPivot2d.append(
            new MechanismLigament2d(
                getName(), sim2dSize / 2, settings.startingAngleInDegrees, 15, settings.color));

    SmartDashboard.putData(getName() + "/2D Simulation", mech2d);
  }

  @Override
  public void periodic() {
    if (motionProfileEnabled) {
      io.setPosition(
          nextState.position,
          settings.feedforward.calculate(
              nextState.position, Units.degreesToRadians(nextState.velocity)));

      Logger.recordOutput(getName() + "/targetMotionProfiledAngleRad", nextState.position);
      Logger.recordOutput(
          getName() + "/targetMotionProfiledAngleDegrees",
          Units.radiansToDegrees(nextState.position));

      Logger.recordOutput(getName() + "/targetMotionProfiledVelocityRadPerSec", nextState.velocity);
      Logger.recordOutput(
          getName() + "/targetMotionProfiledVelocityDegreesPerSec",
          Units.radiansToDegrees(nextState.velocity));

      nextState = motionProfile.calculate(0.02, nextState, targetState);
    }

    super.periodic();
    Logger.recordOutput(getName() + "/targetAngleRad", targetAngleRad);
    Logger.recordOutput(getName() + "/targetAngleDegrees", targetAngleDegrees);

    // We get the current angle of the motor and update the 2D sim accordingly
    armSegment.setAngle(Units.radiansToDegrees(inputs.positionRad));
  }

  @Override
  public ArmSettings getSettings() {
    return settings;
  }

  @Override
  public double getCurrentAngle() {
    return Units.radiansToDegrees(inputs.positionRad);
  }

  @Override
  public void setTargetAngle(double degrees) {
    this.targetAngleDegrees = degrees;
    this.targetAngleRad = Units.degreesToRadians(targetAngleDegrees);

    targetState = new State(this.targetAngleRad, 0);
    nextState =
        motionProfile.calculate(
            0.02, new State(inputs.positionRad, inputs.velocityDegreesPerSec), targetState);
    motionProfileEnabled = true;
  }

  @Override
  public double getTargetAngle() {
    return this.targetAngleDegrees;
  }

  @Override
  public boolean isAtMaxLimit() {
    return (getTargetAngle() >= settings.maxAngleInDegrees);
  }

  @Override
  public boolean isAtMinLimit() {
    return (getTargetAngle() <= settings.minAngleInDegrees);
  }

  @Override
  public boolean isAtSetpoint() {
    return (Math.abs(getCurrentAngle() - this.targetAngleDegrees)
        <= settings.targetAngleToleranceInDegrees);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty(
        "motionProfile/maxVelocity (deg per sec)", this::getMaxVelocity, this::setMaxVelocity);
    builder.addDoubleProperty(
        "motionProfile/maxAcceleration (deg per sec^2)",
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
    if (null != motionProfileConstraintsDegrees) {
      return motionProfileConstraintsDegrees.maxVelocity;
    } else {
      return 0;
    }
  }

  private void setMaxVelocity(double velocityDegreesPerSec) {
    if (motionProfileConstraintsDegrees.maxVelocity != velocityDegreesPerSec)
      setMotionProfileConstraintsDegrees(
          new Constraints(velocityDegreesPerSec, motionProfileConstraintsDegrees.maxAcceleration));
  }

  private double getMaxAcceleration() {
    if (null != motionProfileConstraintsDegrees) {
      return motionProfileConstraintsDegrees.maxAcceleration;
    } else {
      return 0;
    }
  }

  private void setMaxAcceleration(double accelerationDegreesPerSecSquared) {
    if (motionProfileConstraintsDegrees.maxAcceleration != accelerationDegreesPerSecSquared)
      setMotionProfileConstraintsDegrees(
          new Constraints(
              motionProfileConstraintsDegrees.maxVelocity, accelerationDegreesPerSecSquared));
  }

  private void setMotionProfileConstraintsDegrees(Constraints motionProfileConstraintsDegrees) {
    this.motionProfileConstraintsDegrees = motionProfileConstraintsDegrees;
    setMotionProfileConstraintsRad(
        new Constraints(
            Units.degreesToRadians(motionProfileConstraintsDegrees.maxVelocity),
            Units.degreesToRadians(motionProfileConstraintsDegrees.maxAcceleration)));
  }

  private void setFFKs(double Ks) {
    settings.feedforward =
        new ArmFeedforward(
            Ks,
            settings.feedforward.getKg(),
            settings.feedforward.getKv(),
            settings.feedforward.getKa());
  }

  private void setFFKg(double Kg) {
    settings.feedforward =
        new ArmFeedforward(
            settings.feedforward.getKs(),
            Kg,
            settings.feedforward.getKv(),
            settings.feedforward.getKa());
  }

  private void setFFKv(double Kv) {
    settings.feedforward =
        new ArmFeedforward(
            settings.feedforward.getKs(),
            settings.feedforward.getKg(),
            Kv,
            settings.feedforward.getKa());
  }

  private void setFFKa(double Ka) {
    settings.feedforward =
        new ArmFeedforward(
            settings.feedforward.getKs(),
            settings.feedforward.getKg(),
            settings.feedforward.getKv(),
            Ka);
  }
}
