package frc.robot.subsystems.implementations.motor;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.io.interfaces.MotorIO;
import frc.robot.subsystems.interfaces.Flywheel;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class FlywheelMotorSubsystem extends MotorSubsystem implements Flywheel {
  private final FlywheelSettings settings;
  private double targetVelocityRPMs = 0;
  private double targetVelocityRadPerSec = 0;

  /* 2D Simulation */
  private final Mechanism2d mech2d;
  public List<MechanismLigament2d> flywheelSegment =
      new ArrayList<
          MechanismLigament2d>(); /* contains the pinwheel fin objects so that the positions can be updated based on the current position */
  private double currentSimAngle = 0;

  public FlywheelMotorSubsystem(MotorIO io, String name, FlywheelSettings settings) {
    super(io, "Flywheel[" + name + "]");
    this.settings = settings;

    double sim2dSize = 64;
    mech2d = new Mechanism2d(sim2dSize, sim2dSize);

    // We create a pivot point where the flysheel segments are attached
    MechanismRoot2d flywheelPivot2d =
        mech2d.getRoot(getName() + " Pivot", sim2dSize / 2, sim2dSize / 2);

    for (int i = 0; i < 4; i++) {
      flywheelSegment.add(
          flywheelPivot2d.append(
              new MechanismLigament2d(
                  getName() + " Segment " + i,
                  sim2dSize / 2,
                  currentSimAngle + i * (360 / 4),
                  15,
                  settings.color)));
    }

    SmartDashboard.putData(getName() + "/2D Simulation", mech2d);
  }

  @Override
  public void periodic() {
    super.periodic();
    Logger.recordOutput(getName() + "/targetVelocityRPMs", targetVelocityRPMs);
    Logger.recordOutput(getName() + "/targetVelocityRadPerSec", targetVelocityRadPerSec);

    // We get the current angle of the motor and update the 2D sim accordingly
    int angleOffset = 0;

    // We loop through each fin and update the angle to match the reported motor rotation

    currentSimAngle += (inputs.velocityRPMs / 20000) * 45;
    for (MechanismLigament2d segment : flywheelSegment) {
      segment.setAngle(angleOffset + currentSimAngle);
      angleOffset += 360 / 4;
    }
  }

  @Override
  public FlywheelSettings getSettings() {
    return settings;
  }

  @Override
  public double getCurrentVelocity() {
    return Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
  }

  @Override
  public void setTargetVelocity(double velocityRPMs) {
    this.targetVelocityRPMs = velocityRPMs;
    this.targetVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(this.targetVelocityRPMs);
    io.setVelocity(
        this.targetVelocityRadPerSec, settings.feedforward.calculate(this.targetVelocityRadPerSec));
  }

  @Override
  public double getTargetVelocity() {
    return this.targetVelocityRPMs;
  }

  @Override
  public boolean isAtSetpoint() {
    return (Math.abs(getCurrentVelocity() - this.targetVelocityRPMs)
        <= settings.targetVelocityToleranceInRPMs);
  }
}
