package frc.robot.io.implementations.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.io.interfaces.ElevatorIO;
import frc.robot.subsystems.interfaces.Elevator;

public class ElevatorIOStub implements ElevatorIO {
  private final double elevatorGearing = 10.0;
  private final double elevatorCarriageMassInKg = 4.0;
  private final double elevatorDrumRadius = Units.inchesToMeters(2.0);

  private final DCMotor motorPlant = DCMotor.getNEO(1);

  private double currentVolts = 0.0;

  private final ProfiledPIDController pid;
  private boolean softwarePidEnabled = false;
  private double targetMeters = 0;
  private double feedForwardVolts = 0;

  private final ElevatorSim elevatorSim;

  private final TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(
          Elevator.Constants.maxVelocity, Elevator.Constants.maxAcceleration);
  private ElevatorFeedforward ff =
      new ElevatorFeedforward(
          Elevator.Constants.ffKs,
          Elevator.Constants.ffKg,
          Elevator.Constants.ffKv,
          Elevator.Constants.ffKa);

  public ElevatorIOStub() {
    this.pid =
        new ProfiledPIDController(
            Elevator.Constants.pidKp,
            Elevator.Constants.pidKi,
            Elevator.Constants.pidKd,
            constraints,
            Elevator.Constants.pidDt);

    this.elevatorSim =
        new ElevatorSim(
            motorPlant,
            elevatorGearing,
            elevatorCarriageMassInKg,
            elevatorDrumRadius,
            Elevator.Constants.minPositionInMeters,
            Elevator.Constants.maxPositionInMeters,
            true,
            0,
            0.01,
            0.0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.appliedVolts = currentVolts;
    inputs.currentAmps = elevatorSim.getCurrentDrawAmps();
    inputs.positionMeters = elevatorSim.getPositionMeters();
    inputs.velocityInMetersPerSecond = elevatorSim.getVelocityMetersPerSecond();

    inputs.upperLimit = elevatorSim.hasHitUpperLimit();
    inputs.lowerLimit = elevatorSim.hasHitLowerLimit();

    if (softwarePidEnabled) {
      double pidOutput = pid.calculate(inputs.positionMeters);
      feedForwardVolts = ff.calculate(pid.getSetpoint().velocity);

      currentVolts = feedForwardVolts + pidOutput * RobotController.getBatteryVoltage();
    }

    elevatorSim.setInput(currentVolts);
    elevatorSim.update(0.020);
  }

  @Override
  public void setVoltage(double volts) {
    softwarePidEnabled = false;
    currentVolts = volts;
  }

  @Override
  public void setPosition(double meters) {
    targetMeters = meters;
    softwarePidEnabled = true;
    pid.setGoal(targetMeters);
  }
}
