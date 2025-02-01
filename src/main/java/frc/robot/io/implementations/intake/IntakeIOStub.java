package frc.robot.io.implementations.intake;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.io.interfaces.IntakeIO;
import frc.robot.subsystems.implementations.intake.IntakeSubsystem;

public class IntakeIOStub implements IntakeIO {

  private static final double kFlywheelGearing = 1.0;

  // 1/2 MR²
  private static final double kFlywheelMomentOfInertia =
      0.5 * Units.lbsToKilograms(1.5) * Math.pow(Units.inchesToMeters(4), 2);

  private final DCMotor gearbox = DCMotor.getNEO(1);

  private final LinearSystem<N1, N1, N1> plant =
      LinearSystemId.createFlywheelSystem(gearbox, kFlywheelGearing, kFlywheelMomentOfInertia);

  private FlywheelSim sim = new FlywheelSim(plant, gearbox);
  private double appliedVolts = 0.0;
  private double elapsedTimeMotorOn = 0.0;
  private boolean limitSwitchState = true; // Start with piece inserted

  @Override
  public void updateInputs(IntakeIOInputs inputs) {

    sim.update(0.02);

    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.limitSwitchIntake = limitSwitchState;
    inputs.current =  sim.getCurrentDrawAmps();

    // Simulate intake/shooter limit switches
    if (appliedVolts != 0.0) {
      elapsedTimeMotorOn += 0.02;

      if ((elapsedTimeMotorOn > IntakeSubsystem.Constants.intakeTimeoutInSeconds / 2)) {
        limitSwitchState = !limitSwitchState;
        elapsedTimeMotorOn = 0.0;
      }
    } else {
      elapsedTimeMotorOn = 0.0;
    }
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
    sim.setInputVoltage(volts);
  }
}
