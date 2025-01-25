
package frc.robot.io.implementations.motorcontroller;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.io.interfaces.MotorControllerIO;

public class MotorControllerIOSparkMax implements MotorControllerIO {

  private static final double GEAR_RATIO = 1.0;
  private static final double max_voltage = 24.0;
  
  // define SparkMax Controllers
  private final SparkMax SparkMaxController;

  SparkMaxConfig controllerConfig = new SparkMaxConfig();

  public MotorControllerIOSparkMax(int id) {
    SparkMaxController = new SparkMax(id, MotorType.kBrushless);
    // // Set motor to brake mode so shooter stops spinning immediately
    // // Last thing we do is save all settings to flash on sparkmax

    controllerConfig
        .inverted(false)
        .smartCurrentLimit(60, 40)
        .secondaryCurrentLimit(20000)
        .idleMode(SparkBaseConfig.IdleMode.kBrake);

    SparkMaxController.configure(
        controllerConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);
  }

  @Override
  public void updateInputs(MotorControllerIOInputs inputs) {

    // Get applied voltage from the top motor
    inputs.appliedVolts = SparkMaxController.getAppliedOutput() * SparkMaxController.getBusVoltage();
    // Get applied voltage from the top motor
    inputs.current = SparkMaxController.getOutputCurrent();
  }

  public boolean supportsHardwarePid() {
    return false;
  }

  @Override
  public void setVoltage(double volts) {
    // Set the voltage output for the top motor
    SparkMaxController.setVoltage(volts);
  }
}
