// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.implementations.led;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

  private final AddressableLED led;
  private final AddressableLEDBuffer ledBuffer;

  private final LEDPattern orange = LEDPattern.solid(Color.kOrange);
  private final LEDPattern red = LEDPattern.solid(Color.kRed);
  private final LEDPattern green = LEDPattern.solid(Color.kGreen);
  private final LEDPattern blue = LEDPattern.solid(Color.kBlue);
  private final LEDPattern purple = LEDPattern.solid(Color.kPurple);

  // Our LED strip has a density of 120 LEDs per meter
  private static final Distance kLedSpacing = Meters.of(1 / 120.0);

  // Create an LED pattern that will display a rainbow across
  // all hues at maximum saturation and half brightness
  private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);

  // Create a new pattern that scrolls the rainbow pattern across the LED strip, moving at a speed
  // of 1 meter per second.
  private final LEDPattern scrollingRainbow =
      m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    led = new AddressableLED(0);
    // Two strips of 60 leds joined together
    ledBuffer = new AddressableLEDBuffer(120);
    led.setLength(ledBuffer.getLength());

    // AddressableLEDBufferView left = ledBuffer.createView(0, 30);
    // AddressableLEDBufferView center = ledBuffer.createView(31, 90);
    // AddressableLEDBufferView right = ledBuffer.createView(91, 119);

    // Set the data
    led.setData(ledBuffer);
    led.start();

    setDefaultCommand(idleMode());

    String cmdPrefix = "LEDS/Commands/";

    SmartDashboard.putData(cmdPrefix + "redLED", red());
    SmartDashboard.putData(cmdPrefix + "blueLED", blue());
    SmartDashboard.putData(cmdPrefix + "purpleLED", purple());

    SmartDashboard.putData(cmdPrefix + "redBlinkLED", redBlink());
    SmartDashboard.putData(cmdPrefix + "orangeBlinkLED", orangeBlink());
    SmartDashboard.putData(cmdPrefix + "greenBlinkLED", greenBlink());

    SmartDashboard.putData(this);
  }

  @Override
  public void periodic() {

    led.setData(ledBuffer); // Update the LED strip
  }

  private void setBlink(LEDPattern color, double interval) {
    LEDPattern pattern = color.blink(Units.Seconds.of(interval));
    pattern.applyTo(ledBuffer); // Apply  color
    // led.setData(ledBuffer); // Update the LED strip
  }

  private void setOrangeBlink() {
    setBlink(orange, .5);
  }

  private void setRedBlink() {
    setBlink(red, .5);
  }

  private void setGreenBlink() {
    setBlink(green, .5);
  }

  private void setPurpleBlink() {
    setBlink(purple, .5);
  }

  private void setPurple() {
    purple.applyTo(ledBuffer); // Apply  color
  }

  public Command runPattern(LEDPattern pattern) {
    return new RunCommand(
        () -> {
          pattern.applyTo(ledBuffer);
        },
        this); // Ensure LEDSubsystem is required
  }

  public Command purple() {
    return runPattern(purple).withName("purple");
  }

  public Command red() {
    return runPattern(red).withName("red");
  }

  public Command blue() {
    return runPattern(blue).withName("blue");
  }

  public Command green() {
    return runPattern(green).withName("green");
  }

  public Command orangeBlink() {

    return new RunCommand(
            () -> {
              setOrangeBlink();
            },
            this)
        .withName("orangeBlink");
  }

  public Command redBlink() {
    // this is to ensure LEDSubsystem is required
    return new RunCommand(
            () -> {
              setRedBlink();
            },
            this)
        .withName("redBlink");
  }

  public Command greenBlink() {
    // this is to ensure LEDSubsystem is required
    return new RunCommand(
            () -> {
              setGreenBlink();
            },
            this)
        .withName("greenBlink");
  }

  public Command idleMode() {
    return new RunCommand(
            () -> {
              if (DriverStation.isDisabled()) {
                setOrangeBlink();
              } else {
                setPurpleBlink();
              }
            },
            this)
        .withName("ledIdle")
        .ignoringDisable(true);
  }
}
