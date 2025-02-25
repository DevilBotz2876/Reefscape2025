package frc.robot.config.game.reefscape2025;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class RobotConfigRoadkill extends RobotConfig {
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;

  // private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);
  // private static final Distance kLedSpacing = Meters.of(1 / 0.6);
  //  // Our LED strip has a density of 120 LEDs per meter

  //  // Create a new pattern that scrolls the rainbow pattern across the LED strip, moving at a
  // speed
  //  // of 1 meter per second.
  //  private final LEDPattern m_scrollingRainbow =
  //      m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);

  public RobotConfigRoadkill() {
    super(true, true, true);

    // System.out.print("This is the Robot Config for the Roadkill");
    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(0);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(34);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
    // Create an LED pattern that sets the entire strip to solid red
    LEDPattern red = LEDPattern.solid(Color.kRed);

    // Apply the LED pattern to the data buffer
    red.applyTo(m_ledBuffer);

    // Write the data to the LED strip
    m_led.setData(m_ledBuffer);
  }
}
