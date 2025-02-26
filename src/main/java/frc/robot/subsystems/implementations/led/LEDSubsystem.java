package frc.robot.subsystems.implementations.led;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.interfaces.LED;

public class LEDSubsystem extends SubsystemBase implements LED {
  private final AddressableLED led;
  private final AddressableLEDBuffer buffer;
  private final int startingPos;
  private final int endingPos;
  private LEDPattern animationPattern;
  private boolean animation = false;

  public LEDSubsystem(
      AddressableLED led, AddressableLEDBuffer buffer, int startingPos, int endingPos) {
    this.led = led;
    this.buffer = buffer;
    this.startingPos = startingPos;
    this.endingPos = endingPos;
  }

  @Override
  public void setColor(Color color) {
    animation = false;
    LEDPattern pattern = LEDPattern.solid(color);
    AddressableLEDBufferView bufferView = buffer.createView(startingPos, endingPos);
    pattern.applyTo(bufferView);
    led.setData(buffer);
  }

  public void startRainbow(int saturation, int value, int speed) {
    animationPattern = LEDPattern.rainbow(saturation, value);
    Distance kLedSpacing = Units.Meters.of(1 / 36.0);
    animationPattern = animationPattern.scrollAtAbsoluteSpeed(Units.MetersPerSecond.of(speed), kLedSpacing);
    animation = true;
  }

  @Override
  public void periodic() {
      if(animation && animationPattern != null) {
        AddressableLEDBufferView bufferView = buffer.createView(startingPos, endingPos);
        animationPattern.applyTo(bufferView);
        led.setData(buffer);
      }
  }
}
