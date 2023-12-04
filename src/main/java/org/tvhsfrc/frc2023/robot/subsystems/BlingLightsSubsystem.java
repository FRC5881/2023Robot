package org.tvhsfrc.frc2023.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BlingLightsSubsystem extends SubsystemBase {
    public enum Colors {
        GREEN,
        BLUE,
        RAINBOW,
        OFF
    }

    // variables go here
    Colors state = Colors.OFF;

    int m_rainbowFirstPixelHue;

    AddressableLED m_led = new AddressableLED(0);
    // Length is expensive to set, so only set it once, then just update data
    AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(36);

    public BlingLightsSubsystem() {
        m_led.setLength(m_ledBuffer.getLength());

        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    private void rainbow(int m_rainbowFirstPixelHue) {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
            m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        m_rainbowFirstPixelHue += 3;
        m_rainbowFirstPixelHue %= 180;
    }

    private void rgbPicker(int int1, int int2, int int3) {
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, int1, int2, int3);
        }
    }

    @Override
    public void periodic() {
        switch (state) {
            default:
                break;
            case GREEN:
                rgbPicker(0, 255, 32);
                break;
            case BLUE:
                rgbPicker(19, 0, 255);
                break;
            case RAINBOW:
                rainbow(1);
                break;
            case OFF:
                break;
        }
    }

    // Getter and Setter
    public Colors getState() {
        return state;
    }

    public void setState(Colors state) {
        this.state = state;
    }
}
