package org.tvhsfrc.frc2023.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.tvhsfrc.frc2023.robot.Constants;
import org.tvhsfrc.frc2023.robot.utils.Triple;

/** Proof of concept LED subsystem only supporting solid colors. */
public class LEDSubsystem extends SubsystemBase {
    public enum State {
        NONE,
        CUBE,
        CONE,
        DISABLED,
    }

    private AddressableLED leds = new AddressableLED(Constants.LEDs.LED_PWM);
    private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(Constants.LEDs.LED_LENGTH);
    private State state = State.NONE;

    public LEDSubsystem() {
        leds.setLength(ledBuffer.getLength());
        leds.setData(ledBuffer);
        leds.start();
    }

    private void setLEDs(int r, int g, int b) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i, r, g, b);
        }
        leds.setData(ledBuffer);
    }

    private void setLEDs(Triple<Integer, Integer, Integer> rgb) {
        setLEDs(rgb.getA(), rgb.getB(), rgb.getC());
    }

    public void setState(State state) {
        this.state = state;

        switch (state) {
            case NONE:
                setLEDs(0, 0, 0);
                break;
            case CUBE:
                setLEDs(Constants.LEDs.COLOR_CUBE);
                break;
            case CONE:
                setLEDs(Constants.LEDs.COLOR_CONE);
                break;
            case DISABLED:
                setLEDs(Constants.LEDs.COLOR_DISABLED);
                break;
        }
    }

    public State getState() {
        return state;
    }
}
