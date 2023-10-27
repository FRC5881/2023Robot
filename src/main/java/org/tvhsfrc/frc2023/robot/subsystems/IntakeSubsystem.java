package org.tvhsfrc.frc2023.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    public enum IntakeState {
        IN,
        OUT,
        STOPPED
    }

    private IntakeState state = IntakeState.STOPPED;

    // TODO: Add motor

    public IntakeSubsystem() {
        // TODO: Initialize motor
    }

    public void set(IntakeState state) {
        this.state = state;

        // TODO: Control motor
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        // TODO: Add smart dashboard controls
    }

    public IntakeState get() {
        return state;
    }
}
