package org.tvhsfrc.frc2023.robot.subsystems;

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

    public IntakeState get() {
        return state;
    }
}
