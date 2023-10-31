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
        // TODO: Initialize motor parameters
    }

    @Override
    public void periodic() {
        // TODO: Drive motor based on state
    }

    public void setState(IntakeState state) {
        this.state = state;
    }

    public IntakeState getState() {
        return state;
    }
}
