package org.tvhsfrc.frc2023.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.tvhsfrc.frc2023.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    public enum IntakeState {
        IN,
        OUT,
        STOPPED
    }

    private IntakeState state = IntakeState.STOPPED;

    private final CANSparkMax intakeSparkMax =
            new CANSparkMax(Constants.CANConstants.INTAKE, CANSparkMax.MotorType.kBrushless);

    public IntakeSubsystem() {
        // Motor controller setup for the intake
        intakeSparkMax.restoreFactoryDefaults();

        intakeSparkMax.setInverted(true);
        intakeSparkMax.setIdleMode(CANSparkMax.IdleMode.kBrake);

        intakeSparkMax.burnFlash();
    }

    @Override
    public void periodic() {
        switch (state) {
            case IN:
                intakeSparkMax.set(Constants.Intake.speedIN);
                break;
            case OUT:
                intakeSparkMax.set(Constants.Intake.speedOUT);
                break;
            case STOPPED:
                intakeSparkMax.set(0);
                break;
        }
    }

    public void setState(IntakeState state) {
        this.state = state;
    }

    public IntakeState getState() {
        return state;
    }
}
