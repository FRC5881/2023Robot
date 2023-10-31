package org.tvhsfrc.frc2023.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.tvhsfrc.frc2023.robot.subsystems.IntakeSubsystem;
import org.tvhsfrc.frc2023.robot.subsystems.IntakeSubsystem.IntakeState;

public class IntakeIn extends CommandBase {
    private final IntakeSubsystem intake;

    public IntakeIn(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setState(IntakeState.IN);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setState(IntakeState.STOPPED);
    }
}
