package org.tvhsfrc.frc2023.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.tvhsfrc.frc2023.robot.subsystems.IntakeSubsystem;

public class IntakeOut extends CommandBase {
    private final IntakeSubsystem intake;

    public IntakeOut(IntakeSubsystem intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        // TODO: Start the intake
    }

    @Override
    public boolean isFinished() {
        // TODO: Is "false" the correct value here?
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // TODO: Stop the intake
    }
}
