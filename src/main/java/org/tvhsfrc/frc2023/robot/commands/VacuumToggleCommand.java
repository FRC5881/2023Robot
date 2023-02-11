package org.tvhsfrc.frc2023.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.tvhsfrc.frc2023.robot.subsystems.VacuumSubsystem;

/** Toggles on or off the vacuum. Stops immediately! */
public class VacuumToggleCommand extends CommandBase {
    private final VacuumSubsystem vacuumSubsystem;

    public VacuumToggleCommand(VacuumSubsystem vacuumSubsystem) {
        this.vacuumSubsystem = vacuumSubsystem;

        addRequirements(vacuumSubsystem);
    }

    @Override
    public void initialize() {
        vacuumSubsystem.toggle();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
