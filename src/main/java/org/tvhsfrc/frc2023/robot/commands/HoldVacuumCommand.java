package org.tvhsfrc.frc2023.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.tvhsfrc.frc2023.robot.subsystems.VacuumSubsystem;

/**
 * Runs until canceled.
 * Keeps vacuum on while running.
 */
public class HoldVacuumCommand extends CommandBase {
    private final VacuumSubsystem vacuumSubsystem;

    public HoldVacuumCommand(VacuumSubsystem vacuumSubsystem) {
        this.vacuumSubsystem = vacuumSubsystem;

        addRequirements(vacuumSubsystem);
    }

    @Override
    public void initialize() {
        vacuumSubsystem.enable();
    }

    @Override
    public void end(boolean interrupted) {
        vacuumSubsystem.disable();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
