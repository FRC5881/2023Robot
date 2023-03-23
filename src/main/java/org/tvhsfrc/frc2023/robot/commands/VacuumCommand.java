package org.tvhsfrc.frc2023.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.tvhsfrc.frc2023.robot.subsystems.VacuumSubsystem;

/**
 * Starts the vacuum pumps and closes the dump valve while the command is running. On end of
 * command, turns off the vacuum and opens the dump valve.
 */
public class VacuumCommand extends CommandBase {
    private final VacuumSubsystem vacuumSubsystem;

    public VacuumCommand(VacuumSubsystem vacuumSubsystem) {
        this.vacuumSubsystem = vacuumSubsystem;
        addRequirements(vacuumSubsystem);
    }

    @Override
    public void initialize() {
        vacuumSubsystem.vacuum();
    }

    @Override
    public void end(boolean interrupted) {
        vacuumSubsystem.dump();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
