package org.tvhsfrc.frc2023.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.tvhsfrc.frc2023.robot.subsystems.VacuumSubsystem;

/**
 * Starts the vacuum pumps and closes the dump valve
 */
public class VacuumEnableCommand extends CommandBase {
    private final VacuumSubsystem vacuumSubsystem;

    public VacuumEnableCommand(VacuumSubsystem vacuumSubsystem) {
        this.vacuumSubsystem = vacuumSubsystem;
        addRequirements(vacuumSubsystem);
    }

    @Override
    public void initialize() {
        vacuumSubsystem.suck(true);
        vacuumSubsystem.dump(false);
        vacuumSubsystem.setState(true);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
