package org.tvhsfrc.frc2023.robot.commands.vacuum;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.tvhsfrc.frc2023.robot.subsystems.VacuumSubsystem;

/** Stops the vacuum pump and temporarily opens the dump valve */
public class VacuumDisableCommand extends CommandBase {
    private final VacuumSubsystem vacuumSubsystem;

    private Timer timer = new Timer();
    private final double duration;

    public VacuumDisableCommand(VacuumSubsystem vacuumSubsystem, double duration) {
        addRequirements(vacuumSubsystem);

        this.vacuumSubsystem = vacuumSubsystem;
        this.duration = duration;
    }

    @Override
    public void initialize() {
        timer.restart();

        vacuumSubsystem.suction(false);
        vacuumSubsystem.purge(true);
        vacuumSubsystem.setState(false);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(duration);
    }

    @Override
    public void end(boolean interrupted) {
        vacuumSubsystem.purge(false);
    }
}
