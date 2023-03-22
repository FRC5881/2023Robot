package org.tvhsfrc.frc2023.robot.commands.vacuum;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import org.tvhsfrc.frc2023.robot.Constants;
import org.tvhsfrc.frc2023.robot.subsystems.VacuumSubsystem;

// Toggles the vacuum subsystem on and off
public class VacuumToggleCommand extends ConditionalCommand {
    public VacuumToggleCommand(VacuumSubsystem vacuumSubsystem) {
        super(
                new VacuumEnableCommand(vacuumSubsystem),
                new VacuumDisableCommand(vacuumSubsystem, Constants.Vacuum.PURGE_TIME),
                vacuumSubsystem::getState);
    }
}
