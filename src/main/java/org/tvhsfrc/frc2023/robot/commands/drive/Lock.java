package org.tvhsfrc.frc2023.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.tvhsfrc.frc2023.robot.subsystems.SwerveSubsystem;

public class Lock extends CommandBase {
    private final SwerveSubsystem swerveSubsystem;

    public Lock(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        swerveSubsystem.lock();
    }
}
