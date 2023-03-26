package org.tvhsfrc.frc2023.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.tvhsfrc.frc2023.robot.Constants.WAYPOINT;
import org.tvhsfrc.frc2023.robot.subsystems.ArmSubsystem;

public class ArmNext extends CommandBase {
    private final ArmSubsystem armSubsystem;

    public ArmNext(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;
    }

    @Override
    public void initialize() {
        WAYPOINT previous = armSubsystem.getPreviousArmWaypoint();
        WAYPOINT waypoint = armSubsystem.waypointTarget();

        System.out.println("ArmNext: " + previous.toString() + " " + waypoint);

        CommandBase c = armSubsystem.buildPath(previous, waypoint);
        c.schedule();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
