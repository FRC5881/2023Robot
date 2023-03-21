package org.tvhsfrc.frc2023.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.tvhsfrc.frc2023.robot.Constants;
import org.tvhsfrc.frc2023.robot.subsystems.ArmSubsystem;

public class ArmWaypointCommand extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private final Constants.WayPoints wayPoints;

    public ArmWaypointCommand(ArmSubsystem armSubsystem, Constants.WayPoints wayPoints) {
        this.armSubsystem = armSubsystem;
        this.wayPoints = wayPoints;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.setPose(wayPoints.pose);
    }

    @Override
    public void execute() {
        armSubsystem.setPose(wayPoints.pose);
    }

    @Override
    public boolean isFinished() {
        return armSubsystem.isAtPose(wayPoints.pose);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setLastWaypoint(wayPoints);
    }
}
