package org.tvhsfrc.frc2023.robot.commands.arm;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.tvhsfrc.frc2023.robot.Constants.WAYPOINT;
import org.tvhsfrc.frc2023.robot.subsystems.ArmSubsystem;

public class ArmWaypoint extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private final WAYPOINT waypoint;

    public ArmWaypoint(ArmSubsystem armSubsystem, WAYPOINT waypoint) {
        this.armSubsystem = armSubsystem;
        this.waypoint = waypoint;

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        Pair<Rotation2d, Rotation2d> angles = waypoint.getAngle();

        armSubsystem.setStage1Rotations(angles.getFirst().getRotations());
        armSubsystem.setStage2Rotations(angles.getSecond().getRotations());
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return armSubsystem.isAtSetPoint();
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setPreviousArmWaypoint(waypoint);
    }
}
