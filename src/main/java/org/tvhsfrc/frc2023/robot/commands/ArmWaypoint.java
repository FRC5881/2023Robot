package org.tvhsfrc.frc2023.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.tvhsfrc.frc2023.robot.Constants.WAYPOINT;
import org.tvhsfrc.frc2023.robot.subsystems.ArmSubsystem;
import org.tvhsfrc.frc2023.robot.utils.Triple;

public class ArmWaypoint extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private final WAYPOINT waypoint;
    private final boolean hold;

    public ArmWaypoint(ArmSubsystem armSubsystem, WAYPOINT waypoint, boolean hold) {
        this.armSubsystem = armSubsystem;
        this.waypoint = waypoint;
        this.hold = hold;
    }

    @Override
    public void initialize() {
        Triple<Double, Double, Double> position = waypoint.position;

        armSubsystem.setStage1Rotations(position.getA());
        armSubsystem.setStage2Rotations(position.getB());
        armSubsystem.setStage3Rotations(position.getC());
    }

    @Override
    public boolean isFinished() {
        if (hold) {
            return false;
        } else {
            return armSubsystem.isAtSetPoint();
        }
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setPreviousArmWaypoint(waypoint);
    }
}
