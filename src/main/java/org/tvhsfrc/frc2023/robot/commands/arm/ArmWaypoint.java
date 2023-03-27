package org.tvhsfrc.frc2023.robot.commands.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.tvhsfrc.frc2023.robot.Constants.WAYPOINT;
import org.tvhsfrc.frc2023.robot.subsystems.ArmSubsystem;
import org.tvhsfrc.frc2023.robot.utils.Triple;

public class ArmWaypoint extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private final WAYPOINT waypoint;

    public ArmWaypoint(ArmSubsystem armSubsystem, WAYPOINT waypoint) {
        this.armSubsystem = armSubsystem;
        this.waypoint = waypoint;

        SmartDashboard.putString("Arm Waypoint", waypoint.toString());

        addRequirements(armSubsystem);
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
        return armSubsystem.isAtSetPoint();
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setPreviousArmWaypoint(waypoint);
    }
}
