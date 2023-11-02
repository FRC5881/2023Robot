package org.tvhsfrc.frc2023.robot.commands.arm;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.tvhsfrc.frc2023.robot.Constants.WAYPOINT;
import org.tvhsfrc.frc2023.robot.subsystems.ArmSubsystem;

public class ArmWaypoint extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private final WAYPOINT waypoint;
    private final boolean instant;

    /**
     * @param armSubsystem
     * @param waypoint
     * @param instant if true then this command will be run instantly, otherwise it will be run
     *     until the arm is at the setpoint
     */
    public ArmWaypoint(ArmSubsystem armSubsystem, WAYPOINT waypoint, boolean instant) {
        this.armSubsystem = armSubsystem;
        this.waypoint = waypoint;
        this.instant = instant;

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        Pair<Rotation2d, Rotation2d> angles = waypoint.getAngle();
        armSubsystem.setSetpoint(angles);
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return instant || armSubsystem.isAtSetPoint();
    }

    @Override
    public void end(boolean interrupted) {}
}
