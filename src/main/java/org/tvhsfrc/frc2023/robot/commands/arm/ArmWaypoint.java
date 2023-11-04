package org.tvhsfrc.frc2023.robot.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.tvhsfrc.frc2023.robot.Constants;
import org.tvhsfrc.frc2023.robot.Constants.WAYPOINT;
import org.tvhsfrc.frc2023.robot.subsystems.ArmSubsystem;

public class ArmWaypoint extends CommandBase {    
    private static final double TIMEOUT = 2.0;
    private static final double dt = 0.02;

    private final ArmSubsystem arm;
    private final WAYPOINT waypoint;

    private TrapezoidProfile profile;
    private double time = 0;

    public ArmWaypoint(ArmSubsystem arm, WAYPOINT waypoint) {
        this.arm = arm;
        this.waypoint = waypoint;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        TrapezoidProfile.State initial = arm.getStage2Position();
        TrapezoidProfile.State goal = waypoint.getStage2State();
        profile = new TrapezoidProfile(Constants.Arm.STAGE_2_CONSTRAINTS, goal, initial);
    }

    @Override
    public void execute() {
        time += dt;
        TrapezoidProfile.State setpoint = profile.calculate(time);
        arm.setStage2Setpoint(Rotation2d.fromRotations(setpoint.position));
    }

    @Override
    public boolean isFinished() {
        return arm.isAtSetPoint() || time > TIMEOUT;
    }

    @Override
    public void end(boolean interrupted) {}
}
