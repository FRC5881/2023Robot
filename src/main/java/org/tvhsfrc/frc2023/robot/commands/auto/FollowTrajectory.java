package org.tvhsfrc.frc2023.robot.commands.auto;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.tvhsfrc.frc2023.robot.Constants.Autonomous;
import org.tvhsfrc.frc2023.robot.subsystems.SwerveSubsystem;

public class FollowTrajectory extends SequentialCommandGroup {

    public FollowTrajectory(
            SwerveSubsystem drivebase, PathPlannerTrajectory trajectory, boolean resetOdometry) {
        addRequirements(drivebase);

        if (resetOdometry) {
            drivebase.resetOdometry(trajectory.getInitialHolonomicPose());
        }

        addCommands(
                new PPSwerveControllerCommand(
                        trajectory,
                        drivebase::getPose,
                        Autonomous.xAutoPID.createPIDController(),
                        Autonomous.yAutoPID.createPIDController(),
                        Autonomous.angleAutoPID.createPIDController(),
                        drivebase::setChassisSpeeds,
                        drivebase));
    }
}
