package org.tvhsfrc.frc2023.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.ArrayList;
import org.tvhsfrc.frc2023.robot.Constants.WAYPOINT;
import org.tvhsfrc.frc2023.robot.subsystems.ArmSubsystem;

public class ArmTrajectory extends SequentialCommandGroup {
    public ArmTrajectory(ArmSubsystem armSubsystem, ArrayList<WAYPOINT> points) {
        if (points != null) {
            for (int i = 0; i < points.size() - 1; i++) {
                addCommands(new ArmWaypoint(armSubsystem, points.get(i), false));
            }

            addCommands(new ArmWaypoint(armSubsystem, points.get(points.size() - 1), true));
        } else {
            System.out.println("Null points");
        }
    }
}