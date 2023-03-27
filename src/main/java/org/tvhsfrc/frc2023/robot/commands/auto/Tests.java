package org.tvhsfrc.frc2023.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.tvhsfrc.frc2023.robot.Constants.WAYPOINT;
import org.tvhsfrc.frc2023.robot.subsystems.ArmSubsystem;

public class Tests {
    // A routine that tests the arm pathfinding
    public static CommandBase armCycle1(ArmSubsystem arm) {
        // Preform this cycle of waypoints
        WAYPOINT[] cycle = {
            WAYPOINT.HOME,
            WAYPOINT.FLOOR_CONE,
            WAYPOINT.FLOOR_CUBE,
            WAYPOINT.LOW_CUBE,
            WAYPOINT.LOW_CONE,
            WAYPOINT.MID_CUBE,
            WAYPOINT.MID_CONE,
            WAYPOINT.HIGH_CUBE,
            WAYPOINT.HIGH_CONE,
            WAYPOINT.DOUBLE_SUBSTATION_CUBE,
            WAYPOINT.DOUBLE_SUBSTATION_CONE,
            WAYPOINT.HOME
        };

        SequentialCommandGroup group = new SequentialCommandGroup();
        for (int i = 0; i < cycle.length - 1; i++) {
            group.addCommands(arm.buildPath(cycle[i], cycle[i + 1]));
        }
        // Add the reverse path
        for (int i = cycle.length - 1; i > 0; i--) {
            group.addCommands(arm.buildPath(cycle[i], cycle[i - 1]));
        }
        return group;
    }
}
