package org.tvhsfrc.frc2023.robot;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;
import org.tvhsfrc.frc2023.robot.Constants.WAYPOINT;
import org.tvhsfrc.frc2023.robot.subsystems.ArmSubsystem;

class TestArmPathFinding {
    @Test
    void homeToSafe() {
        var path = ArmSubsystem.dijkstra(WAYPOINT.HOME, WAYPOINT.SAFE);

        assertEquals(2, path.size());
        assertEquals(WAYPOINT.HOME, path.get(0));
        assertEquals(WAYPOINT.SAFE, path.get(1));
    }

    @Test
    void homeToHighCone() {
        var path = ArmSubsystem.dijkstra(WAYPOINT.HOME, WAYPOINT.HIGH_CONE);

        assertEquals(3, path.size());
        assertEquals(WAYPOINT.HOME, path.get(0));
        assertEquals(WAYPOINT.HIGH_CONE_MIDPOINT, path.get(1));
        assertEquals(WAYPOINT.HIGH_CONE, path.get(2));
    }

    @Test
    void safeToHighCone() {
        var path = ArmSubsystem.dijkstra(WAYPOINT.SAFE, WAYPOINT.HIGH_CONE);

        assertEquals(3, path.size());
        assertEquals(WAYPOINT.SAFE, path.get(0));
        assertEquals(WAYPOINT.HIGH_CONE_MIDPOINT, path.get(1));
        assertEquals(WAYPOINT.HIGH_CONE, path.get(2));
    }

    @Test
    void homeToLowCone() {
        var path = ArmSubsystem.dijkstra(WAYPOINT.HOME, WAYPOINT.LOW_CONE);

        assertEquals(3, path.size());
        assertEquals(WAYPOINT.HOME, path.get(0));
        assertEquals(WAYPOINT.SAFE, path.get(1));
        assertEquals(WAYPOINT.LOW_CONE, path.get(2));
    }

    @Test
    void allPathsWork() {
        // This test checks that the code never crashes
        for (var start : WAYPOINT.values()) {
            for (var end : WAYPOINT.values()) {
                ArmSubsystem.dijkstra(start, end);
            }
        }
    }

    // Print out all of the distances
    @Test
    void printDistances() {
        for (var start : WAYPOINT.values()) {
            for (var end : WAYPOINT.values()) {
                var path = ArmSubsystem.dijkstra(start, end);
                System.out.println(start + " to " + end + " = " + path.size());

                // print out the path
                System.out.println("Path: " + path.toString());

                // calculate the distance
                double distance = 0;
                for (int i = 0; i < path.size() - 1; i++) {
                    distance += ArmSubsystem.distance(path.get(i), path.get(i + 1));
                }

                System.out.println("Distance: " + distance);

                // calculate the direct distance
                double directDistance = ArmSubsystem.distance(start, end);

                System.out.println("Direct Distance: " + directDistance);
            }
        }
    }
}