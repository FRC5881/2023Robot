package org.tvhsfrc.frc2023.robot;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.Pair;
import java.util.ArrayList;
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
    void safeToLowCone() {
        var path = ArmSubsystem.dijkstra(WAYPOINT.SAFE, WAYPOINT.LOW_CONE);

        assertEquals(2, path.size());
        assertEquals(WAYPOINT.SAFE, path.get(0));
        assertEquals(WAYPOINT.LOW_CONE, path.get(1));
    }

    @Test
    void highConeToLowCone() {
        var path = ArmSubsystem.dijkstra(WAYPOINT.HIGH_CONE, WAYPOINT.LOW_CONE);

        assertEquals(4, path.size());
        assertEquals(WAYPOINT.HIGH_CONE, path.get(0));
        assertEquals(WAYPOINT.HIGH_CONE_MIDPOINT, path.get(1));
        assertEquals(WAYPOINT.SAFE, path.get(2));
        assertEquals(WAYPOINT.LOW_CONE, path.get(3));
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

    // This test checks that the code never crashes
    @Test
    void allPathsWork() {
        for (var start : WAYPOINT.values()) {
            for (var end : WAYPOINT.values()) {
                ArmSubsystem.dijkstra(start, end);
            }
        }
    }

    // Every node is at most 4 nodes away from every other node
    @Test
    void allDistancesShort() {
        for (var start : WAYPOINT.values()) {
            for (var end : WAYPOINT.values()) {
                var path = ArmSubsystem.dijkstra(start, end);

                System.out.println("start: " + start + " end: " + end + " path: " + path);

                assertTrue(path.size() <= 4);
            }
        }
    }

    // Verifies that the path from a to b is at least as long as the direct distance between a and b
    @Test
    void allDistancesCorrect() {
        for (var start : WAYPOINT.values()) {
            for (var end : WAYPOINT.values()) {
                var path = ArmSubsystem.dijkstra(start, end);
                double distance = 0;
                for (int i = 0; i < path.size() - 1; i++) {
                    distance += ArmSubsystem.distance(path.get(i), path.get(i + 1));
                }

                double directDistance = ArmSubsystem.distance(start, end);

                // Direct distance must be greater than or equal to the path distance
                assertTrue(directDistance - distance < 0.001);
            }
        }
    }

    // Triangle inequality
    // distance(a, c) <= distance(a, b) + distance(b, c)
    @Test
    void triangleInequality() {
        for (WAYPOINT a : WAYPOINT.values()) {
            for (WAYPOINT b : WAYPOINT.values()) {
                for (WAYPOINT c : WAYPOINT.values()) {
                    var pathA = ArmSubsystem.dijkstra(a, b);
                    var pathB = ArmSubsystem.dijkstra(b, c);
                    var pathC = ArmSubsystem.dijkstra(a, c);

                    double distanceA = 0;
                    for (int i = 0; i < pathA.size() - 1; i++) {
                        distanceA += ArmSubsystem.distance(pathA.get(i), pathA.get(i + 1));
                    }

                    double distanceB = 0;
                    for (int i = 0; i < pathB.size() - 1; i++) {
                        distanceB += ArmSubsystem.distance(pathB.get(i), pathB.get(i + 1));
                    }

                    double distanceC = 0;
                    for (int i = 0; i < pathC.size() - 1; i++) {
                        distanceC += ArmSubsystem.distance(pathC.get(i), pathC.get(i + 1));
                    }

                    System.out.println("a: " + a + " b: " + b + " c: " + c);
                    System.out.println("a + b: " + (distanceA + distanceB) + " c: " + distanceC);

                    assertTrue(distanceC - (distanceA + distanceB) < 0.01);
                }
            }
        }
    }

    // No waypoint should have itself as a neighbor
    @Test
    void noSelfNeighbors() {
        for (var waypoint : WAYPOINT.values()) {
            System.out.println(waypoint);
            assertFalse(Constants.Arm.ADJACENCY_LIST.get(waypoint).contains(waypoint));
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

    // Prints the shortest paths with the greatest difference between the direct distance and the
    // path distance
    @Test
    void printWorstCaseDistances() {
        ArrayList<Pair<ArrayList<WAYPOINT>, Double>> paths = new ArrayList<>();

        WAYPOINT[] values = WAYPOINT.values();

        for (int i = 0; i < values.length; i++) {
            for (int j = i; j < values.length; j++) {
                var path = ArmSubsystem.dijkstra(values[i], values[j]);
                double distance = 0;
                for (int k = 0; k < path.size() - 1; k++) {
                    distance += ArmSubsystem.distance(path.get(k), path.get(k + 1));
                }

                double directDistance = ArmSubsystem.distance(values[i], values[j]);

                paths.add(new Pair<>(path, distance - directDistance));
            }
        }

        paths.sort(
                (a, b) -> {
                    if (a.getSecond() > b.getSecond()) {
                        return -1;
                    } else if (a.getSecond() < b.getSecond()) {
                        return 1;
                    } else {
                        return 0;
                    }
                });

        for (int i = 0; i < 10; i++) {
            System.out.println(paths.get(i).getFirst() + " " + paths.get(i).getSecond());
        }
    }
}
