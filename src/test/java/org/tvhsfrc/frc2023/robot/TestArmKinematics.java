package org.tvhsfrc.frc2023.robot;

import static org.tvhsfrc.frc2023.robot.Constants.Arm.STAGE_1_LENGTH;
import static org.tvhsfrc.frc2023.robot.Constants.Arm.STAGE_2_LENGTH;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import org.junit.jupiter.api.Test;
import org.tvhsfrc.frc2023.robot.subsystems.ArmSubsystem;

class TestArmKinematics {
    @Test
    void IsInverses() {
        for (int i = 0; i < 100; i++) {
            // Generate random angles within safe range
            double angle1 =
                    Math.random() * (Constants.Arm.STAGE_1_LIMIT - Constants.Arm.STAGE_1_HOME)
                            + Constants.Arm.STAGE_1_HOME;
            double angle2 =
                    Math.random() * (Constants.Arm.STAGE_2_LIMIT - Constants.Arm.STAGE_2_HOME)
                            + Constants.Arm.STAGE_2_HOME;

            // Convert to rotations
            Rotation2d r1 = Rotation2d.fromRotations(angle1);
            Rotation2d r2 = Rotation2d.fromRotations(angle2);

            // Test
            Translation2d Translation = ArmSubsystem.forwardKinematics(r1, r2);

            Pair<Rotation2d, Rotation2d> angles = ArmSubsystem.inverseKinematics(Translation);

            System.out.println("Angles:" + angle1 + ", " + angle2);
            System.out.println(
                    "Inverse:"
                            + angles.getFirst().getRotations()
                            + ", "
                            + angles.getSecond().getRotations());

            assert (angles.getFirst().equals(r1));
            assert (angles.getSecond().equals(r2));
        }
    }

    @Test
    void knownPoints() {
        var pairs = new ArrayList<Pair<Pair<Double, Double>, Translation2d>>();
        pairs.add(
                new Pair<>(
                        new Pair<>(0.0, 90.0), new Translation2d(STAGE_2_LENGTH, STAGE_1_LENGTH)));
        pairs.add(
                new Pair<>(
                        new Pair<>(0.0, 90.0), new Translation2d(STAGE_2_LENGTH, STAGE_1_LENGTH)));
        pairs.add(
                new Pair<>(
                        new Pair<>(0.0, 0.0),
                        new Translation2d(0, STAGE_1_LENGTH - STAGE_2_LENGTH)));

        for (var pair : pairs) {
            var angles = pair.getFirst();
            var pose = pair.getSecond();

            var r1 = Rotation2d.fromDegrees(angles.getFirst());
            var r2 = Rotation2d.fromDegrees(angles.getSecond());

            var calculatedPose = ArmSubsystem.forwardKinematics(r1, r2);

            System.out.println("Angles: " + angles.getFirst() + ", " + angles.getSecond());
            System.out.println("Pose: " + pose);
            System.out.println("Calculated: " + calculatedPose);
            System.out.println();

            assert (calculatedPose.equals(pose));
        }
    }
}
