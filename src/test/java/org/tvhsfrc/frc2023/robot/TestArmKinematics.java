package org.tvhsfrc.frc2023.robot;

import static org.tvhsfrc.frc2023.robot.Constants.Arm.STAGE_1_LENGTH;
import static org.tvhsfrc.frc2023.robot.Constants.Arm.STAGE_2_LENGTH;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.ArrayList;
import org.junit.jupiter.api.Test;
import org.tvhsfrc.frc2023.robot.subsystems.ArmSubsystem;
import org.tvhsfrc.frc2023.robot.utils.Triple;

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
            double angle3 =
                    Math.random() * (Constants.Arm.STAGE_3_LIMIT - Constants.Arm.STAGE_3_HOME)
                            + Constants.Arm.STAGE_3_HOME;

            // Convert to rotations
            Rotation2d r1 = Rotation2d.fromRotations(angle1);
            Rotation2d r2 = Rotation2d.fromRotations(angle2);
            Rotation2d r3 = Rotation2d.fromRotations(angle3);

            // Test
            Pose2d pose = ArmSubsystem.forwardKinematics(r1, r2, r3);

            Triple<Rotation2d, Rotation2d, Rotation2d> angles =
                    ArmSubsystem.inverseKinematics(pose);

            System.out.println("Angles:" + angle1 + ", " + angle2 + ", " + angle3);
            System.out.println(
                    "Inverse:"
                            + angles.getA().getRotations()
                            + ", "
                            + angles.getB().getRotations()
                            + ", "
                            + angles.getC().getRotations());

            assert (angles.getA().equals(r1));
            assert (angles.getB().equals(r2));
            assert (angles.getC().equals(r3));
        }
    }

    @Test
    void knownPoints() {
        var pairs = new ArrayList<Pair<Triple<Double, Double, Double>, Pose2d>>();
        pairs.add(
                new Pair<>(
                        new Triple<>(0.0, 90.0, 0.0),
                        new Pose2d(STAGE_2_LENGTH, STAGE_1_LENGTH, Rotation2d.fromDegrees(-180))));
        pairs.add(
                new Pair<>(
                        new Triple<>(0.0, 90.0, 90.0),
                        new Pose2d(STAGE_2_LENGTH, STAGE_1_LENGTH, Rotation2d.fromDegrees(-90))));
        pairs.add(
                new Pair<>(
                        new Triple<>(0.0, 0.0, 0.0),
                        new Pose2d(
                                0, STAGE_1_LENGTH - STAGE_2_LENGTH, Rotation2d.fromDegrees(90))));

        for (var pair : pairs) {
            var angles = pair.getFirst();
            var pose = pair.getSecond();

            var r1 = Rotation2d.fromDegrees(angles.getA());
            var r2 = Rotation2d.fromDegrees(angles.getB());
            var r3 = Rotation2d.fromDegrees(angles.getC());

            var calculatedPose = ArmSubsystem.forwardKinematics(r1, r2, r3);

            System.out.println(
                    "Angles: " + angles.getA() + ", " + angles.getB() + ", " + angles.getC());
            System.out.println("Pose: " + pose);
            System.out.println("Calculated: " + calculatedPose);
            System.out.println();

            assert (calculatedPose.equals(pose));
        }
    }
}
