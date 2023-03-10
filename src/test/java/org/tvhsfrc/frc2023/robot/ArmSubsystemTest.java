package org.tvhsfrc.frc2023.robot;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;
import org.tvhsfrc.frc2023.robot.subsystems.ArmSubsystem;

// To learn more about how to write unit tests, see the
// JUnit 5 User Guide at https://junit.org/junit5/docs/current/user-guide/
class ArmSubsystemTest {
    private static final double epsilon = 0.0001;

    // Test the Arm's kinematics functions with a know position
    //
    // * ------- *
    // | 90   90 |
    // |         |
    // |
    // |
    // |
    // * 0
    //
    @Test
    void testArmKinematicsSquare() {
        System.out.println("testArmKinematicsSquare");

        // Test that forward kinematics and inverse kinematics are inverses of each other
        // Stage 1 : vertical
        // Stage 2 : horizontal
        // Stage 3 : vertical
        Rotation2d stage1Angle = Rotation2d.fromDegrees(0);
        Rotation2d stage2Angle = Rotation2d.fromDegrees(90);
        Rotation2d stage3Angle = Rotation2d.fromDegrees(90);

        System.out.println("stage1Angle: " + stage1Angle.getDegrees());
        System.out.println("stage2Angle: " + stage2Angle.getDegrees());
        System.out.println("stage3Angle: " + stage3Angle.getDegrees());

        Pose2d pose = ArmSubsystem.forwardKinematics(stage1Angle, stage2Angle, stage3Angle);
        System.out.println(
                "pose: "
                        + pose.getTranslation().getX()
                        + ", "
                        + pose.getTranslation().getY()
                        + ", "
                        + pose.getRotation().getDegrees());

        assertEquals(Constants.Arm.STAGE_2_LENGTH, pose.getX(), epsilon);
        assertEquals(Constants.Arm.STAGE_1_LENGTH, pose.getY(), epsilon);
        assertEquals(Rotation2d.fromDegrees(270), pose.getRotation());

        var angles = ArmSubsystem.inverseKinematics(pose);
        System.out.println(angles);

        // Flip asserts
        assertEquals(stage1Angle, angles.getA());
        assertEquals(stage2Angle, angles.getB());
        assertEquals(stage3Angle, angles.getC());
    }

    // Randomly align stage2 with stage1. Stage3 points parallel to the ground
    //
    //        * ------- *
    //       /
    //      /
    //     *
    //    /
    //   /
    //  /
    // *
    //
//    @Test
//    void testArmKinematicsExtended() {
//        System.out.println("testArmKinematicsExtended");
//
//         for (int i = 0; i < 100; i++) {
//            System.out.println();
//
//            // Random angle between -10 and LIMIT_DEGREE_STAGE_1
//            Rotation2d stage1Angle =
//                    Rotation2d.fromDegrees(
//                            Math.random() * (Constants.Arm.LIMIT_DEGREE_STAGE_1 - 10) + 10);
//            Rotation2d stage2Angle = Rotation2d.fromDegrees(180);
//            Rotation2d stage3Angle = Rotation2d.fromDegrees(90).plus(stage1Angle);
//
//            System.out.println("stage1Angle: " + stage1Angle.getDegrees());
//            System.out.println("stage2Angle: " + stage2Angle.getDegrees());
//            System.out.println("stage3Angle: " + stage3Angle.getDegrees());
//
//            Pose2d pose = ArmSubsystem.forwardKinematics(stage1Angle, stage2Angle, stage3Angle);
//            System.out.println(
//                    "pose: "
//                            + pose.getTranslation().getX()
//                            + ", "
//                            + pose.getTranslation().getY()
//                            + ", "
//                            + pose.getRotation().getDegrees());
//
//            // Get the complement of stage1Angle
//            Rotation2d stage1Complement = Rotation2d.fromDegrees(90).minus(stage1Angle);
//
//            // Since the arms are aligned we expect the bot to trace out a circle of radius
//            // STAGE_1_LENGTH + STAGE_2_LENGTH
//            double expectedX =
//                    stage1Complement.getCos()
//                            * (Constants.Arm.STAGE_1_LENGTH + Constants.Arm.STAGE_2_LENGTH);
//            double expectedY =
//                    stage1Complement.getSin()
//                            * (Constants.Arm.STAGE_1_LENGTH + Constants.Arm.STAGE_2_LENGTH);
//
//            assertEquals(expectedX, pose.getX(), epsilon);
//            assertEquals(expectedY, pose.getY(), epsilon);
//            assertEquals(new Rotation2d(), pose.getRotation());
//
//            var angles = ArmSubsystem.inverseKinematics(pose);
//            System.out.println(angles);
//
//            assertEquals(stage1Angle, angles.getA());
//            assertEquals(stage2Angle, angles.getB());
//            assertEquals(stage3Angle, angles.getC());
//         }
//    }
//
//    // Randomly align stage2 with stage1 (inner). Stage3 points parallel to the ground
//    //
//    //     *
//    //    /
//    //   / ---- *
//    //  /
//    // *
//    //
//    @Test
//    void testArmKinematicsFolded() {
//        System.out.println("testArmKinematicsFolded");
//
//         for (int i = 0; i < 100; i++) {
//            System.out.println();
//
//            // Random angle between -10 and LIMIT_DEGREE_STAGE_1
//            Rotation2d stage1Angle = Rotation2d.fromDegrees(Math.random() * 360);
//            Rotation2d stage2Angle = Rotation2d.fromDegrees(0);
//            Rotation2d stage3Angle = Rotation2d.fromDegrees(90).plus(stage1Angle).unaryMinus();
//
//            System.out.println("stage1Angle: " + stage1Angle.getDegrees());
//            System.out.println("stage2Angle: " + stage2Angle.getDegrees());
//            System.out.println("stage3Angle: " + stage3Angle.getDegrees());
//
//            Pose2d pose = ArmSubsystem.forwardKinematics(stage1Angle, stage2Angle, stage3Angle);
//            System.out.println(
//                    "pose: "
//                            + pose.getTranslation().getX()
//                            + ", "
//                            + pose.getTranslation().getY()
//                            + ", "
//                            + pose.getRotation().getDegrees());
//
//            // Get the complement of stage1Angle
//            Rotation2d stage1Complement = Rotation2d.fromDegrees(90).minus(stage1Angle);
//
//            // Since the arms are aligned we expect the bot to trace out a circle of radius
//            // STAGE_1_LENGTH + STAGE_2_LENGTH
//            double expectedX =
//                    stage1Complement.getCos()
//                            * (Constants.Arm.STAGE_1_LENGTH - Constants.Arm.STAGE_2_LENGTH);
//            double expectedY =
//                    stage1Complement.getSin()
//                            * (Constants.Arm.STAGE_1_LENGTH - Constants.Arm.STAGE_2_LENGTH);
//
//            assertEquals(expectedX, pose.getX(), epsilon);
//            assertEquals(expectedY, pose.getY(), epsilon);
//            assertEquals(new Rotation2d(), pose.getRotation());
//
//            var angles = ArmSubsystem.inverseKinematics(pose);
//            System.out.println(angles);
//
//            assertEquals(stage1Angle, angles.getA());
//            assertEquals(stage2Angle, angles.getB());
//            assertEquals(stage3Angle, angles.getC());
//         }
//    }

    // Fuzz arm kinematics
    @Test
    void fuzzArmKinematics() {
        System.out.println("fuzzArmKinematics");

         for (int i = 0; i < 1000; i++) {
            System.out.println();

            Rotation2d stage1Angle = Rotation2d.fromDegrees(Math.random() * 360);
            Rotation2d stage2Angle = Rotation2d.fromDegrees(Math.random() * 180);
            Rotation2d stage3Angle = Rotation2d.fromDegrees(Math.random() * 360);

            System.out.println("stage1Angle: " + stage1Angle.getDegrees());
            System.out.println("stage2Angle: " + stage2Angle.getDegrees());
            System.out.println("stage3Angle: " + stage3Angle.getDegrees());

            Pose2d pose = ArmSubsystem.forwardKinematics(stage1Angle, stage2Angle, stage3Angle);
            System.out.println(
                    "pose: "
                            + pose.getTranslation().getX()
                            + ", "
                            + pose.getTranslation().getY()
                            + ", "
                            + pose.getRotation().getDegrees());

            var angles = ArmSubsystem.inverseKinematics(pose);
            System.out.println(angles);

            // Flip assert order
            assertEquals(stage1Angle, angles.getA());
            assertEquals(stage2Angle, angles.getB());
            assertEquals(stage3Angle, angles.getC());
         }
    }
}
