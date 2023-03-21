// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.tvhsfrc.frc2023.robot;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class MassConstants {
        public static final double ROBOT_MASS = Units.lbsToKilograms(148); // TODO
        public static final Matter CHASSIS =
                new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
        public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
    }

    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
    }

    /** Identifiers for all of the CAN devices on the robot. */
    public static class CANConstants {
        /* Removed as defined in the swerve lib json

        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 1;
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 2;

        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 3;
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 4;

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 10;
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 11;

        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 12;
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 13;

        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 14;
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 15;

        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 16;
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 17;
        */

        public static final int VACUUM_ONE = 20;
        public static final int VACUUM_TWO = 21;
        public static final int VACUUM_THREE = 22;

        public static final int ARM_STAGE_ONE = 25;
        public static final int ARM_STAGE_TWO = 26;
        public static final int ARM_STAGE_THREE = 27;
    }

    public static final class Arm {
        /** Length of the first stage of the arm in meters */
        public static final double STAGE_1_LENGTH = Units.inchesToMeters(38.136);

        public static final double GEARBOX_RATIO_STAGE_1 = 5 * 5 * 5;
        public static final double STAGE_1_LIMIT = 60 / 360d;

        /** Stage 1 PID Settings */
        public static final PIDFConfig STAGE_1_PID = new PIDFConfig(0.1, 0.0001, 0, 0);
        /** Stage 1 Maximum output (as percentage) for PID control */
        public static final double STAGE_1_MIN_OUTPUT = -0.2;
        /** Stage 1 Minimum output (as negative percentage) for PID control */
        public static final double STAGE_1_MAX_OUTPUT = 0.2;

        /** Length of the second stage of the arm in meters */
        public static final double STAGE_2_LENGTH = Units.inchesToMeters(35);

        public static final double GEARBOX_RATIO_STAGE_2 = 3 * 5 * 5;
        public static final double STAGE_2_LIMIT = 180 / 360d;

        /** Stage 2 PID Settings */
        public static final PIDFConfig STAGE_2_PID = new PIDFConfig(0.15, 0, 0.05, 0);
        /** Stage 2 Maximum output (as percentage) for PID control */
        public static final double STAGE_2_MIN_OUTPUT = -0.15;
        /** Stage 2 Minimum output (as negative percentage) for PID control */
        public static final double STAGE_2_MAX_OUTPUT = 0.15;

        /** Length of the third stage of the arm/the grabber in meters */
        public static final double STAGE_3_LENGTH = Units.inchesToMeters(7);

        public static final double GEARBOX_RATIO_STAGE_3 =
                4 * 4 * (28 / 16d) * (28 / 16d) * (28 / 16d);
        public static final double STAGE_3_LIMIT = 270 / 360d;

        /** Stage 3 PID Settings */
        public static final PIDFConfig STAGE_3_PID = new PIDFConfig(0.15, 0, 0.3, 0);
        /** Stage 3 Maximum output (as percentage) for PID control */
        public static final double STAGE_3_MIN_OUTPUT = -0.2;
        /** Stage 3 Minimum output (as negative percentage) for PID control */
        public static final double STAGE_3_MAX_OUTPUT = 0.2;

        // TODO: choose tolerances
        public static final double ANGLE_TOLERANCE = 2;
        public static final double DISTANCE_TOLERANCE = Units.inchesToMeters(1);
    }

    public enum WayPoints {
        HOME(new Pose2d(-0.0048, 0.08, Rotation2d.fromDegrees(45))),
        SAFE(new Pose2d(0.26, 0.17, Rotation2d.fromDegrees(45))),
        CUBE_BOTTOM(new Pose2d(0.50, 0.2820, Rotation2d.fromDegrees(45))),
        CUBE_MIDDLE(new Pose2d(0.87, 0.91, Rotation2d.fromDegrees(45))),
        CUBE_TOP(new Pose2d(1.14, 1.15, Rotation2d.fromDegrees(45))),
        CONE_BOTTOM(new Pose2d(0.483, 0.113, Rotation2d.fromDegrees(45))),
        CONE_MIDDLE(new Pose2d(0.870, 0.995, Rotation2d.fromDegrees(-45))),
        CONE_TOP(new Pose2d(1.098, 1.397, Rotation2d.fromDegrees(45))),
        CUBE_STORE(new Pose2d(0.366, 0.170, Rotation2d.fromDegrees(45))),
        CONE_STORE(new Pose2d(0.366, 0.170, Rotation2d.fromDegrees(45)));

        public final Pose2d pose;

        WayPoints(Pose2d pose) {
            this.pose = pose;
        }

        public boolean insideBot() {
            switch (this) {
                case HOME:
                case CONE_STORE:
                case CUBE_STORE:
                case SAFE:
                    return true;
                case CUBE_BOTTOM:
                case CONE_TOP:
                case CONE_MIDDLE:
                case CONE_BOTTOM:
                case CUBE_TOP:
                case CUBE_MIDDLE:
                    return false;
            }
            return false;
        }
    }

    /** Constants related to PhotoVision, the camera, and the raspberry pi. */
    public static final class Vision {
        /**
         * Position and angle of the camera relative to the center of the bot
         *
         * <p>TODO: define bot axes
         */
        public static final Transform3d CAMERA_TRANSFORM =
                new Transform3d(new Translation3d(), new Rotation3d());

        /** The april tag layout to use */
        public static final String FIELD_LAYOUT = AprilTagFields.k2023ChargedUp.m_resourceFile;

        public static final String CAMERA_NAME = "photonvision";
    }

    public static class Vacuum {
        /** This is the target velocity in RPM. */
        public static final double VACUUM_VELOCITY = 6000;

        /**
         * Original motor was rated for 5500 RPM. Our motor is rated for ~11000 RPM. 0.6 current
         * limiting keeps it from exceeding the original rating by an excessive amount.
         */
        public static final double MAX_OUTPUT = 0.6;

        /**
         * The time in seconds that the vacuum dump valve should be open for when purging the
         * vacuum.
         */
        public static final double DUMP_TIME = 5;
    }

    public static class Autonomous {
        public static final PIDFConfig xAutoPID = new PIDFConfig(0.7, 0, 0);
        public static final PIDFConfig yAutoPID = new PIDFConfig(0.7, 0, 0);
        public static final PIDFConfig angleAutoPID = new PIDFConfig(0.4, 0, 0.01);

        public static final double MAX_SPEED = 4;
        public static final double MAX_ACCELERATION = 2;
    }
}
