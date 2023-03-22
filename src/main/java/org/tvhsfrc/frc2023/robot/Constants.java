// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.tvhsfrc.frc2023.robot;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
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
        public static final int VACUUM_ONE = 18;
        public static final int VACUUM_TWO = 19;
        public static final int VACUUM_THREE = 20;
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
        public static final double VELOCITY = 6000;

        /**
         * Original motor was rated for 5500 RPM. Our motor is rated for ~11000 RPM. 0.6 current
         * limiting keeps it from exceeding the original rating by an excessive amount.
         */
        public static final double MAX_OUTPUT = 0.6;

        /**
         * The amount of time to keep the vacuum purge solenoid on when disable the vacuum. In
         * seconds.
         */
        public static final double PURGE_TIME = 5;
    }

    public static class Autonomous {
        public static final PIDFConfig xAutoPID = new PIDFConfig(0.7, 0, 0);
        public static final PIDFConfig yAutoPID = new PIDFConfig(0.7, 0, 0);
        public static final PIDFConfig angleAutoPID = new PIDFConfig(0.4, 0, 0.01);

        public static final double MAX_SPEED = 4;
        public static final double MAX_ACCELERATION = 2;
    }
}
