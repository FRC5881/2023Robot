// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.tvhsfrc.frc2023.robot;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
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

    public static final class Swerve {
        /** Current limit to protect swerve module turn motors. */
        public static final int TURN_CURRENT_LIMIT = 20;

        /** Current limit to protect swerve module drive motors. */
        public static final int DRIVE_CURRENT_LIMIT = 40;

        /** The physical wheel diameter in meters. */
        public static final double WHEEL_DIAMETER = 0.10033;
        // Units.inchesToMeters(3.95);

        /**
         * The reduction factor from drive encoder revolutions revolutions to wheel revolutions.
         *
         * <p>This is 1 / (gear ratio)
         */
        public static final double DRIVE_REDUCTION_FACTOR = 1 / 8.14;

        /**
         * The reduction factor from turn encoder motor revolutions to turning wheel revolutions.
         *
         * <p>This is 1 / (gear ratio)
         */
        public static final double TURN_REDUCTION_FACTOR = (15.0 / 32.0) * (10.0 / 60.0);

        /** Conversion factor from drive encoder revolutions to lienar distance. */
        public static final double DRIVE_CONVERSION_FACTOR =
                (WHEEL_DIAMETER * Math.PI) * DRIVE_REDUCTION_FACTOR / 60;

        /**
         * The left-to-right distance between the drivetrain wheels
         *
         * <p>Should be measured from center to center.
         */
        // public static final double TRACKWIDTH = Units.inchesToMeters(22.5);
        public static final double TRACKWIDTH = 0.4445;

        /**
         * The front-to-back distance between the drivetrain wheels.
         *
         * <p>Should be measured from center to center.
         */
        // public static final double WHEELBASE = Units.inchesToMeters(22.5);
        public static final double WHEELBASE = 0.7239;

        /** The maximum velocity of the drivetrain in meters per second. */
        public static final double MAX_VELOCITY_METERS_PER_SECOND =
                5880.0 / 60.0 * DRIVE_REDUCTION_FACTOR * WHEEL_DIAMETER * Math.PI;

        /**
         * The maximum angular velocity of the robot in radians per second.
         *
         * <p>This is a measure of how fast the robot can rotate in place.
         */
        // Here we calculate the theoretical maximum angular velocity. You can also
        // replace this with a measured amount.
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =
                MAX_VELOCITY_METERS_PER_SECOND / Math.hypot(TRACKWIDTH / 2.0, WHEELBASE / 2.0);

        /**
         * The kinematics object for the swerve drive. This is used to convert a desired "chassis
         * speed" into individual module speeds.
         */
        public static final SwerveDriveKinematics KINEMATICS =
                new SwerveDriveKinematics(
                        new Translation2d(TRACKWIDTH / 2, WHEELBASE / 2),
                        new Translation2d(TRACKWIDTH / 2, -WHEELBASE / 2),
                        new Translation2d(-TRACKWIDTH / 2, WHEELBASE / 2),
                        new Translation2d(-TRACKWIDTH / 2, -WHEELBASE / 2));
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

        public static final String CAMERA_NAME = "camera";
    }

    /** SwerveModuleConstants contains constants unique to each swerve module. */
    public static class SwerveModuleConstants {
        /** The CAN ID of the turn neo. */
        public final int turnMotorCANID;

        /** The CAN ID of the drive neo. */
        public final int driveMotorCANID;

        /** The CAN ID of the ctr magenitic encoder. */
        public final int turnEncoderCANID;

        /** The offset of the encoder from the zero position. In radians. */
        public final double turnEncoderOffset;

        /** Subsystem name. For example: "Front Left Swerve" */
        public final String name;

        public SwerveModuleConstants(
                int turnMotorCANID,
                int driveMotorCANID,
                int turnEncoderCANID,
                double turnEncoderOffset,
                String name) {
            this.turnMotorCANID = turnMotorCANID;
            this.driveMotorCANID = driveMotorCANID;
            this.turnEncoderCANID = turnEncoderCANID;
            this.turnEncoderOffset = turnEncoderOffset;
            this.name = name;
        }
    }

    public static final SwerveModuleConstants FRONT_LEFT_SWERVE_MODULE =
            new SwerveModuleConstants(
                    CANConstants.FRONT_LEFT_MODULE_STEER_MOTOR,
                    CANConstants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
                    CANConstants.FRONT_LEFT_MODULE_STEER_ENCODER,
                    138.86,
                    "Front Left Swerve");

    public static final SwerveModuleConstants FRONT_RIGHT_SWERVE_MODULE =
            new SwerveModuleConstants(
                    CANConstants.FRONT_RIGHT_MODULE_STEER_MOTOR,
                    CANConstants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                    CANConstants.FRONT_RIGHT_MODULE_STEER_ENCODER,
                    136.494,
                    "Front Right Swerve");

    public static final SwerveModuleConstants BACK_LEFT_SWERVE_MODULE =
            new SwerveModuleConstants(
                    CANConstants.BACK_LEFT_MODULE_STEER_MOTOR,
                    CANConstants.BACK_LEFT_MODULE_DRIVE_MOTOR,
                    CANConstants.BACK_LEFT_MODULE_STEER_ENCODER,
                    107.05,
                    "Back Left Swerve");

    public static final SwerveModuleConstants BACK_RIGHT_SWERVE_MODULE =
            new SwerveModuleConstants(
                    CANConstants.BACK_RIGHT_MODULE_STEER_MOTOR,
                    CANConstants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
                    CANConstants.BACK_RIGHT_MODULE_STEER_ENCODER,
                    42.89,
                    "Back Right Swerve");

    public static class VacuumConstants {
        /** This is the target velocity in RPM. */
        public static final double VacuumVelocity = 6000;
        /**
         * Original motor was rated for 5500 RPM. Our motor is rated for ~11000 RPM. 0.6 current
         * limiting keeps it from exceeding the original rating by an excessive amount.
         */
        public static final double maxOutput = 0.6;
    }

    public static class Autonomous {
        /** Maximum speed m/s */
        public static final double MAX_SPEED = 0.25 * Swerve.MAX_VELOCITY_METERS_PER_SECOND;

        /** Maximum acceleration m/s^s */
        public static final double MAX_ACCELERATION = 2;
    }
}
