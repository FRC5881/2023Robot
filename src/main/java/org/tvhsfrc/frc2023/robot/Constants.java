// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.tvhsfrc.frc2023.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
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
    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
    }

    /** Identifiers for all Digital IO devices on the robot. */
    public static class DIOConstants {
        public static final int STAGE_1_LIMIT_SWITCH = 0;
    }

    /** Identifiers for all the CAN devices on the robot. */
    public static class CANConstants {
        public static final int FRONT_RIGHT_STEER_ENCODER = 1;
        public static final int FRONT_LEFT_STEER_ENCODER = 2;
        public static final int BACK_LEFT_STEER_ENCODER = 3;
        public static final int BACK_RIGHT_STEER_ENCODER = 4;

        public static final int FRONT_RIGHT_DRIVE_MOTOR = 10;
        public static final int FRONT_RIGHT_STEER_MOTOR = 11;
        public static final int FRONT_LEFT_DRIVE_MOTOR = 12;
        public static final int FRONT_LEFT_STEER_MOTOR = 13;
        public static final int BACK_LEFT_DRIVE_MOTOR = 14;
        public static final int BACK_LEFT_STEER_MOTOR = 15;
        public static final int BACK_RIGHT_DRIVE_MOTOR = 16;
        public static final int BACK_RIGHT_STEER_MOTOR = 17;

        public static final int ARM_STAGE_1 = 25;
        public static final int ARM_STAGE_2 = 26;

        public static final int INTAKE = 30;
    }

    public static final class Arm {
        /** Converts Motor rotations to Axle rotations */
        public static final double GEARBOX_RATIO = 3 * 5 * 5;

        /** Converts Motor rotations to Axle radians */
        public static final double CONVERSION_FACTOR = GEARBOX_RATIO * 2.0 * Math.PI;

        /** Maximum output (as voltage) */
        public static final double MIN_OUTPUT = -0.25 * 12;

        /** Minimum output (as voltage) */
        public static final double MAX_OUTPUT = 0.25 * 12;

        /** Stage 2 motor starting position, and soft-reverse limit */
        public static final Rotation2d HOME = Rotation2d.fromDegrees(-180);

        /** Stage 2 motor soft-forward limit */
        public static final Rotation2d LIMIT = Rotation2d.fromDegrees(45);

        /**
         * Stage 2 will continuously attempt to get closer to the setpoint, but when within the
         * TOLERANCE it will report that we are at the setpoint
         */
        public static final Rotation2d TOLERANCE = Rotation2d.fromDegrees(5);

        /** Stage 2 is too fast for it's own good, so we limit the speed and acceleration */
        public static final Constraints CONSTRAINTS = new Constraints(100 / 360d, 60 / 360d);
    }

    /**
     * Pre-programmed waypoints for the arm to move to. For use in either autonomous or teleop
     * control.
     *
     * <p>Stage 1 has 2 states - Home and Away, Home being Optional.empty() and Away being a
     * Rotation2d
     */
    public enum WAYPOINT {
        HOME(0),
        LOW_CUBE(0),
        MID_CUBE(0),
        HIGH_CUBE(0),
        DOUBLE_SUBSTATION_CUBE(0);

        private final Rotation2d angle;

        private WAYPOINT(double radians) {
            this.angle = Rotation2d.fromRadians(radians);
        }

        public Rotation2d getAngle() {
            return angle;
        }
    }

    public static final class Intake {
        public static final double speedIN = 1.0;
        public static final double speedOUT = -1.0;
    }

    public static class Autonomous {
        public static final PIDFConfig xAutoPID = new PIDFConfig(0.7, 0, 0);
        public static final PIDFConfig yAutoPID = new PIDFConfig(0.7, 0, 0);
        public static final PIDFConfig angleAutoPID = new PIDFConfig(0.4, 0, 0.01);

        public static final double MAX_SPEED = 4;
        public static final double MAX_ACCELERATION = 2;
    }
}
