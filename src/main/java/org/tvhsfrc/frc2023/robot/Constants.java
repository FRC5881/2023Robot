// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.tvhsfrc.frc2023.robot;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
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
        // ------ STAGE 1 ------ //

        /** Length of the first stage of the arm in meters */
        public static final double STAGE_1_LENGTH = Units.inchesToMeters(38.136);

        /** Converts Motor Rotations to Axle Rotations */
        public static final double GEARBOX_RATIO_STAGE_1 = 3 * 5 * 5 * 5;

        /** Stage 1 PID Settings */
        public static final PIDFConfig STAGE_1_PID = new PIDFConfig(4, 0, 0, 0);

        /** Stage 1 Maximum output (as percentage) for PID control */
        public static final double STAGE_1_MIN_OUTPUT = -0.1;

        /** Stage 1 Minimum output (as negative percentage) for PID control */
        public static final double STAGE_1_MAX_OUTPUT = 0.1;

        /** Stage 1 Away angle (rotations) */
        public static final Rotation2d STAGE_1_AWAY = Rotation2d.fromRotations(0.0161);

        /** Stage 1 Soft limit */
        public static final Rotation2d STAGE_1_LIMIT = Rotation2d.fromDegrees(60);

        /** Stage 1 Starting Position */
        public static final Rotation2d STAGE_1_HOME = Rotation2d.fromDegrees(-10);

        /**
         * Stage 1 will continuously attempt to get closer to the setpoint, but when within the
         * TOLERANCE it will report that it is at the setpoint
         */
        public static final Rotation2d STAGE_1_TOLERANCE = Rotation2d.fromDegrees(5);

        // ------ STAGE 2 ------ //

        /** Length of the second stage of the arm in meters */
        public static final double STAGE_2_LENGTH = Units.inchesToMeters(35);

        /** Converts Motor Rotations to Axle Rotations */
        public static final double GEARBOX_RATIO_STAGE_2 = 3 * 5 * 5;

        /** Stage 2 PID Settings - Use values from the SPARK Max Hardware Client */
        public static final PIDFConfig STAGE_2_PID = new PIDFConfig(11.25, 0, 3.75, 0);

        /** Stage 2 Maximum output (as percentage) for PID control */
        public static final double STAGE_2_MIN_OUTPUT = -0.25;

        /** Stage 2 Minimum output (as negative percentage) for PID control */
        public static final double STAGE_2_MAX_OUTPUT = 0.25;

        /** Stage 2 motor starting position (rotations) */
        public static final Rotation2d STAGE_2_HOME = Rotation2d.fromDegrees(0);

        /** Stage 2 motor soft-forward limit */
        public static final Rotation2d STAGE_2_LIMIT = Rotation2d.fromDegrees(135);

        /**
         * Stage 2 will continuously attempt to get closer to the setpoint, but when within the
         * TOLERANCE it will report that it is at the setpoint
         */
        public static final Rotation2d STAGE_2_TOLERANCE = Rotation2d.fromDegrees(5);

        /** Stage 2 is too fast for it's own good, so we limit the speed and acceleration */
        public static final Constraints STAGE_2_CONSTRAINTS =
                new Constraints(120 / 360d, 60 / 360d);
    }

    /**
     * Pre-programmed waypoints for the arm to move to. For use in either autonomous or teleop
     * control.
     *
     * <p>Stage 1 has 2 states - Home and Away, Home being Optional.empty() and Away being a
     * Rotation2d
     */
    public enum WAYPOINT {
        HOME(true, 0),
        LOW_CUBE(true, 0.0784),
        MID_CUBE(true, 0.1742),
        HIGH_CUBE(false, 0.3314),
        DOUBLE_SUBSTATION_CUBE(true, 0.1869);

        private final Pair<Boolean, Rotation2d> angles;

        private WAYPOINT(boolean stage1, double stage2) {
            this.angles = new Pair<>(stage1, Rotation2d.fromRotations(stage2));
        }

        public boolean isStage1Home() {
            return angles.getFirst();
        }

        public Rotation2d getStage2Angle() {
            return angles.getSecond();
        }

        public TrapezoidProfile.State getStage2State() {
            return new TrapezoidProfile.State(angles.getSecond().getRotations(), 0);
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
