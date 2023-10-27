// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.tvhsfrc.frc2023.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import org.tvhsfrc.frc2023.robot.utils.Triple;
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
        public static final double ROBOT_MASS = Units.lbsToKilograms(148);
        public static final Matter CHASSIS =
                new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
        public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms spark max velocity lag
    }

    public static class OperatorConstants {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int ARM_CONTROLLER_PORT = 1;
    }

    /** Identifiers for all Digital IO devices on the robot. */
    public static class DIOConstants {
        public static final int STAGE_1_LIMIT_SWITCH = 0;
    }

    /** Identifiers for all the CAN devices on the robot. */
    public static class CANConstants {
        /*
         * Removed as defined in the swerve lib json
         *
         * public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 1;
         * public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 2;
         *
         * public static final int BACK_LEFT_MODULE_STEER_ENCODER = 3;
         * public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 4;
         *
         * public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 10;
         * public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 11;
         *
         * public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 12;
         * public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 13;
         *
         * public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 14;
         * public static final int BACK_LEFT_MODULE_STEER_MOTOR = 15;
         *
         * public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 16;
         * public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 17;
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

        public static final double GEARBOX_RATIO_STAGE_1 = 3 * 5 * 5 * 5;

        /** Stage 1 PID Settings */
        public static final PIDFConfig STAGE_1_PID = new PIDFConfig(4.0, 0.1, 0.4, 0);

        /** Stage 1 Maximum output (as percentage) for PID control */
        public static final double STAGE_1_MIN_OUTPUT = -1;
        /** Stage 1 Minimum output (as negative percentage) for PID control */
        public static final double STAGE_1_MAX_OUTPUT = 1;

        /** Length of the second stage of the arm in meters */
        public static final double STAGE_2_LENGTH = Units.inchesToMeters(35);

        public static final double GEARBOX_RATIO_STAGE_2 = 3 * 5 * 5;
        /** Stage 2 PID Settings - Use values from the SPARK Max Hardware Client */
        public static final PIDFConfig STAGE_2_PID = new PIDFConfig(0.15, 0, 0.05, 0);
        /** Stage 2 Maximum output (as percentage) for PID control */
        public static final double STAGE_2_MIN_OUTPUT = -0.25;
        /** Stage 2 Minimum output (as negative percentage) for PID control */
        public static final double STAGE_2_MAX_OUTPUT = 0.25;

        /** Length of the third stage of the arm/the grabber in meters */
        public static final double STAGE_3_LENGTH = Units.inchesToMeters(7);

        public static final double GEARBOX_RATIO_STAGE_3 =
                4 * 4 * (28 / 16d) * (28 / 16d) * (28 / 16d);

        /** Stage 1 motor starting position (rotations) */
        public static final double STAGE_1_HOME = -10 / 360d;
        /** Stage 1 motor soft-forward limit */
        public static final double STAGE_1_LIMIT = 60 / 360d;
        /**
         * Stage 1 will continuously attempt to get closer to the setpoint, but when within the
         * TOLERANCE it will report that it is at the setpoint
         */
        public static final double STAGE_1_TOLERANCE = 5 / 360d;

        /** Stage 2 motor starting position (rotations) */
        public static final double STAGE_2_HOME = 0 / 360d;
        /** Stage 2 motor soft-forward limit */
        public static final double STAGE_2_LIMIT = 180 / 360d;
        /**
         * Stage 2 will continuously attempt to get closer to the setpoint, but when within the
         * TOLERANCE it will report that it is at the setpoint
         */
        public static final double STAGE_2_TOLERANCE = 5 / 360d;

        /** Stage 3 motor starting position (rotations) */
        public static final double STAGE_3_HOME = 0 / 360d;
        /** Stage 3 motor soft-forward limit */
        public static final double STAGE_3_LIMIT = 270 / 360d;
        /**
         * Stage 3 will continuously attempt to get closer to the setpoint, but when within the
         * TOLERANCE it will report that it is at the setpoint
         */
        public static final double STAGE_3_TOLERANCE = 5 / 360d;

        /** Stage 3 PID Settings */
        public static final PIDFConfig STAGE_3_PID = new PIDFConfig(0.15, 0, 0.3, 0);
        /** Stage 3 Maximum output (as percentage) for PID control */
        public static final double STAGE_3_MIN_OUTPUT = -0.25;
        /** Stage 3 Minimum output (as negative percentage) for PID control */
        public static final double STAGE_3_MAX_OUTPUT = 0.25;

        /**
         * ADJACENCY_LIST is a HashMap that maps a waypoint to a list of waypoints that can be
         * reached
         *
         * <p>Every waypoint should have an entry in the map.
         */
        public static final HashMap<WAYPOINT, ArrayList<WAYPOINT>> ADJACENCY_LIST =
                new HashMap<>() {
                    {
                        put(
                                WAYPOINT.HOME,
                                new ArrayList<>(
                                        Arrays.asList(
                                                // home can only reach mid points
                                                WAYPOINT.SAFE,
                                                WAYPOINT.MID_CONE_MIDPOINT,
                                                WAYPOINT.MID_CUBE_MIDPOINT,
                                                WAYPOINT.HIGH_CONE_MIDPOINT,
                                                WAYPOINT.HIGH_CUBE_MIDPOINT,
                                                WAYPOINT.DOUBLE_SUBSTATION_CONE,
                                                WAYPOINT.DOUBLE_SUBSTATION_CUBE)));
                        put(
                                WAYPOINT.SAFE,
                                new ArrayList<>(
                                        Arrays.asList(
                                                // other "mid points"
                                                WAYPOINT.HOME,
                                                WAYPOINT.MID_CONE_MIDPOINT,
                                                WAYPOINT.MID_CUBE_MIDPOINT,
                                                WAYPOINT.HIGH_CONE_MIDPOINT,
                                                WAYPOINT.HIGH_CUBE_MIDPOINT,
                                                WAYPOINT.DOUBLE_SUBSTATION_CONE,
                                                WAYPOINT.DOUBLE_SUBSTATION_CUBE,
                                                // end points
                                                WAYPOINT.LOW_CUBE,
                                                WAYPOINT.LOW_CONE,
                                                WAYPOINT.FLOOR_CONE,
                                                WAYPOINT.FLOOR_CUBE)));
                        put(
                                WAYPOINT.MID_CONE_MIDPOINT,
                                new ArrayList<>(
                                        Arrays.asList(
                                                // other "mid points"
                                                WAYPOINT.HOME,
                                                WAYPOINT.SAFE,
                                                WAYPOINT.MID_CUBE_MIDPOINT,
                                                WAYPOINT.HIGH_CONE_MIDPOINT,
                                                WAYPOINT.HIGH_CUBE_MIDPOINT,
                                                WAYPOINT.DOUBLE_SUBSTATION_CONE,
                                                WAYPOINT.DOUBLE_SUBSTATION_CUBE,
                                                // end points
                                                WAYPOINT.MID_CONE)));
                        put(
                                WAYPOINT.MID_CUBE_MIDPOINT,
                                new ArrayList<>(
                                        Arrays.asList(
                                                // other "mid points"
                                                WAYPOINT.HOME,
                                                WAYPOINT.SAFE,
                                                WAYPOINT.MID_CONE_MIDPOINT,
                                                WAYPOINT.HIGH_CONE_MIDPOINT,
                                                WAYPOINT.HIGH_CUBE_MIDPOINT,
                                                WAYPOINT.DOUBLE_SUBSTATION_CONE,
                                                WAYPOINT.DOUBLE_SUBSTATION_CUBE,
                                                // end points
                                                WAYPOINT.MID_CUBE)));
                        put(
                                WAYPOINT.HIGH_CONE_MIDPOINT,
                                new ArrayList<>(
                                        Arrays.asList(
                                                // other "mid points"
                                                WAYPOINT.HOME,
                                                WAYPOINT.SAFE,
                                                WAYPOINT.MID_CUBE_MIDPOINT,
                                                WAYPOINT.MID_CONE_MIDPOINT,
                                                WAYPOINT.HIGH_CUBE_MIDPOINT,
                                                WAYPOINT.DOUBLE_SUBSTATION_CONE,
                                                WAYPOINT.DOUBLE_SUBSTATION_CUBE,
                                                // end points
                                                WAYPOINT.HIGH_CONE)));
                        put(
                                WAYPOINT.HIGH_CUBE_MIDPOINT,
                                new ArrayList<>(
                                        Arrays.asList(
                                                // other "mid points"
                                                WAYPOINT.HOME,
                                                WAYPOINT.SAFE,
                                                WAYPOINT.MID_CUBE_MIDPOINT,
                                                WAYPOINT.MID_CONE_MIDPOINT,
                                                WAYPOINT.HIGH_CONE_MIDPOINT,
                                                WAYPOINT.DOUBLE_SUBSTATION_CONE,
                                                WAYPOINT.DOUBLE_SUBSTATION_CUBE,
                                                // end points
                                                WAYPOINT.HIGH_CUBE)));
                        put(
                                WAYPOINT.DOUBLE_SUBSTATION_CONE,
                                new ArrayList<>(
                                        Arrays.asList(
                                                // other "mid points"
                                                WAYPOINT.HOME,
                                                WAYPOINT.SAFE,
                                                WAYPOINT.MID_CUBE_MIDPOINT,
                                                WAYPOINT.MID_CONE_MIDPOINT,
                                                WAYPOINT.HIGH_CUBE_MIDPOINT,
                                                WAYPOINT.HIGH_CONE_MIDPOINT,
                                                WAYPOINT.DOUBLE_SUBSTATION_CUBE)));
                        // end points
                        put(
                                WAYPOINT.DOUBLE_SUBSTATION_CUBE,
                                new ArrayList<>(
                                        Arrays.asList(
                                                // other "mid points"
                                                WAYPOINT.HOME,
                                                WAYPOINT.SAFE,
                                                WAYPOINT.MID_CUBE_MIDPOINT,
                                                WAYPOINT.MID_CONE_MIDPOINT,
                                                WAYPOINT.HIGH_CUBE_MIDPOINT,
                                                WAYPOINT.HIGH_CONE_MIDPOINT,
                                                WAYPOINT.DOUBLE_SUBSTATION_CONE)));
                        // end points
                        put(
                                WAYPOINT.LOW_CUBE,
                                new ArrayList<>(
                                        Arrays.asList(
                                                // mid points
                                                WAYPOINT.SAFE,
                                                // end points
                                                WAYPOINT.FLOOR_CONE,
                                                WAYPOINT.FLOOR_CUBE,
                                                WAYPOINT.LOW_CONE)));
                        put(
                                WAYPOINT.LOW_CONE,
                                new ArrayList<>(
                                        Arrays.asList(
                                                // mid points
                                                WAYPOINT.SAFE,
                                                // end points
                                                WAYPOINT.FLOOR_CONE,
                                                WAYPOINT.FLOOR_CUBE,
                                                WAYPOINT.LOW_CUBE)));
                        put(
                                WAYPOINT.FLOOR_CONE,
                                new ArrayList<>(
                                        Arrays.asList(
                                                // mid points
                                                WAYPOINT.SAFE,
                                                // end points
                                                WAYPOINT.LOW_CONE,
                                                WAYPOINT.LOW_CUBE,
                                                WAYPOINT.FLOOR_CUBE)));
                        put(
                                WAYPOINT.FLOOR_CUBE,
                                new ArrayList<>(
                                        Arrays.asList(
                                                // mid points
                                                WAYPOINT.SAFE,
                                                // end points
                                                WAYPOINT.LOW_CONE,
                                                WAYPOINT.LOW_CUBE,
                                                WAYPOINT.FLOOR_CONE)));
                        put(
                                WAYPOINT.MID_CONE,
                                new ArrayList<>(
                                        Arrays.asList(
                                                // mid points
                                                WAYPOINT.MID_CONE_MIDPOINT,
                                                WAYPOINT.SAFE,
                                                WAYPOINT.DOUBLE_SUBSTATION_CONE,
                                                WAYPOINT.DOUBLE_SUBSTATION_CUBE,
                                                WAYPOINT.HIGH_CONE,
                                                WAYPOINT.MID_CUBE,
                                                WAYPOINT.HIGH_CUBE
                                                // other positions?
                                                )));
                        put(
                                WAYPOINT.MID_CUBE,
                                new ArrayList<>(
                                        Arrays.asList(
                                                // mid points
                                                WAYPOINT.MID_CUBE_MIDPOINT,
                                                WAYPOINT.SAFE,
                                                WAYPOINT.DOUBLE_SUBSTATION_CUBE,
                                                WAYPOINT.DOUBLE_SUBSTATION_CONE,
                                                WAYPOINT.HIGH_CUBE,
                                                WAYPOINT.MID_CONE,
                                                WAYPOINT.HIGH_CONE
                                                // other positions?
                                                )));
                        put(
                                WAYPOINT.HIGH_CONE,
                                new ArrayList<>(
                                        Arrays.asList(
                                                // mid points
                                                WAYPOINT.HIGH_CONE_MIDPOINT,
                                                WAYPOINT.HIGH_CUBE,
                                                WAYPOINT.MID_CONE,
                                                WAYPOINT.DOUBLE_SUBSTATION_CONE,
                                                WAYPOINT.DOUBLE_SUBSTATION_CUBE
                                                // other positions?
                                                )));
                        put(
                                WAYPOINT.HIGH_CUBE,
                                new ArrayList<>(
                                        Arrays.asList(
                                                // mid points
                                                WAYPOINT.HIGH_CUBE_MIDPOINT
                                                // other positions?
                                                )));
                    }
                };

        public enum GAME_PIECE_TYPE {
            CONE,
            CUBE
        }

        public enum ARM_TARGET {
            HOME,
            SAFE,
            FLOOR,
            LOW,
            MID,
            HIGH,
            DOUBLE_SUBSTATION;
        }
    }

    public enum WAYPOINT {
        HOME(0, 0.0, 0.0),
        SAFE(0, 0.0504, 0.0),

        LOW_CUBE(0, 0.0876, 0.2954),
        MID_CUBE_MIDPOINT(0, 0.185, 0.0),
        MID_CUBE(0, 0.2311, 0.3778), // score
        HIGH_CUBE_MIDPOINT(0, 0.3758, 0.0),
        HIGH_CUBE(0.0805, 0.3847, 0.4037), // score
        FLOOR_CUBE(0.041, 0.0933, 0.359), // Floor cube midpoint stage 2: 0.0933
        DOUBLE_SUBSTATION_CUBE(0, 0.2314, 0.2501),

        LOW_CONE(0.0249, 0.0733, 0.1352),
        MID_CONE_MIDPOINT(0, 0.218, 0.0),
        MID_CONE(0, 0.2457, 0.2193), // score
        HIGH_CONE_MIDPOINT(0, 0.2495, 0.0),
        HIGH_CONE(0.0996, 0.3879, 0.3890), // score
        FLOOR_CONE(0.0673, 0.0822, 0.4556),
        DOUBLE_SUBSTATION_CONE(0, 0.2314, 0.2501);

        public final Triple<Double, Double, Double> position;

        WAYPOINT(Triple<Double, Double, Double> position) {
            this.position = new Triple<>(position.getA(), position.getB(), position.getC());
        }

        WAYPOINT(double stage1, double stage2, double stage3) {
            this(new Triple<>(stage1, stage2, stage3));
        }
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
