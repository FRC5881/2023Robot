// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.tvhsfrc.frc2023.robot;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import org.tvhsfrc.frc2023.robot.subsystems.ArmSubsystem;
import org.tvhsfrc.frc2023.robot.utils.Triple;
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
    /** Global switch for competition mode */
    public static final boolean COMPETITION_MODE = true;

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

    public static final class LEDs {
        public static final int LED_PWM = 0;
        public static final int LED_LENGTH = 60;

        public static final Triple<Integer, Integer, Integer> COLOR_CUBE =
                new Triple<>(159, 23, 169);
        public static final Triple<Integer, Integer, Integer> COLOR_CONE =
                new Triple<>(235, 220, 13);
        public static final Triple<Integer, Integer, Integer> COLOR_DISABLED =
                new Triple<>(255, 0, 0);
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
        public static final double STAGE_2_MIN_OUTPUT = -1;
        /** Stage 2 Minimum output (as negative percentage) for PID control */
        public static final double STAGE_2_MAX_OUTPUT = 1;

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

        /**
         * ADJACENCY_LIST is a HashMap that maps a waypoint to a list of waypoints that can be
         * reached
         *
         * <p>Every waypoint should have an entry in the map.
         */
        public static final HashMap<WAYPOINT, ArrayList<WAYPOINT>> ADJACENCY_LIST =
                new HashMap<>() {
                    {
                        // HOME, LOW, MID, DOUBLE_SUBSTATION are all reachable from any other
                        // HIGH CUBE is ONLY reachable from mid-cube

                        put(
                                WAYPOINT.HOME,
                                new ArrayList<>(
                                        Arrays.asList(
                                                WAYPOINT.LOW_CONE,
                                                WAYPOINT.LOW_CUBE,
                                                WAYPOINT.MID_CONE,
                                                WAYPOINT.MID_CUBE,
                                                WAYPOINT.DOUBLE_SUBSTATION_CONE,
                                                WAYPOINT.DOUBLE_SUBSTATION_CUBE)));

                        put(
                                WAYPOINT.DOUBLE_SUBSTATION_CONE,
                                new ArrayList<>(
                                        Arrays.asList(
                                                WAYPOINT.HOME,
                                                WAYPOINT.LOW_CONE,
                                                WAYPOINT.LOW_CUBE,
                                                WAYPOINT.MID_CONE,
                                                WAYPOINT.MID_CUBE)));

                        put(
                                WAYPOINT.DOUBLE_SUBSTATION_CUBE,
                                new ArrayList<>(
                                        Arrays.asList(
                                                WAYPOINT.HOME,
                                                WAYPOINT.LOW_CONE,
                                                WAYPOINT.LOW_CUBE,
                                                WAYPOINT.MID_CONE,
                                                WAYPOINT.MID_CUBE)));

                        put(
                                WAYPOINT.LOW_CONE,
                                new ArrayList<>(
                                        Arrays.asList(
                                                WAYPOINT.HOME,
                                                WAYPOINT.LOW_CUBE,
                                                WAYPOINT.MID_CONE,
                                                WAYPOINT.MID_CUBE,
                                                WAYPOINT.DOUBLE_SUBSTATION_CONE,
                                                WAYPOINT.DOUBLE_SUBSTATION_CUBE)));

                        put(
                                WAYPOINT.LOW_CUBE,
                                new ArrayList<>(
                                        Arrays.asList(
                                                WAYPOINT.HOME,
                                                WAYPOINT.LOW_CONE,
                                                WAYPOINT.MID_CONE,
                                                WAYPOINT.MID_CUBE,
                                                WAYPOINT.DOUBLE_SUBSTATION_CONE,
                                                WAYPOINT.DOUBLE_SUBSTATION_CUBE)));

                        put(
                                WAYPOINT.MID_CONE,
                                new ArrayList<>(
                                        Arrays.asList(
                                                WAYPOINT.HOME,
                                                WAYPOINT.LOW_CONE,
                                                WAYPOINT.LOW_CUBE,
                                                WAYPOINT.MID_CUBE,
                                                WAYPOINT.DOUBLE_SUBSTATION_CONE,
                                                WAYPOINT.DOUBLE_SUBSTATION_CUBE)));

                        put(
                                WAYPOINT.MID_CUBE,
                                new ArrayList<>(
                                        Arrays.asList(
                                                // Uniquely reachable from mid-cube
                                                WAYPOINT.HIGH_CUBE,
                                                WAYPOINT.HOME,
                                                WAYPOINT.LOW_CONE,
                                                WAYPOINT.LOW_CUBE,
                                                WAYPOINT.MID_CONE,
                                                WAYPOINT.DOUBLE_SUBSTATION_CONE,
                                                WAYPOINT.DOUBLE_SUBSTATION_CUBE)));

                        put(WAYPOINT.HIGH_CUBE, new ArrayList<>(Arrays.asList(WAYPOINT.MID_CUBE)));
                    }
                };

        public enum GAME_PIECE_TYPE {
            CONE,
            CUBE
        }

        public enum ARM_TARGET {
            HOME,
            LOW,
            MID,
            HIGH,
            DOUBLE_SUBSTATION;
        }
    }

    public enum WAYPOINT {
        HOME(Arm.STAGE_1_HOME, Arm.STAGE_2_HOME),

        LOW_CUBE(Arm.STAGE_1_HOME, 0.0876),
        LOW_CONE(0.0249, 0.0733),

        MID_CUBE(Arm.STAGE_1_HOME, 0.2311),
        MID_CONE(Arm.STAGE_1_HOME, 0.2457),

        HIGH_CUBE(0.0805, 0.3847),

        DOUBLE_SUBSTATION_CUBE(Arm.STAGE_1_HOME, 0.2314),
        DOUBLE_SUBSTATION_CONE(Arm.STAGE_1_HOME, 0.2314);

        private final Pair<Rotation2d, Rotation2d> angles;

        private WAYPOINT(double stage1, double stage2) {
            this.angles =
                    new Pair<>(Rotation2d.fromRotations(stage1), Rotation2d.fromRotations(stage2));
        }

        public Pair<Rotation2d, Rotation2d> getAngle() {
            return this.angles;
        }

        public Translation2d getTranslation() {
            return ArmSubsystem.forwardKinematics(angles.getFirst(), angles.getSecond());
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
