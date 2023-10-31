package org.tvhsfrc.frc2023.robot.subsystems;

import static org.tvhsfrc.frc2023.robot.Constants.Arm.*;

import com.revrobotics.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.PriorityQueue;
import org.tvhsfrc.frc2023.robot.Constants;
import org.tvhsfrc.frc2023.robot.Constants.Arm.ARM_TARGET;
import org.tvhsfrc.frc2023.robot.Constants.Arm.GAME_PIECE_TYPE;
import org.tvhsfrc.frc2023.robot.Constants.WAYPOINT;
import org.tvhsfrc.frc2023.robot.commands.arm.ArmNext;
import org.tvhsfrc.frc2023.robot.commands.arm.ArmWaypoint;

public class ArmSubsystem extends SubsystemBase {
    private final DigitalInput stage1LimitSwitch =
            new DigitalInput(Constants.DIOConstants.STAGE_1_LIMIT_SWITCH);

    private final CANSparkMax stage1 =
            new CANSparkMax(
                    Constants.CANConstants.ARM_STAGE_1, CANSparkMaxLowLevel.MotorType.kBrushless);

    private final CANSparkMax stage2 =
            new CANSparkMax(
                    Constants.CANConstants.ARM_STAGE_2, CANSparkMaxLowLevel.MotorType.kBrushless);

    private double stage1Setpoint;
    private double stage2Setpoint;

    private GAME_PIECE_TYPE currentGamePieceTarget = GAME_PIECE_TYPE.CONE;
    private WAYPOINT previousArmWaypoint = WAYPOINT.HOME;
    private ARM_TARGET currentArmTarget = ARM_TARGET.HOME;

    public ArmSubsystem() {
        // Stage 1 motor controller setup
        stage1.restoreFactoryDefaults();

        stage1.getPIDController().setP(STAGE_1_PID.p);
        stage1.getPIDController().setI(STAGE_1_PID.i);
        stage1.getPIDController().setD(STAGE_1_PID.d);
        stage1.getPIDController().setFF(STAGE_1_PID.f);
        stage1.getPIDController().setOutputRange(STAGE_1_MIN_OUTPUT, STAGE_1_MAX_OUTPUT);

        stage1.setInverted(true);
        stage1.setIdleMode(CANSparkMax.IdleMode.kCoast);
        stage1.getEncoder().setPositionConversionFactor(1 / GEARBOX_RATIO_STAGE_1);
        stage1.getEncoder().setVelocityConversionFactor(1 / GEARBOX_RATIO_STAGE_1);
        stage1.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        stage1.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        stage1.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) STAGE_1_LIMIT);
        stage1.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) STAGE_1_HOME);
        stage1.getEncoder().setPosition(STAGE_1_HOME);

        stage1.burnFlash();

        // Stage 2 motor controller setup
        stage2.restoreFactoryDefaults();

        stage2.getPIDController().setP(STAGE_2_PID.p);
        stage2.getPIDController().setI(STAGE_2_PID.i);
        stage2.getPIDController().setD(STAGE_2_PID.d);
        stage2.getPIDController().setFF(STAGE_2_PID.f);
        stage2.getPIDController().setOutputRange(STAGE_2_MIN_OUTPUT, STAGE_2_MAX_OUTPUT);

        stage2.setIdleMode(CANSparkMax.IdleMode.kBrake);
        stage2.getEncoder().setPositionConversionFactor(1 / GEARBOX_RATIO_STAGE_2);
        stage2.getEncoder().setVelocityConversionFactor(1 / GEARBOX_RATIO_STAGE_2);
        stage2.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        stage2.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        stage2.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) STAGE_2_LIMIT);
        stage2.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) STAGE_2_HOME);
        stage2.getEncoder().setPosition(STAGE_2_HOME);

        stage2.burnFlash();

        SmartDashboard.putData("Arm", this);

        // Start at home
        setSetpoint(Rotation2d.fromRotations(STAGE_1_HOME), Rotation2d.fromRotations(STAGE_2_HOME));
    }

    @Override
    public void periodic() {
        if (!stage1LimitSwitch.get()) {
            stage1.getEncoder().setPosition(0);
        }
    }

    /**
     * This just takes the inputs for the Law of Cosines, and spits out the desired angle, gamma.
     *
     * <p>If the edges suggest the triangle is impossible this returns NaN.
     *
     * @param a a leg
     * @param b a leg
     * @param c side opposite the returned angle (gamma)
     * @return Gives the value of gamma in radians
     */
    private static double lawOfCosines(double a, double b, double c) {
        return Math.acos((a * a + b * b - c * c) / (2 * a * b));
    }

    /**
     * Given a Pose2d (a position & a angle) of the end effector return the angles of the joints.
     *
     * <p>If the target is out of reach, stage 1 and 2 will be made into a straight line that points
     * to the target.
     *
     * @param translation The position and angle of the end effector
     * @return The angles of the joints
     */
    public static Pair<Rotation2d, Rotation2d> inverseKinematics(Translation2d translation) {
        double c = translation.getNorm();

        // Outside the outer circle of the donut
        if (c >= STAGE_1_LENGTH + STAGE_2_LENGTH) {
            Rotation2d r1 = Rotation2d.fromDegrees(90).minus(translation.getAngle());
            Rotation2d r2 = Rotation2d.fromDegrees(180);
            return new Pair<>(r1, r2);
        }

        // Inside the inner circle of the donut
        if (c <= Math.abs(STAGE_1_LENGTH - STAGE_2_LENGTH)) {
            Rotation2d r1 = Rotation2d.fromDegrees(90).minus(translation.getAngle());
            Rotation2d r2 = Rotation2d.fromDegrees(0);
            return new Pair<>(r1, r2);
        }

        double r1 =
                lawOfCosines(STAGE_1_LENGTH, c, STAGE_2_LENGTH)
                        + translation.getAngle().getRadians();
        double r2 = lawOfCosines(STAGE_1_LENGTH, STAGE_2_LENGTH, c);

        return new Pair<>(
                Rotation2d.fromDegrees(90).minus(Rotation2d.fromRadians(r1)),
                Rotation2d.fromRadians(r2));
    }

    /**
     * Given the angles of the joints, return the position of the end effector.
     *
     * @param r1 The angle of the first joint
     * @param r2 The angle of the second joint
     * @return The position of the end effector
     */
    public static Translation2d forwardKinematics(Rotation2d r1, Rotation2d r2) {
        Rotation2d angle1 = Rotation2d.fromDegrees(90).minus(r1);
        double x1 = STAGE_1_LENGTH * angle1.getCos();
        double y1 = STAGE_1_LENGTH * angle1.getSin();

        Rotation2d angle2 = r2.minus(Rotation2d.fromDegrees(90)).minus(r1);
        double x2 = STAGE_2_LENGTH * angle2.getCos();
        double y2 = STAGE_2_LENGTH * angle2.getSin();

        return new Translation2d(x1 + x2, y1 + y2);
    }

    /**
     * Given the angles of the joints, return the position of the end effector.
     *
     * @param angles The angles of the joints
     * @return The position of the end effector
     */
    public static Translation2d forwardKinematics(Pair<Rotation2d, Rotation2d> angles) {
        return forwardKinematics(angles.getFirst(), angles.getSecond());
    }

    /**
     * Sets the set point for the arm's 2 stages of rotation.
     *
     * <p>Applies clamping to the setpoints.
     *
     * @param stage1Rotations The desired rotation for the first stage.
     * @param stage2Rotations The desired rotation for the second stage.
     */
    public void setSetpoint(Rotation2d stage1Rotations, Rotation2d stage2Rotations) {
        stage1Setpoint =
                MathUtil.clamp(stage1Rotations.getRotations(), STAGE_1_HOME, STAGE_1_LIMIT);
        stage2Setpoint =
                MathUtil.clamp(stage2Rotations.getRotations(), STAGE_2_HOME, STAGE_2_LIMIT);

        stage1.getPIDController().setReference(stage1Setpoint, CANSparkMax.ControlType.kPosition);
        stage2.getPIDController().setReference(stage2Setpoint, CANSparkMax.ControlType.kPosition);
    }

    /**
     * Sets the set point for the arm's 2 stages of rotation.
     *
     * @param rotations a triple of rotations to set the set point to
     */
    public void setSetpoint(Pair<Rotation2d, Rotation2d> rotations) {
        setSetpoint(rotations.getFirst(), rotations.getSecond());
    }

    /**
     * Sets the set point for the arm's 2 stages of rotation.
     *
     * @param pose the pose to set the set point to
     */
    public void setSetpoint(Translation2d position) {
        setSetpoint(inverseKinematics(position));
    }

    /**
     * Gets the set point for the arm as a pair of rotations.
     *
     * @return a triple of rotations
     */
    public Pair<Rotation2d, Rotation2d> getSetpointRotations() {
        return new Pair<>(new Rotation2d(stage1Setpoint), new Rotation2d(stage2Setpoint));
    }

    /**
     * Gets the set point for the arm as a Translation2d.
     *
     * @return the position of the set point
     */
    public Translation2d getSetpointPosition() {
        return forwardKinematics(getSetpointRotations());
    }

    /** returns true if the arm is close to the setpoint, within the tolerance */
    public boolean isAtSetPoint() {
        Pair<Rotation2d, Rotation2d> setpoint = getSetpointRotations();
        Pair<Rotation2d, Rotation2d> current = getRotations();

        boolean stage1AtGoal =
                Math.abs(setpoint.getFirst().getRotations() - current.getFirst().getRotations())
                        < STAGE_1_TOLERANCE;
        boolean stage2AtGoal =
                Math.abs(setpoint.getSecond().getRotations() - current.getSecond().getRotations())
                        < STAGE_2_TOLERANCE;

        return stage1AtGoal && stage2AtGoal;
    }

    /**
     * Applies an offset to the arm subsystem.
     *
     * @param stage1Delta the change in rotation for stage 1
     * @param stage2Delta the change in rotation for stage 2
     */
    public void addSetPoint(Rotation2d stage1Delta, Rotation2d stage2Delta) {
        Pair<Rotation2d, Rotation2d> rotations = getSetpointRotations();

        Rotation2d stage1 = rotations.getFirst().rotateBy(stage2Delta);
        Rotation2d stage2 = rotations.getSecond().rotateBy(stage2Delta);

        setSetpoint(stage1, stage2);
    }

    /**
     * Adds an offset to the arm subsystem.
     *
     * @param rotations a pair of rotations to move the set point by
     */
    public void addSetPoint(Pair<Rotation2d, Rotation2d> rotations) {
        addSetPoint(rotations.getFirst(), rotations.getSecond());
    }

    /**
     * Adds an offset to the arm subsystem.
     *
     * @param poseDelta
     */
    public void addSetPoint(Translation2d poseDelta) {
        Translation2d pose = getSetpointPosition().plus(poseDelta);
        setSetpoint(pose);
    }

    /**
     * Gets the current rotations of the arm.
     *
     * @return a triple of rotations
     */
    public Pair<Rotation2d, Rotation2d> getRotations() {
        Rotation2d r1 = Rotation2d.fromRotations(stage1.getEncoder().getPosition());
        Rotation2d r2 = Rotation2d.fromRotations(stage2.getEncoder().getPosition());
        return new Pair<>(r1, r2);
    }

    /** Gets the current position of the arm. */
    public Translation2d getPose() {
        return forwardKinematics(getRotations());
    }

    /**
     * The distance metric used for the pathfinding algorithm. Currently this uses kinematics to
     * calculate the distance between two waypoints.
     *
     * @param a first waypoint
     * @param b second waypoint
     * @return the distance between the two waypoints
     */
    public static double distance(WAYPOINT a, WAYPOINT b) {
        // run the forward kinematics on the two waypoints
        Translation2d aTranslation = a.getTranslation();
        Translation2d bTranslation = b.getTranslation();

        return aTranslation.getDistance(bTranslation);
    }

    /**
     * Uses dijkstra's algorithm to find the shortest path between the two waypoints.
     *
     * <p>https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm
     *
     * @param start waypoint
     * @param end waypoint
     * @return A SequentialCommandGroup
     */
    public static ArrayList<WAYPOINT> dijkstra(WAYPOINT start, WAYPOINT end) {
        // Priority queue for the waypoints
        PriorityQueue<Pair<Double, WAYPOINT>> queue =
                new PriorityQueue<>(Comparator.comparing(Pair::getFirst));

        HashMap<WAYPOINT, WAYPOINT> previous = new HashMap<WAYPOINT, WAYPOINT>();
        HashMap<WAYPOINT, Double> distance = new HashMap<WAYPOINT, Double>();

        // Initialize the queue
        for (WAYPOINT waypoint : WAYPOINT.values()) {
            distance.put(waypoint, Double.POSITIVE_INFINITY);
            previous.put(waypoint, null);
        }

        // Add the start waypoint to the queue
        queue.add(new Pair<Double, WAYPOINT>(0.0, start));
        distance.put(start, 0.0);

        while (!queue.isEmpty()) {
            // Get the waypoint with the smallest distance
            WAYPOINT u = queue.poll().getSecond();

            // Loop through all the neighbors
            for (WAYPOINT v : ADJACENCY_LIST.get(u)) {
                // Calculate the distance from start to the neighbor
                double alt = distance.get(u) + distance(u, v);

                // If new the distance is less than the current distance, update the distance
                if (alt < distance.get(v)) {
                    distance.put(v, alt);
                    previous.put(v, u);
                    queue.add(new Pair<Double, WAYPOINT>(alt, v));
                }
            }
        }

        // Build the path
        ArrayList<WAYPOINT> path = new ArrayList<WAYPOINT>();

        // Start at the end and work backwards
        WAYPOINT current = end;
        while (current != start) {
            path.add(current);
            current = previous.get(current);
        }
        // path.add(start); // include the starting waypoint as part of the path

        // Reverse the path
        Collections.reverse(path);
        return path;
    }

    public CommandBase buildPath(WAYPOINT start, WAYPOINT end) {
        ArrayList<WAYPOINT> path = dijkstra(start, end);

        // Build the command group
        SequentialCommandGroup commandGroup = new SequentialCommandGroup();
        for (WAYPOINT waypoint : path) {
            commandGroup.addCommands(new ArmWaypoint(this, waypoint));
        }

        return commandGroup;
    }

    public WAYPOINT waypointTarget() {
        if (currentGamePieceTarget.equals(GAME_PIECE_TYPE.CUBE)) {
            switch (currentArmTarget) {
                case HOME:
                    return WAYPOINT.HOME;
                case LOW:
                    return WAYPOINT.LOW_CUBE;
                case MID:
                    return WAYPOINT.MID_CUBE;
                case HIGH:
                    return WAYPOINT.HIGH_CUBE;
                case DOUBLE_SUBSTATION:
                    return WAYPOINT.DOUBLE_SUBSTATION_CUBE;
                default:
                    return WAYPOINT.HOME;
            }
        } else {
            switch (currentArmTarget) {
                case HOME:
                    return WAYPOINT.HOME;
                case LOW:
                    return WAYPOINT.LOW_CONE;
                case MID:
                    return WAYPOINT.MID_CONE;
                case DOUBLE_SUBSTATION:
                    return WAYPOINT.DOUBLE_SUBSTATION_CONE;
                default:
                    return WAYPOINT.HOME;
            }
        }
    }

    /**
     * Sets game piece to cube when gamePieceCube is called and set to cone when gamePieceCone is
     * called.
     */
    public void gamePieceCube() {
        currentGamePieceTarget = GAME_PIECE_TYPE.CUBE;
    }

    public void gamePieceCone() {
        currentGamePieceTarget = GAME_PIECE_TYPE.CONE;
    }

    public CommandBase cGoToWaypoint(ARM_TARGET target) {
        return Commands.sequence(new InstantCommand(() -> setArmTarget(target)), new ArmNext(this));
    }

    public void setArmTarget(ARM_TARGET target) {
        currentArmTarget = target;
    }

    public void setPreviousArmWaypoint(WAYPOINT waypoint) {
        previousArmWaypoint = waypoint;
    }

    public WAYPOINT getPreviousArmWaypoint() {
        return previousArmWaypoint;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("Stage 1 Limit Switch", () -> !stage1LimitSwitch.get(), null);

        builder.addDoubleProperty(
                "Stage 1", () -> this.getRotations().getFirst().getRotations(), null);
        builder.addDoubleProperty(
                "Stage 2", () -> this.getRotations().getSecond().getRotations(), null);

        builder.addDoubleProperty("Stage 1 Setpoint", () -> stage1Setpoint, null);
        builder.addDoubleProperty("Stage 2 Setpoint", () -> stage2Setpoint, null);

        builder.addDoubleProperty("Stage 1 Temperature", stage1::getMotorTemperature, null);
        builder.addDoubleProperty("Stage 2 Temperature", stage2::getMotorTemperature, null);

        builder.addDoubleProperty("Stage 1 Output", stage1::getAppliedOutput, null);
        builder.addDoubleProperty("Stage 2 Output", stage2::getAppliedOutput, null);
    }
}
