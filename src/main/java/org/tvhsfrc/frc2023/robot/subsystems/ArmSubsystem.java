package org.tvhsfrc.frc2023.robot.subsystems;

import static org.tvhsfrc.frc2023.robot.Constants.Arm.*;

import com.revrobotics.*;
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
    private final CANSparkMax stage1 =
            new CANSparkMax(
                    Constants.CANConstants.ARM_STAGE_ONE, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final RelativeEncoder stage1Encoder =
            stage1.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 2048);
    private final DigitalInput stage1LimitSwitch =
            new DigitalInput(Constants.DIOConstants.STAGE_1_LIMIT_SWITCH);

    private final CANSparkMax stage2 =
            new CANSparkMax(
                    Constants.CANConstants.ARM_STAGE_TWO, CANSparkMaxLowLevel.MotorType.kBrushless);

    private double stage1SetPoint;
    private double stage2SetPoint;

    private GAME_PIECE_TYPE currentGamePieceTarget = GAME_PIECE_TYPE.CONE;
    private WAYPOINT previousArmWaypoint = WAYPOINT.HOME;
    private ARM_TARGET currentArmTarget = ARM_TARGET.HOME;

    public ArmSubsystem() {
        // Stage 1
        setStage1P(STAGE_1_PID.p);
        setStage1I(STAGE_1_PID.i);
        setStage1D(STAGE_1_PID.d);
        stage1.getPIDController().setFF(STAGE_1_PID.f);
        stage1.getPIDController().setOutputRange(STAGE_1_MIN_OUTPUT, STAGE_1_MAX_OUTPUT);

        stage1.setInverted(true);
        stage1.setIdleMode(CANSparkMax.IdleMode.kCoast);
        stage1.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        stage1.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        stage1.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) STAGE_1_LIMIT);
        stage1.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) 0);

        // Stage 2
        setStage2P(STAGE_2_PID.p);
        setStage2I(STAGE_2_PID.i);
        setStage2D(STAGE_2_PID.d);
        stage2.getPIDController().setFF(STAGE_2_PID.f);
        stage2.getPIDController().setOutputRange(STAGE_2_MIN_OUTPUT, STAGE_2_MAX_OUTPUT);

        stage2.setIdleMode(CANSparkMax.IdleMode.kBrake);
        stage2.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        stage2.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        stage2.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) 0);
        stage2.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) STAGE_2_LIMIT);
        stage2.getEncoder().setPositionConversionFactor(1 / GEARBOX_RATIO_STAGE_2);
        stage2.getEncoder().setVelocityConversionFactor(1 / GEARBOX_RATIO_STAGE_2);

        SmartDashboard.putData("Arm", this);

        // Start at home
        setStage1Rotations(0);
        setStage2Rotations(0);
    }

    public boolean isAtSetPoint() {
        // Compare the current position to the set point
        boolean stage1AtGoal = Math.abs(getStage1Rotations() - stage1SetPoint) < STAGE_1_TOLERANCE;
        boolean stage2AtGoal = Math.abs(getStage2Rotations() - stage2SetPoint) < STAGE_2_TOLERANCE;

        return stage1AtGoal && stage2AtGoal;
    }

    @Override
    public void periodic() {
        if (!stage1LimitSwitch.get()) {
            stage1Encoder.setPosition(0);
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
                case SAFE:
                    return WAYPOINT.SAFE;
                case FLOOR:
                    return WAYPOINT.FLOOR_CUBE;
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
                case SAFE:
                    return WAYPOINT.SAFE;
                case FLOOR:
                    return WAYPOINT.FLOOR_CONE;
                case LOW:
                    return WAYPOINT.LOW_CONE;
                case MID:
                    return WAYPOINT.MID_CONE;
                case HIGH:
                    return WAYPOINT.HIGH_CONE;
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

    // Calculate slop
    public double slop() {
        double inner = stage1.getEncoder().getPosition();
        double outer = stage1Encoder.getPosition();

        return inner - outer;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Stage 1", this::getStage1Rotations, null);
        builder.addDoubleProperty("Stage 2", this::getStage2Rotations, null);

        builder.addDoubleProperty("Stage 1 slop", this::slop, null);
        builder.addDoubleProperty("outer", () -> stage1Encoder.getPosition(), null);

        builder.addDoubleProperty(
                "Stage 1 Set Point", () -> stage1SetPoint, this::setStage1Rotations);
        builder.addDoubleProperty(
                "Stage 2 Set Point", () -> stage2SetPoint, this::setStage2Rotations);

        builder.addDoubleProperty("PID Stage 1 P", this::getStage1P, this::setStage1P);
        builder.addDoubleProperty("PID Stage 1 I", this::getStage1I, this::setStage1I);
        builder.addDoubleProperty("PID Stage 1 D", this::getStage1D, this::setStage1D);
        builder.addDoubleProperty(
                "PID Stage 1 Min Output", this::getStage1MinOutput, this::setStage1MinOutput);
        builder.addDoubleProperty(
                "PID Stage 1 Max Output", this::getStage1MaxOutput, this::setStage1MaxOutput);

        builder.addDoubleProperty("PID Stage 2 P", this::getStage2P, this::setStage2P);
        builder.addDoubleProperty("PID Stage 2 I", this::getStage2I, this::setStage2I);
        builder.addDoubleProperty("PID Stage 2 D", this::getStage2D, this::setStage2D);

        builder.addStringProperty("Game Piece", () -> currentGamePieceTarget.toString(), null);
        builder.addStringProperty("Arm Target", () -> currentArmTarget.toString(), null);
        builder.addStringProperty(
                "Previous Arm Waypoint", () -> previousArmWaypoint.toString(), null);
        builder.addBooleanProperty("isAtSetPoint", () -> this.isAtSetPoint(), null);

        builder.addDoubleProperty("Stage 1 Temperature", stage1::getMotorTemperature, null);
        builder.addDoubleProperty("Stage 2 Temperature", stage2::getMotorTemperature, null);

        builder.addDoubleProperty("Stage 1 Output", stage1::getAppliedOutput, null);
        builder.addDoubleProperty("Stage 1 Velocity", this::getStage1Vel, null);
        builder.addBooleanProperty("Stage 1 Limit Switch", () -> !stage1LimitSwitch.get(), null);
        builder.addDoubleProperty("Stage 2 Output", stage2::getAppliedOutput, null);
    }

    public double getStage1P() {
        return stage1.getPIDController().getP();
    }

    public void setStage1P(double p) {
        stage1.getPIDController().setP(p);
    }

    public double getStage1I() {
        return stage1.getPIDController().getI();
    }

    public void setStage1I(double i) {
        stage1.getPIDController().setI(i);
    }

    public double getStage1D() {
        return stage1.getPIDController().getD();
    }

    public void setStage1D(double d) {
        stage1.getPIDController().setD(d);
    }

    public double getStage1MinOutput() {
        return stage1.getPIDController().getOutputMin();
    }

    public double getStage1MaxOutput() {
        return stage1.getPIDController().getOutputMax();
    }

    public void setStage1MinOutput(double min) {
        stage1.getPIDController().setOutputRange(min, getStage1MaxOutput());
    }

    public void setStage1MaxOutput(double max) {
        stage1.getPIDController().setOutputRange(getStage1MinOutput(), max);
    }

    public double getStage2P() {
        return stage2.getPIDController().getP() / GEARBOX_RATIO_STAGE_2;
    }

    public void setStage2P(double p) {
        stage2.getPIDController().setP(p * GEARBOX_RATIO_STAGE_2);
    }

    public double getStage2I() {
        return stage2.getPIDController().getI() / GEARBOX_RATIO_STAGE_2;
    }

    public void setStage2I(double i) {
        stage2.getPIDController().setI(i * GEARBOX_RATIO_STAGE_2);
    }

    public double getStage2D() {
        return stage2.getPIDController().getD() / GEARBOX_RATIO_STAGE_2;
    }

    public void setStage2D(double d) {
        stage2.getPIDController().setD(d * GEARBOX_RATIO_STAGE_2);
    }

    public void setStage1Rotations(double stage1Rotations) {
        stage1.getPIDController().setReference(stage1SetPoint, CANSparkMax.ControlType.kPosition);
        stage1SetPoint = stage1Rotations;
    }

    public double getStage1Rotations() {
        return stage1.getEncoder().getPosition();
    }

    public double getStage1Vel() {
        return stage1.getEncoder().getVelocity();
    }

    public void setStage2Rotations(double stage2Rotations) {
        stage2.getPIDController().setReference(stage2Rotations, CANSparkMax.ControlType.kPosition);
        stage2SetPoint = stage2Rotations;
    }

    public double getStage2Rotations() {
        return stage2.getEncoder().getPosition();
    }

    public double getStage1SetPoint() {
        return stage1SetPoint;
    }

    public double getStage2SetPoint() {
        return stage2SetPoint;
    }
}
