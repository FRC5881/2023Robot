package org.tvhsfrc.frc2023.robot.subsystems;

import static org.tvhsfrc.frc2023.robot.Constants.Arm.*;

import com.revrobotics.*;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.PriorityQueue;
import org.tvhsfrc.frc2023.robot.Constants;
import org.tvhsfrc.frc2023.robot.Constants.WAYPOINT;
import org.tvhsfrc.frc2023.robot.commands.arm.ArmWaypoint;

public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax stage1 =
            new CANSparkMax(
                    Constants.CANConstants.ARM_STAGE_ONE, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax stage2 =
            new CANSparkMax(
                    Constants.CANConstants.ARM_STAGE_TWO, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax stage3 =
            new CANSparkMax(
                    Constants.CANConstants.ARM_STAGE_THREE,
                    CANSparkMaxLowLevel.MotorType.kBrushless);
    private final RelativeEncoder stage1Encoder =
            stage1.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 2048);

    private double stage1SetPoint;
    private double stage2SetPoint;
    private double stage3SetPoint;

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

        stage1.setInverted(false);
        stage1.setIdleMode(CANSparkMax.IdleMode.kBrake);
        stage1.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        stage1.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        stage1.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) STAGE_1_HOME);
        stage1.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) STAGE_1_LIMIT);
        stage1.getPIDController().setFeedbackDevice(stage1Encoder);
        stage1.getEncoder().setPositionConversionFactor(1 / GEARBOX_RATIO_STAGE_1);
        stage1.getEncoder().setVelocityConversionFactor(1 / GEARBOX_RATIO_STAGE_1);

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

        // Stage 3
        setStage3P(STAGE_3_PID.p);
        setStage3I(STAGE_3_PID.i);
        setStage3D(STAGE_3_PID.d);
        stage3.getPIDController().setFF(STAGE_3_PID.f);
        stage3.getPIDController().setOutputRange(STAGE_3_MIN_OUTPUT, STAGE_3_MAX_OUTPUT);

        stage3.setIdleMode(CANSparkMax.IdleMode.kBrake);
        stage3.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        stage3.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        stage3.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) 0);
        stage3.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) STAGE_3_LIMIT);
        stage3.getEncoder().setPositionConversionFactor(1 / GEARBOX_RATIO_STAGE_3);
        stage3.getEncoder().setVelocityConversionFactor(1 / GEARBOX_RATIO_STAGE_3);

        SmartDashboard.putData("Arm", this);

        // Start at home
        setStage1Rotations(STAGE_1_HOME);
        setStage2Rotations(0);
        setStage3Rotations(0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Stage 1", this::getStage1Rotations, null);
        builder.addDoubleProperty("Stage 2", this::getStage2Rotations, null);
        builder.addDoubleProperty("Stage 3", this::getStage3Rotations, null);

        builder.addDoubleProperty("Stage 1 slop", this::slop, null);
        builder.addDoubleProperty("inner", () -> stage1.getEncoder().getPosition(), null);
        builder.addDoubleProperty("outer", () -> stage1Encoder.getPosition(), null);
        builder.addDoubleProperty("calculate", () -> calculate(), null);

        builder.addDoubleProperty(
                "Stage 1 Set Point", () -> stage1SetPoint, this::setStage1Rotations);
        builder.addDoubleProperty(
                "Stage 2 Set Point", () -> stage2SetPoint, this::setStage2Rotations);
        builder.addDoubleProperty(
                "Stage 3 Set Point", () -> stage3SetPoint, this::setStage3Rotations);

        builder.addDoubleProperty("PID Stage 1 P", this::getStage1P, this::setStage1P);
        builder.addDoubleProperty("PID Stage 1 I", this::getStage1I, this::setStage1I);
        builder.addDoubleProperty("PID Stage 1 D", this::getStage1D, this::setStage1D);

        builder.addDoubleProperty("PID Stage 2 P", this::getStage2P, this::setStage2P);
        builder.addDoubleProperty("PID Stage 2 I", this::getStage2I, this::setStage2I);
        builder.addDoubleProperty("PID Stage 2 D", this::getStage2D, this::setStage2D);

        builder.addDoubleProperty("PID Stage 3 P", this::getStage3P, this::setStage3P);
        builder.addDoubleProperty("PID Stage 3 I", this::getStage3I, this::setStage3I);
        builder.addDoubleProperty("PID Stage 3 D", this::getStage3D, this::setStage3D);

        builder.addStringProperty("Game Piece", () -> currentGamePieceTarget.toString(), null);
        builder.addStringProperty("Arm Target", () -> currentArmTarget.toString(), null);
        builder.addStringProperty(
                "Previous Arm Waypoint", () -> previousArmWaypoint.toString(), null);
        builder.addBooleanProperty("isAtSetPoint", () -> this.isAtSetPoint(), null);

        builder.addDoubleProperty("Stage 1 Temperature", stage1::getMotorTemperature, null);
        builder.addDoubleProperty("Stage 2 Temperature", stage2::getMotorTemperature, null);
        builder.addDoubleProperty("Stage 3 Temperature", stage3::getMotorTemperature, null);

        builder.addDoubleProperty("Stage 1 Output", () -> 100 * stage1.getAppliedOutput(), null);
        builder.addDoubleProperty("Stage 2 Output", () -> 100 * stage2.getAppliedOutput(), null);
        builder.addDoubleProperty("Stage 3 Output", () -> 100 * stage3.getAppliedOutput(), null);
    }

    public double getStage1P() {
        return stage1.getPIDController().getP() / GEARBOX_RATIO_STAGE_1;
    }

    public void setStage1P(double p) {
        stage1.getPIDController().setP(p * GEARBOX_RATIO_STAGE_1);
    }

    public double getStage1I() {
        return stage1.getPIDController().getI() / GEARBOX_RATIO_STAGE_1;
    }

    public void setStage1I(double i) {
        stage1.getPIDController().setI(i * GEARBOX_RATIO_STAGE_1);
    }

    public double getStage1D() {
        return stage1.getPIDController().getD() / GEARBOX_RATIO_STAGE_1;
    }

    public void setStage1D(double d) {
        stage1.getPIDController().setD(d * GEARBOX_RATIO_STAGE_1);
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

    public double getStage3P() {
        return stage3.getPIDController().getP() / GEARBOX_RATIO_STAGE_3;
    }

    public void setStage3P(double p) {
        stage3.getPIDController().setP(p * GEARBOX_RATIO_STAGE_3);
    }

    public double getStage3I() {
        return stage3.getPIDController().getI() / GEARBOX_RATIO_STAGE_3;
    }

    public void setStage3I(double i) {
        stage3.getPIDController().setI(i * GEARBOX_RATIO_STAGE_3);
    }

    public double getStage3D() {
        return stage3.getPIDController().getD() / GEARBOX_RATIO_STAGE_3;
    }

    public void setStage3D(double d) {
        stage3.getPIDController().setD(d * GEARBOX_RATIO_STAGE_3);
    }

    public void setStage1Rotations(double stage1Rotations) {

        stage1.getPIDController().setReference(stage1Rotations, CANSparkMax.ControlType.kPosition);
        stage1SetPoint = stage1Rotations;
    }

    public double getStage1Rotations() {
        return stage1Encoder.getPosition();
    }

    public double getStage1Vel() {
        return stage1Encoder.getVelocity();
    }

    public void setStage2Rotations(double stage2Rotations) {
        stage2.getPIDController().setReference(stage2Rotations, CANSparkMax.ControlType.kPosition);
        stage2SetPoint = stage2Rotations;
    }

    public double getStage2Rotations() {
        return stage2.getEncoder().getPosition();
    }

    public void setStage3Rotations(double stage3Rotations) {
        stage3.getPIDController().setReference(stage3Rotations, CANSparkMax.ControlType.kPosition);
        stage3SetPoint = stage3Rotations;
    }

    public double getStage3Rotations() {
        return stage3.getEncoder().getPosition();
    }

    public boolean isAtSetPoint() {
        // Compare the current position to the set point
        return Math.abs(getStage1Rotations() - stage1SetPoint) < Constants.Arm.STAGE_1_TOLERANCE
                && Math.abs(getStage2Rotations() - stage2SetPoint) < Constants.Arm.STAGE_2_TOLERANCE
                && Math.abs(getStage3Rotations() - stage3SetPoint)
                        < Constants.Arm.STAGE_3_TOLERANCE;
    }

    /**
     * Given the angles of the joints, return the position of the end effector.
     *
     * @param r1 The angle of the first joint
     * @param r2 The angle of the second joint
     * @return The position of the end effector
     */
    public static Translation2d forwardKinematics(Rotation2d r1, Rotation2d r2) {
        r1 = Rotation2d.fromDegrees(90).minus(r1);

        double x1 = STAGE_1_LENGTH * r1.getCos();
        double y1 = STAGE_1_LENGTH * r1.getSin();

        Rotation2d angle2 = r1.plus(r2).minus(Rotation2d.fromDegrees(180));
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
        Translation2d aPos =
                forwardKinematics(
                        Rotation2d.fromRotations(a.position.getA()),
                        Rotation2d.fromRotations(a.position.getB()));
        Translation2d bPos =
                forwardKinematics(
                        Rotation2d.fromRotations(b.position.getA()),
                        Rotation2d.fromRotations(b.position.getB()));

        // return the distance between the two points
        return aPos.getDistance(bPos);
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
                // Calculate the distance to the neighbor
                double alt = distance.get(u) + distance(u, v);

                // If the distance is less than the current distance, update the distance
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
        path.add(start); // include the starting waypoint as part of the path

        // Reverse the path
        Collections.reverse(path);

        return path;
    }

    public CommandBase buildPath(WAYPOINT start, WAYPOINT end) {
        ArrayList<WAYPOINT> path = dijkstra(start, end);

        // Build the command group
        SequentialCommandGroup commandGroup = new SequentialCommandGroup();
        for (WAYPOINT waypoint : path) {
            commandGroup.addCommands(new ArmWaypoint(this, waypoint).withTimeout(5));
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

    /** Toggles the game piece type between cube and cone. */
    public void toggleGamePiece() {
        if (currentGamePieceTarget.equals(GAME_PIECE_TYPE.CUBE)) {
            currentGamePieceTarget = GAME_PIECE_TYPE.CONE;
        } else {
            currentGamePieceTarget = GAME_PIECE_TYPE.CUBE;
        }
    }

    /** Cycles though the available ARM_TARGET values. */
    public void cycleArmTarget(boolean reverse) {
        if (reverse) {
            switch (currentArmTarget) {
                case HOME:
                    currentArmTarget = ARM_TARGET.DOUBLE_SUBSTATION;
                    break;
                case SAFE:
                    currentArmTarget = ARM_TARGET.HOME;
                    break;
                case FLOOR:
                    currentArmTarget = ARM_TARGET.SAFE;
                    break;
                case LOW:
                    currentArmTarget = ARM_TARGET.FLOOR;
                    break;
                case MID:
                    currentArmTarget = ARM_TARGET.LOW;
                    break;
                case HIGH:
                    currentArmTarget = ARM_TARGET.MID;
                    break;
                case DOUBLE_SUBSTATION:
                    currentArmTarget = ARM_TARGET.HIGH;
                    break;
            }
        } else {
            switch (currentArmTarget) {
                case HOME:
                    currentArmTarget = ARM_TARGET.SAFE;
                    break;
                case SAFE:
                    currentArmTarget = ARM_TARGET.FLOOR;
                    break;
                case FLOOR:
                    currentArmTarget = ARM_TARGET.LOW;
                    break;
                case LOW:
                    currentArmTarget = ARM_TARGET.MID;
                    break;
                case MID:
                    currentArmTarget = ARM_TARGET.HIGH;
                    break;
                case HIGH:
                    currentArmTarget = ARM_TARGET.DOUBLE_SUBSTATION;
                    break;
                case DOUBLE_SUBSTATION:
                    currentArmTarget = ARM_TARGET.HOME;
                    break;
            }
        }
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

    public double calculate() {
        double error = getStage1Rotations() - stage1SetPoint;

        return getStage1P() * error;
    }
}
