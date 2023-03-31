package org.tvhsfrc.frc2023.robot.subsystems;

import static org.tvhsfrc.frc2023.robot.Constants.Arm.*;

import com.revrobotics.*;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ProfiledPIDController;
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
    private final CANSparkMax stage3 =
            new CANSparkMax(
                    Constants.CANConstants.ARM_STAGE_THREE,
                    CANSparkMaxLowLevel.MotorType.kBrushless);

    private double stage1SetPoint;
    private double stage2SetPoint;
    private double stage3SetPoint;

    private GAME_PIECE_TYPE currentGamePieceTarget = GAME_PIECE_TYPE.CONE;
    private WAYPOINT previousArmWaypoint = WAYPOINT.HOME;
    private ARM_TARGET currentArmTarget = ARM_TARGET.HOME;

    private final ProfiledPIDController stage1PidController = Constants.Arm.STAGE_1_PID;

    public ArmSubsystem() {
        // Stage 1
        stage1.setInverted(false);
        stage1.setIdleMode(CANSparkMax.IdleMode.kBrake);
        stage1.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        stage1.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
        stage1.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) STAGE_1_LIMIT);
        stage1PidController.setTolerance(STAGE_1_TOLERANCE);

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

    public boolean isAtSetPoint() {
        // Compare the current position to the set point
        boolean stage1AtGoal = stage1PidController.atGoal();
        boolean stage2AtGoal = Math.abs(getStage2Rotations() - stage2SetPoint) < STAGE_2_TOLERANCE;
        boolean stage3AtGoal = Math.abs(getStage3Rotations() - stage3SetPoint) < STAGE_3_TOLERANCE;

        return stage1AtGoal && stage2AtGoal && stage3AtGoal;
    }

    private double calculateStage1Output() {
        double output = stage1PidController.calculate(getStage1Rotations());

        if (!stage1LimitSwitch.get()) {
            output = Math.max(output, 0);
        }

        return output;
    }

    public void setStage1Output() {
        stage1.set(calculateStage1Output());
    }

    @Override
    public void periodic() {
        if (!stage1LimitSwitch.get()) {
            stage1Encoder.setPosition(0);
        }
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
//        path.add(start); // include the starting waypoint as part of the path

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
        builder.addDoubleProperty("Stage 3", this::getStage3Rotations, null);

        builder.addDoubleProperty("Stage 1 slop", this::slop, null);
        builder.addDoubleProperty("outer", () -> stage1Encoder.getPosition(), null);

        builder.addDoubleProperty(
                "Stage 1 Position Set Point",
                () -> stage1PidController.getSetpoint().position,
                this::setStage1Rotations);
        builder.addDoubleProperty(
                "Stage 1 Velocity Set Point",
                () -> stage1PidController.getSetpoint().velocity,
                null);

        builder.addDoubleProperty(
                "Stage 2 Set Point", () -> stage2SetPoint, this::setStage2Rotations);
        builder.addDoubleProperty(
                "Stage 3 Set Point", () -> stage3SetPoint, this::setStage3Rotations);

        builder.addDoubleProperty("PID Stage 1 P", this::getStage1P, this::setStage1P);
        builder.addDoubleProperty("PID Stage 1 I", this::getStage1I, this::setStage1I);
        builder.addDoubleProperty("PID Stage 1 D", this::getStage1D, this::setStage1D);
        builder.addDoubleProperty(
                "PID Stage 1 Min Output", this::getStage1MinOutput, this::setStage1MinOutput);
        builder.addDoubleProperty(
                "PID Stage 1 Max Output", this::getStage1MaxOutput, this::setStage1MaxOutput);
        builder.addDoubleProperty(
                "PID Stage 1 tolerance", this::getStage1Tolerance, this::setStage1Tolerance);

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

        builder.addDoubleProperty("Stage 1 Output", stage1::getAppliedOutput, null);
        builder.addDoubleProperty("Stage 1 Calculate", this::calculateStage1Output, null);
        builder.addDoubleProperty("Stage 1 Velocity", this::getStage1Vel, null);
        builder.addBooleanProperty("Stage 1 Limit Switch", () -> !stage1LimitSwitch.get(), null);
        builder.addDoubleProperty("Stage 2 Output", stage2::getAppliedOutput, null);
        builder.addDoubleProperty("Stage 3 Output", stage3::getAppliedOutput, null);
    }

    public double getStage1P() {
        return stage1PidController.getP();
    }

    public void setStage1P(double p) {
        stage1PidController.setP(p);
    }

    public double getStage1I() {
        return stage1PidController.getI();
    }

    public void setStage1I(double i) {
        stage1PidController.setI(i);
    }

    public double getStage1D() {
        return stage1PidController.getD();
    }

    public void setStage1D(double d) {
        stage1PidController.setD(d);
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

    public double getStage1Tolerance() {
        return stage1PidController.getPositionTolerance();
    }

    public void setStage1Tolerance(double tolerance) {
        stage1PidController.setTolerance(tolerance);
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
        stage1PidController.setGoal(stage1Rotations);
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

    public double getStage1SetPoint() {
        return stage1PidController.getGoal().position;
    }

    public double getStage2SetPoint() {
        return stage2SetPoint;
    }

    public double getStage3SetPoint() {
        return stage3SetPoint;
    }
}
