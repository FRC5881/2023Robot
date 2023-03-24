package org.tvhsfrc.frc2023.robot.subsystems;

import static org.tvhsfrc.frc2023.robot.Constants.Arm.*;

import com.revrobotics.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.Collections;

import org.tvhsfrc.frc2023.robot.Constants;
import org.tvhsfrc.frc2023.robot.Constants.WayPoint;
import org.tvhsfrc.frc2023.robot.commands.ArmTrajectory;

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

    /** The current cube / cone mode of the arm. */
    private boolean mode = true;

    private double stage1setpoint = 0;
    private double stage2setpoint = 0;
    private double stage3setpoint = 0;

    private GAME_PIECE_TYPE currentGamePieceTarget = GAME_PIECE_TYPE.CONE;
    private ARM_TARGET currentArmTarget = ARM_TARGET.HOME;

    public ArmSubsystem() {
        // Stage 1
        setStage1P(STAGE_1_PID.p);
        setStage1I(STAGE_1_PID.i);
        setStage1D(STAGE_1_PID.d);
        stage1.getPIDController().setFF(STAGE_1_PID.f); // TODO: Scale this
        stage1.getPIDController().setOutputRange(STAGE_1_MIN_OUTPUT, STAGE_1_MAX_OUTPUT);

        stage1.setInverted(false);
        stage1.setIdleMode(CANSparkMax.IdleMode.kBrake);
        stage1.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        stage1.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        stage1.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) 0);
        stage1.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) STAGE_1_LIMIT);
        stage1.getPIDController().setFeedbackDevice(stage1Encoder);

        // Stage 2
        setStage2P(STAGE_2_PID.p);
        setStage2I(STAGE_2_PID.i);
        setStage2D(STAGE_2_PID.d);
        stage2.getPIDController().setFF(STAGE_2_PID.f); // TODO: Scale this
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
        stage3.getPIDController().setFF(STAGE_3_PID.f); // TODO: Scale this
        stage1.getPIDController().setOutputRange(STAGE_3_MIN_OUTPUT, STAGE_3_MAX_OUTPUT);

        stage3.setIdleMode(CANSparkMax.IdleMode.kBrake);
        stage3.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        stage3.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        stage3.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) 0);
        stage3.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) STAGE_3_LIMIT);
        stage3.getEncoder().setPositionConversionFactor(1 / GEARBOX_RATIO_STAGE_3);
        stage3.getEncoder().setVelocityConversionFactor(1 / GEARBOX_RATIO_STAGE_3);

        ShuffleboardTab tab = Shuffleboard.getTab("Arm");

        tab.add(this);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("isCubeMode", () -> mode, (mode) -> this.mode = mode);

        builder.addDoubleProperty("Stage 1", this::getStage1Rotations, null);
        builder.addDoubleProperty("Stage 2", this::getStage2Rotations, null);
        builder.addDoubleProperty("Stage 3", this::getStage3Rotations, null);

        builder.addDoubleProperty("Stage 1 setpoint", null, this::setStage1Rotations);
        builder.addDoubleProperty("Stage 2 setpoint", null, this::setStage2Rotations);
        builder.addDoubleProperty("Stage 3 setpoint", null, this::setStage3Rotations);

        builder.addDoubleProperty("PID Stage 1 P", this::getStage1P, this::setStage1P);
        builder.addDoubleProperty("PID Stage 1 I", this::getStage1I, this::setStage1I);
        builder.addDoubleProperty("PID Stage 1 D", this::getStage1D, this::setStage1D);

        builder.addDoubleProperty("PID Stage 2 P", this::getStage2P, this::setStage2P);
        builder.addDoubleProperty("PID Stage 2 I", this::getStage2I, this::setStage2I);
        builder.addDoubleProperty("PID Stage 2 D", this::getStage2D, this::setStage2D);

        builder.addDoubleProperty(
                "PID Stage 3 P", () -> stage3.getPIDController().getP(), this::setStage3P);
        builder.addDoubleProperty(
                "PID Stage 3 I", () -> stage3.getPIDController().getI(), this::setStage3I);
        builder.addDoubleProperty(
                "PID Stage 3 D", () -> stage3.getPIDController().getD(), this::setStage3D);

        builder.addStringProperty("Game Piece", currentGamePieceTarget::toString, null);
        builder.addStringProperty("Arm Target", currentArmTarget::toString, null);
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
    // private static double lawOfCosines(double a, double b, double c) {
    //     return Math.acos((a * a + b * b - c * c) / (2 * a * b));
    // }

    /**
     * Given a Pose2d (a position & an angle) of the end effector return the angles of the joints.
     *
     * <p>If the target is out of reach, stage 1 and 2 will be made into a straight line that points
     * to the target.
     *
     * @param pose The position and angle of the end effector
     * @return The angles of the joints
     */
    // public static Triple<Rotation2d, Rotation2d, Rotation2d> inverseKinematics(Pose2d pose) {
    //     Translation2d translation = pose.getTranslation();
    //     double c = translation.getNorm();

    //     // Outside the outer circle of the donut
    //     if (c > STAGE_1_LENGTH + STAGE_2_LENGTH
    //             || Math.abs(c - (STAGE_1_LENGTH + STAGE_2_LENGTH)) < 10e-6) {
    //         Rotation2d r1 = translation.getAngle();
    //         Rotation2d r2 = Rotation2d.fromDegrees(180);
    //         Rotation2d r3 = pose.getRotation().minus(r1).minus(r2);
    //         return new Triple<>(Rotation2d.fromDegrees(90).minus(r1), r2, r3);
    //     }

    //     // Inside the inner circle of the donut
    //     if (c < STAGE_1_LENGTH - STAGE_2_LENGTH
    //             || Math.abs(c - (STAGE_1_LENGTH - STAGE_2_LENGTH)) < 10e-6) {
    //         Rotation2d r1 = translation.getAngle();
    //         Rotation2d r2 = Rotation2d.fromDegrees(0);
    //         Rotation2d r3 = pose.getRotation().minus(r1).minus(r2);
    //         return new Triple<>(Rotation2d.fromDegrees(90).minus(r1), r2, r3);
    //     }

    //     double r1 =
    //             lawOfCosines(STAGE_1_LENGTH, c, STAGE_2_LENGTH)
    //                     + translation.getAngle().getRadians();
    //     double r2 = lawOfCosines(STAGE_1_LENGTH, STAGE_2_LENGTH, c);
    //     double r3 = pose.getRotation().getRadians() - r1 - r2;

    //     return new Triple<>(
    //             Rotation2d.fromRadians(0.5 * Math.PI - r1),
    //             Rotation2d.fromRadians(r2),
    //             Rotation2d.fromRadians(r3));
    // }

    /**
     * Given the angles of the joints, return the position and angle of the end effector.
     *
     * @param r1 The angle of the first joint
     * @param r2 The angle of the second joint
     * @param r3 The angle of the third joint
     * @return The position and angle of the end effector
     */
    // public static Pose2d forwardKinematics(Rotation2d r1, Rotation2d r2, Rotation2d r3) {
    //     r1 = Rotation2d.fromDegrees(90).minus(r1);

    //     double x1 = STAGE_1_LENGTH * r1.getCos();
    //     double y1 = STAGE_1_LENGTH * r1.getSin();

    //     Rotation2d angle2 = r1.plus(r2).minus(Rotation2d.fromDegrees(180));
    //     double x2 = STAGE_2_LENGTH * angle2.getCos();
    //     double y2 = STAGE_2_LENGTH * angle2.getSin();

    //     Rotation2d theta = r1.plus(r2).plus(r3);
    //     return new Pose2d(x1 + x2, y1 + y2, theta);
    // }

    /**
     * Makes arm go straight to a position. DOES NOT CHECK FOR SAFETY
     *
     * @param pose Input position the code will tell the arm to go.
     */
    // public void setPose(Pose2d pose) {
    //     Triple<Rotation2d, Rotation2d, Rotation2d> angles = inverseKinematics(pose);
    //     setPose(angles.getA(), angles.getB(), angles.getC());
    // }

    // private void setPose(Rotation2d stage1, Rotation2d stage2, Rotation2d stage3) {
    //     setStage1Rotations(stage1.getRotations());
    //     setStage2Rotations(stage2.getRotations());
    //     setStage3Rotations(stage3.getRotations());
    // }

    public void setStage1Rotations(double stage1Rotations) {
        stage1.getPIDController().setReference(stage1Rotations, CANSparkMax.ControlType.kPosition);
    }

    public void setStage1Rotations(Rotation2d stage1Rotations) {
        setStage1Rotations(stage1Rotations.getRotations());
    }

    public double getStage1Rotations() {
        return stage1Encoder.getPosition();
    }

    public void setStage2Rotations(double stage2Rotations) {
        stage2.getPIDController().setReference(stage2Rotations, CANSparkMax.ControlType.kPosition);
    }

    public void setStage2Rotations(Rotation2d stage2Rotations) {
        setStage2Rotations(stage2Rotations.getRotations());
    }

    public double getStage2Rotations() {
        return stage2.getEncoder().getPosition();
    }

    public void setStage3Rotations(double stage3Rotations) {
        stage3.getPIDController().setReference(stage3Rotations, CANSparkMax.ControlType.kPosition);
    }

    public void setStage3Rotations(Rotation2d stage3Rotations) {
        setStage3Rotations(stage3Rotations.getRotations());
    }

    public double getStage3Rotations() {
        return stage3.getEncoder().getPosition();
    }

    public boolean isAtSetpoint() {
        // Compare the current position to the setpoint
        return Math.abs(getStage1Rotations() - stage1setpoint) < Constants.Arm.STAGE_1_TOLERANCE
                && Math.abs(getStage2Rotations() - stage2setpoint) < Constants.Arm.STAGE_2_TOLERANCE
                && Math.abs(getStage3Rotations() - stage3setpoint)
                        < Constants.Arm.STAGE_3_TOLERANCE;
    }

    // public Pose2d getCurrentPose() {
    //     Rotation2d stage1Angle =
    //             Rotation2d.fromRotations(getStage1Rotations()).minus(Rotation2d.fromDegrees(10));
    //     Rotation2d stage2Angle =
    //             Rotation2d.fromRotations(getStage2Rotations());
    //     Rotation2d stage3Angle =
    //             Rotation2d.fromRotations(getStage3Rotations());

    //     return forwardKinematics(stage1Angle, stage2Angle, stage3Angle);
    // }

    // public boolean isAtPose(Pose2d targetPose) {
    //     Pose2d currentPose = getCurrentPose();
    //     double distance = targetPose.getTranslation().getDistance(currentPose.getTranslation());
    //     double angleDifference =
    //             Math.abs(targetPose.getRotation().minus(currentPose.getRotation()).getDegrees());
    //     return (distance < DISTANCE_TOLERANCE && angleDifference < ANGLE_TOLERANCE);
    // }

    public CommandBase buildPath(
            WayPoint start, WayPoint end) {
        if (start == WayPoint.HOME) {
            return new ArmTrajectory(this, Constants.Arm.HOME_PATHS.get(end));
        }

        if (end == WayPoint.HOME) {
            ArrayList<WayPoint> path = (ArrayList<WayPoint>) Constants.Arm.HOME_PATHS.get(start).clone();

            // remove the last element
            path.remove(path.size() - 1);

            // reverse the list
            Collections.reverse(path);

            // add HOME to the end
            path.add(WayPoint.HOME);

            return new ArmTrajectory(this, path);
        }

        // Otherwise just go home and then to the target
        return buildPath(start, WayPoint.HOME).andThen(buildPath(WayPoint.HOME, end));
    }


    /**
     * Toggles the game piece type between cube and cone.
     */
    public void toggleGamePiece() {
        if (currentGamePieceTarget.equals(GAME_PIECE_TYPE.CUBE)) {
            currentGamePieceTarget = GAME_PIECE_TYPE.CONE;
        } else {
            currentGamePieceTarget = GAME_PIECE_TYPE.CUBE;
        }
    }

    /**
     * Cycles though the available ARM_TARGET values.
     */
    public void cycleArmTarget() {
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
