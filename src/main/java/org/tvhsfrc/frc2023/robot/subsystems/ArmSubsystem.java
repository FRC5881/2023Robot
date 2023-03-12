package org.tvhsfrc.frc2023.robot.subsystems;

import static org.tvhsfrc.frc2023.robot.Constants.Arm.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.tvhsfrc.frc2023.robot.Constants;
import org.tvhsfrc.frc2023.robot.Constants.WayPoints;
import org.tvhsfrc.frc2023.robot.commands.SetArmWaypointCommand;
import org.tvhsfrc.frc2023.robot.utils.Triple;

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
    private WayPoints lastWaypoint = WayPoints.HOME;
    private boolean mode = true;

    public ArmSubsystem() {
        // Stage 1
        stage1.getPIDController().setP(0);
        stage1.getPIDController().setI(0);
        stage1.getPIDController().setD(0);
        stage1.getPIDController().setFF(0);
        stage1.getPIDController().setOutputRange(-0.25, 0.25);
        stage1.setIdleMode(CANSparkMax.IdleMode.kBrake);

        stage1.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) 0);
        stage1.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) STAGE_1_LIMIT);
        stage1.getEncoder().setPositionConversionFactor(1 / GEARBOX_RATIO_STAGE_1);
        stage1.getEncoder().setVelocityConversionFactor(1 / GEARBOX_RATIO_STAGE_1);

        // Stage 2
        stage2.getPIDController().setP(0.018);
        stage2.getPIDController().setI(0);
        stage2.getPIDController().setD(0.005);
        stage2.getPIDController().setFF(0.001);
        stage2.getPIDController().setOutputRange(-0.2, 0.2);
        stage2.setIdleMode(CANSparkMax.IdleMode.kBrake);

        stage2.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) 0);
        stage2.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) STAGE_2_LIMIT);
        stage2.getEncoder().setPositionConversionFactor(1 / GEARBOX_RATIO_STAGE_2);
        stage2.getEncoder().setVelocityConversionFactor(1 / GEARBOX_RATIO_STAGE_2);

        // Stage 3
        stage3.getPIDController().setP(0);
        stage3.getPIDController().setI(0);
        stage3.getPIDController().setD(0);
        stage3.getPIDController().setFF(0);
        stage1.getPIDController().setOutputRange(-0.1, 0.1);
        stage3.setIdleMode(CANSparkMax.IdleMode.kBrake);

        stage3.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) 0);
        stage3.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) STAGE_3_LIMIT);
        stage3.getEncoder().setPositionConversionFactor(1 / GEARBOX_RATIO_STAGE_3);
        stage3.getEncoder().setVelocityConversionFactor(1 / GEARBOX_RATIO_STAGE_3);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Pose X", () -> getCurrentPose().getTranslation().getX(), null);
        builder.addDoubleProperty("Pose Y", () -> getCurrentPose().getTranslation().getY(), null);
        builder.addDoubleProperty(
                "Pose Theta",
                () -> getCurrentPose().getTranslation().getAngle().getDegrees(),
                null);

        builder.addDoubleProperty("Target Pose X", () -> lastWaypoint.pose.getX(), null);
        builder.addDoubleProperty("Target Pose Y", () -> lastWaypoint.pose.getY(), null);
        builder.addDoubleProperty(
                "Target Pose Theta", () -> lastWaypoint.pose.getRotation().getDegrees(), null);

        builder.addBooleanProperty("isCubeMode", () -> mode, (mode) -> this.mode = mode);

        ShuffleboardTab tab = Shuffleboard.getTab("Arm");

        tab.add("Stage 1", stage1);
        tab.add("Stage 2", stage2);
        tab.add("Stage 3", stage3);

        ShuffleboardLayout list = tab.getLayout("Commands", BuiltInLayouts.kList);
        list.add("cSetModeCube", cSetModeCube());
        list.add("cSetModeCone", cSetModeCone());
        list.add("cScoreLow", cScoreLow());
        list.add("cScoreMid", cScoreMid());
        list.add("cScoreHigh", cScoreHigh());
        list.add("cStore", cStore());
        list.add("cSafety", cSafety());
        list.add("cHome", cHome());
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
     * Given a Pose2d (a position & an angle) of the end effector return the angles of the joints.
     *
     * <p>If the target is out of reach, stage 1 and 2 will be made into a straight line that points
     * to the target.
     *
     * @param pose The position and angle of the end effector
     * @return The angles of the joints
     */
    public static Triple<Rotation2d, Rotation2d, Rotation2d> inverseKinematics(Pose2d pose) {
        Translation2d translation = pose.getTranslation();
        double c = translation.getNorm();

        // Outside the outer circle of the donut
        if (c > STAGE_1_LENGTH + STAGE_2_LENGTH
                || Math.abs(c - (STAGE_1_LENGTH + STAGE_2_LENGTH)) < 10e-6) {
            Rotation2d r1 = translation.getAngle();
            Rotation2d r2 = Rotation2d.fromDegrees(180);
            Rotation2d r3 = pose.getRotation().minus(r1).minus(r2);
            return new Triple<>(Rotation2d.fromDegrees(90).minus(r1), r2, r3);
        }

        // Inside the inner circle of the donut
        if (c < STAGE_1_LENGTH - STAGE_2_LENGTH
                || Math.abs(c - (STAGE_1_LENGTH - STAGE_2_LENGTH)) < 10e-6) {
            Rotation2d r1 = translation.getAngle();
            Rotation2d r2 = Rotation2d.fromDegrees(0);
            Rotation2d r3 = pose.getRotation().minus(r1).minus(r2);
            return new Triple<>(Rotation2d.fromDegrees(90).minus(r1), r2, r3);
        }

        double r1 =
                lawOfCosines(STAGE_1_LENGTH, c, STAGE_2_LENGTH)
                        + translation.getAngle().getRadians();
        double r2 = lawOfCosines(STAGE_1_LENGTH, STAGE_2_LENGTH, c);
        double r3 = pose.getRotation().getRadians() - r1 - r2;

        return new Triple<>(
                Rotation2d.fromRadians(0.5 * Math.PI - r1),
                Rotation2d.fromRadians(r2),
                Rotation2d.fromRadians(r3));
    }

    /**
     * Given the angles of the joints, return the position and angle of the end effector.
     *
     * @param r1 The angle of the first joint
     * @param r2 The angle of the second joint
     * @param r3 The angle of the third joint
     * @return The position and angle of the end effector
     */
    public static Pose2d forwardKinematics(Rotation2d r1, Rotation2d r2, Rotation2d r3) {
        r1 = Rotation2d.fromDegrees(90).minus(r1);

        double x1 = STAGE_1_LENGTH * r1.getCos();
        double y1 = STAGE_1_LENGTH * r1.getSin();

        Rotation2d angle2 = r1.plus(r2).minus(Rotation2d.fromDegrees(180));
        double x2 = STAGE_2_LENGTH * angle2.getCos();
        double y2 = STAGE_2_LENGTH * angle2.getSin();

        Rotation2d theta = r1.plus(r2).plus(r3);
        return new Pose2d(x1 + x2, y1 + y2, theta);
    }

    public void setPose(Pose2d pose) {
        Triple<Rotation2d, Rotation2d, Rotation2d> angles = inverseKinematics(pose);
        stage1.getPIDController()
                .setReference(angles.getA().getRotations(), CANSparkMax.ControlType.kPosition);
        stage2.getPIDController()
                .setReference(angles.getB().getRotations(), CANSparkMax.ControlType.kPosition);
        stage3.getPIDController()
                .setReference(angles.getC().getRotations(), CANSparkMax.ControlType.kPosition);
    }

    public Pose2d getCurrentPose() {
        Rotation2d stage1Angle =
                Rotation2d.fromRotations(stage1.getEncoder().getPosition() - (10d / 360d));
        Rotation2d stage2Angle = Rotation2d.fromRotations(stage2.getEncoder().getPosition());
        Rotation2d stage3Angle = Rotation2d.fromRotations(stage3.getEncoder().getPosition());
        return forwardKinematics(stage1Angle, stage2Angle, stage3Angle);
    }

    public boolean isAtPose(Pose2d targetPose) {
        Pose2d currentPose = getCurrentPose();
        double distance = targetPose.getTranslation().getDistance(currentPose.getTranslation());
        double angleDifference =
                targetPose.getRotation().minus(currentPose.getRotation()).getDegrees();
        return (distance < DISTANCE_TOLERANCE && angleDifference < ANGLE_TOLERANCE);
    }

    public void setLastWaypoint(WayPoints lastWaypoint) {
        this.lastWaypoint = lastWaypoint;
    }

    public Command buildPath(WayPoints end) {
        if (lastWaypoint == WayPoints.SAFE || end == WayPoints.SAFE) {
            return new SetArmWaypointCommand(this, WayPoints.SAFE);
        }

        if (lastWaypoint.insideBot() && end.insideBot()) {
            return new SetArmWaypointCommand(this, end);
        } else {
            return Commands.sequence(
                    new SetArmWaypointCommand(this, WayPoints.SAFE),
                    new SetArmWaypointCommand(this, end));
        }
    }

    public Command cSetModeCube() {
        return runOnce(() -> mode = true);
    }

    public Command cSetModeCone() {
        return runOnce(() -> mode = false);
    }

    public Command cScoreLow() {
        if (mode) {
            return buildPath(WayPoints.CUBE_BOTTOM);
        } else {
            return buildPath(WayPoints.CONE_BOTTOM);
        }
    }

    public Command cScoreMid() {
        if (mode) {
            return buildPath(WayPoints.CUBE_MIDDLE);
        } else {
            return buildPath(WayPoints.CONE_MIDDLE);
        }
    }

    public Command cScoreHigh() {
        if (mode) {
            return buildPath(WayPoints.CUBE_TOP);
        } else {
            return buildPath(WayPoints.CONE_TOP);
        }
    }

    public Command cStore() {
        if (mode) {
            return buildPath(WayPoints.CUBE_STORE);
        } else {
            return buildPath(WayPoints.CONE_STORE);
        }
    }

    public Command cSafety() {
        return new SetArmWaypointCommand(this, WayPoints.SAFE);
    }

    public Command cHome() {
        return new SetArmWaypointCommand(this, WayPoints.HOME);
    }
}
