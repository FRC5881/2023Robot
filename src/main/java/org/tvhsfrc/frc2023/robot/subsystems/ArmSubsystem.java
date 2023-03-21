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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.tvhsfrc.frc2023.robot.Constants;
import org.tvhsfrc.frc2023.robot.Constants.Arm;
import org.tvhsfrc.frc2023.robot.Constants.WayPoints;
import org.tvhsfrc.frc2023.robot.commands.ArmWaypointCommand;
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
        stage1.getPIDController().setP(0.0005);
        stage1.getPIDController().setI(0);
        stage1.getPIDController().setD(0);
        stage1.getPIDController().setFF(0);
        stage1.getPIDController().setOutputRange(-0.4, 0.4);
        stage1.getPIDController().setSmartMotionMaxVelocity(1200, 0);
        stage1.getPIDController().setSmartMotionMaxAccel(2400, 0);

        stage1.setInverted(false);

        stage1.setIdleMode(CANSparkMax.IdleMode.kBrake);
        stage1.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
        stage1.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
        // stage1.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) 0);
        // stage1.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) STAGE_1_LIMIT);

        // Stage 2
        stage2.getPIDController().setP(0.15);
        stage2.getPIDController().setI(0);
        stage2.getPIDController().setD(0.0);
        stage2.getPIDController().setFF(0.0);
        stage2.getPIDController().setOutputRange(-0.2, 0.2);

        stage2.setIdleMode(CANSparkMax.IdleMode.kBrake);
        stage2.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
        stage2.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
        // stage2.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) 0);
        // stage2.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) STAGE_2_LIMIT);

        // Stage 3
        stage3.getPIDController().setP(0.05);
        stage3.getPIDController().setI(0);
        stage3.getPIDController().setD(0);
        stage3.getPIDController().setFF(0);
        stage1.getPIDController().setOutputRange(-0.2, 0.2);

        stage3.setIdleMode(CANSparkMax.IdleMode.kBrake);
        stage3.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
        stage3.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
        // stage3.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) 0);
        // stage3.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) STAGE_3_LIMIT);

        ShuffleboardTab tab = Shuffleboard.getTab("Arm");

        ShuffleboardLayout list = tab.getLayout("Commands", BuiltInLayouts.kList);
        list.add("cToggleMode", cToggleMode());
        list.add("cSetModeCube", cSetModeCube());
        list.add("cSetModeCone", cSetModeCone());
        list.add("cScoreBottom", cScoreBottom());
        list.add("cScoreMid", cScoreMiddle());
        list.add("cScoreHigh", cScoreTop());
        list.add("cStore", cStore());
        list.add("cSafety", cSafety());
        list.add("cHome", cHome());

        tab.add(this);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Pose X", () -> getCurrentPose().getTranslation().getX(), null);
        builder.addDoubleProperty("Pose Y", () -> getCurrentPose().getTranslation().getY(), null);
        builder.addDoubleProperty(
                "Pose Theta", () -> getCurrentPose().getRotation().getDegrees(), null);

        builder.addDoubleProperty("Target Pose X", () -> lastWaypoint.pose.getX(), null);
        builder.addDoubleProperty("Target Pose Y", () -> lastWaypoint.pose.getY(), null);
        builder.addDoubleProperty(
                "Target Pose Theta", () -> lastWaypoint.pose.getRotation().getDegrees(), null);

        builder.addBooleanProperty("isCubeMode", () -> mode, (mode) -> this.mode = mode);

        builder.addDoubleProperty("Stage 1", () -> stage1.getEncoder().getPosition(), null);
        builder.addDoubleProperty("Stage 2", () -> stage2.getEncoder().getPosition(), null);
        builder.addDoubleProperty("Stage 3", () -> stage3.getEncoder().getPosition(), null);

        builder.addDoubleProperty("Stage 1 output", () -> stage1.getAppliedOutput(), null);
        builder.addDoubleProperty("Stage 2 output", () -> stage2.getAppliedOutput(), null);
        builder.addDoubleProperty("Stage 3 output", () -> stage3.getAppliedOutput(), null);
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
        theta = theta.plus(Rotation2d.fromDegrees(180));

        return new Pose2d(x1 + x2, y1 + y2, theta);
    }

    /**
     * Makes arm go straight to a position. DOES NOT CHECK FOR SAFETY
     *
     * @param pose Input position the code will tell the arm to go.
     */
    public void setPose(Pose2d pose) {
        Triple<Rotation2d, Rotation2d, Rotation2d> angles = inverseKinematics(pose);
        stage1.getPIDController()
                .setReference(
                        angles.getA().getRotations() * Constants.Arm.GEARBOX_RATIO_STAGE_1,
                        CANSparkMax.ControlType.kSmartMotion);

        SmartDashboard.putNumber(
                "Stage 1 reference",
                angles.getA().getRotations() * Constants.Arm.GEARBOX_RATIO_STAGE_1);

        stage2.getPIDController()
                .setReference(
                        angles.getB().getRotations() * Constants.Arm.GEARBOX_RATIO_STAGE_2,
                        CANSparkMax.ControlType.kPosition);

        SmartDashboard.putNumber(
                "Stage 2 reference",
                angles.getB().getRotations() * Constants.Arm.GEARBOX_RATIO_STAGE_2);

        stage3.getPIDController()
                .setReference(
                        angles.getC().getRotations() * Constants.Arm.GEARBOX_RATIO_STAGE_3,
                        CANSparkMax.ControlType.kPosition);

        SmartDashboard.putNumber(
                "Stage 3 reference",
                angles.getC().getRotations() * Constants.Arm.GEARBOX_RATIO_STAGE_3);
    }

    public Pose2d getCurrentPose() {
        Rotation2d stage1Angle =
                Rotation2d.fromRotations(
                        stage1.getEncoder().getPosition() / Arm.GEARBOX_RATIO_STAGE_1
                                - (10d / 360d));
        Rotation2d stage2Angle =
                Rotation2d.fromRotations(
                        stage2.getEncoder().getPosition() / Arm.GEARBOX_RATIO_STAGE_2);
        Rotation2d stage3Angle =
                Rotation2d.fromRotations(
                        stage3.getEncoder().getPosition() / Arm.GEARBOX_RATIO_STAGE_3);
        return forwardKinematics(stage1Angle, stage2Angle, stage3Angle);
    }

    public boolean isAtPose(Pose2d targetPose) {
        Pose2d currentPose = getCurrentPose();
        double distance = targetPose.getTranslation().getDistance(currentPose.getTranslation());
        double angleDifference =
                Math.abs(targetPose.getRotation().minus(currentPose.getRotation()).getDegrees());

        SmartDashboard.putNumber("Distance", distance);
        SmartDashboard.putNumber("Angle difference", angleDifference);

        return (distance < DISTANCE_TOLERANCE && angleDifference < ANGLE_TOLERANCE);
    }

    public void setLastWaypoint(WayPoints lastWaypoint) {
        this.lastWaypoint = lastWaypoint;
    }

    public CommandBase buildPath(WayPoints end) {
        return new ArmWaypointCommand(this, end);

        // if (lastWaypoint.equals(WayPoints.SAFE) || end.equals(WayPoints.SAFE)) {
        //     return new SetArmWaypointCommand(this, end);
        // }

        // if (lastWaypoint.insideBot() && end.insideBot()) {
        //     return new SetArmWaypointCommand(this, end);
        // } else {
        //     return Commands.sequence(
        //             new SetArmWaypointCommand(this, WayPoints.SAFE),
        //             new SetArmWaypointCommand(this, end));
        // }
    }

    public CommandBase cToggleMode() {
        return runOnce(() -> mode = !mode);
    }

    public CommandBase cSetModeCube() {
        return runOnce(() -> mode = true);
    }

    public CommandBase cSetModeCone() {
        return runOnce(() -> mode = false);
    }

    public CommandBase cScoreBottom() {
        if (mode) {
            return buildPath(WayPoints.CUBE_BOTTOM);
        } else {
            return buildPath(WayPoints.CONE_BOTTOM);
        }
    }

    public CommandBase cScoreMiddle() {
        if (mode) {
            return buildPath(WayPoints.CUBE_MIDDLE);
        } else {
            return buildPath(WayPoints.CONE_MIDDLE);
        }
    }

    public CommandBase cScoreTop() {
        if (mode) {
            return buildPath(WayPoints.CUBE_TOP);
        } else {
            return buildPath(WayPoints.CONE_TOP);
        }
    }

    public CommandBase cStore() {
        if (mode) {
            return buildPath(WayPoints.CUBE_STORE);
        } else {
            return buildPath(WayPoints.CONE_STORE);
        }
    }

    public CommandBase cSafety() {
        return new ArmWaypointCommand(this, WayPoints.SAFE);
    }

    public CommandBase cHome() {
        return buildPath(WayPoints.HOME);
    }
}
