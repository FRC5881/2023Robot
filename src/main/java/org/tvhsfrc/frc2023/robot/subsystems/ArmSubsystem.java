package org.tvhsfrc.frc2023.robot.subsystems;

import static org.tvhsfrc.frc2023.robot.Constants.Arm.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.tvhsfrc.frc2023.robot.Constants;
import org.tvhsfrc.frc2023.robot.utils.Triple;

public class ArmSubsystem extends SubsystemBase {
    private Pose2d targetPose;

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

    public ArmSubsystem() {
        // TODO: Configure motor controllers with constants - PID constants

        // Stage 1
        stage1.getPIDController().setP(1);
        stage1.getPIDController().setI(1);
        stage1.getPIDController().setD(1);
        stage1.getPIDController().setFF(1);
        stage1.setClosedLoopRampRate(1);
        stage1.setIdleMode(CANSparkMax.IdleMode.kBrake);

        stage1.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) STAGE_1_LIMIT);
        stage1.getPIDController()
                .setOutputRange(0, stage1.getSoftLimit(CANSparkMax.SoftLimitDirection.kReverse));

        // Stage 2
        stage2.getPIDController().setP(1);
        stage2.getPIDController().setI(1);
        stage2.getPIDController().setD(1);
        stage2.getPIDController().setFF(1);
        stage2.setClosedLoopRampRate(1);
        stage2.setIdleMode(CANSparkMax.IdleMode.kBrake);

        stage2.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) STAGE_2_LIMIT);
        stage2.getPIDController()
                .setOutputRange(0, stage2.getSoftLimit(CANSparkMax.SoftLimitDirection.kForward));

        // Stage 3
        stage3.getPIDController().setP(1);
        stage3.getPIDController().setI(1);
        stage3.getPIDController().setD(1);
        stage3.getPIDController().setFF(1);
        stage3.setClosedLoopRampRate(1);
        stage3.setIdleMode(CANSparkMax.IdleMode.kBrake);

        stage3.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) STAGE_3_LIMIT);
        stage3.getPIDController()
                .setOutputRange(0, stage3.getSoftLimit(CANSparkMax.SoftLimitDirection.kForward));
    }

    /** Stage 1 */
    private final ShuffleboardTab tab = Shuffleboard.getTab("Arm");

    ShuffleboardLayout stage1PIDList =
            tab.getLayout("Stage 1 PID", BuiltInLayouts.kList).withPosition(6, 0).withSize(1, 2);

    private GenericEntry stage1Kp =
            stage1PIDList
                    .addPersistent("kP", 0)
                    .withWidget(BuiltInWidgets.kTextView)
                    .getEntry("double");

    private GenericEntry stage1Ki =
            stage1PIDList
                    .addPersistent("kI", 0)
                    .withWidget(BuiltInWidgets.kTextView)
                    .getEntry("double");

    private GenericEntry stage1Kd =
            stage1PIDList
                    .addPersistent("kD", 0)
                    .withWidget(BuiltInWidgets.kTextView)
                    .getEntry("double");

    /** Stage 2 */
    ShuffleboardLayout stage2PIDList =
            tab.getLayout("Stage 2 PID", BuiltInLayouts.kList).withPosition(6, 0).withSize(1, 2);

    private GenericEntry stage2Kp =
            stage2PIDList
                    .addPersistent("kP", 0)
                    .withWidget(BuiltInWidgets.kTextView)
                    .getEntry("double");

    private GenericEntry stage2Ki =
            stage2PIDList
                    .addPersistent("kI", 0)
                    .withWidget(BuiltInWidgets.kTextView)
                    .getEntry("double");

    private GenericEntry stage2Kd =
            stage2PIDList
                    .addPersistent("kD", 0)
                    .withWidget(BuiltInWidgets.kTextView)
                    .getEntry("double");

    /** Stage 3 */
    ShuffleboardLayout stage3PIDList =
            tab.getLayout("Stage 3 PID", BuiltInLayouts.kList).withPosition(6, 0).withSize(1, 2);

    private GenericEntry stage3Kp =
            stage3PIDList
                    .addPersistent("kP", 0)
                    .withWidget(BuiltInWidgets.kTextView)
                    .getEntry("double");

    private GenericEntry stage3Ki =
            stage3PIDList
                    .addPersistent("kI", 0)
                    .withWidget(BuiltInWidgets.kTextView)
                    .getEntry("double");

    private GenericEntry stage3Kd =
            stage3PIDList
                    .addPersistent("kD", 0)
                    .withWidget(BuiltInWidgets.kTextView)
                    .getEntry("double");

    /** Updates the PID values in each of the ARM motors to match the NT Values */
    public void updatePID() {
        runOnce(
                () -> {
                    stage1.getPIDController().setP(stage1Kp.getDouble(0));
                    stage1.getPIDController().setI(stage1Ki.getDouble(0));
                    stage1.getPIDController().setD(stage1Kd.getDouble(0));

                    stage2.getPIDController().setP(stage2Kp.getDouble(0));
                    stage2.getPIDController().setI(stage2Ki.getDouble(0));
                    stage2.getPIDController().setD(stage2Kd.getDouble(0));

                    stage3.getPIDController().setP(stage3Kp.getDouble(0));
                    stage3.getPIDController().setI(stage3Ki.getDouble(0));
                    stage3.getPIDController().setD(stage3Kd.getDouble(0));
                });
    }

    public void setStage1(double angle) {
        stage1.getPIDController()
                .setReference(
                        angle * (GEARBOX_RATIO_STAGE_1 / 360d), CANSparkMax.ControlType.kPosition);
    }

    public void setStage2(double angle) {
        stage1.getPIDController()
                .setReference(
                        angle * (GEARBOX_RATIO_STAGE_2 / 360d), CANSparkMax.ControlType.kPosition);
    }

    public void setStage3(double angle) {
        stage1.getPIDController()
                .setReference(
                        angle * (GEARBOX_RATIO_STAGE_3 / 360d), CANSparkMax.ControlType.kPosition);
    }

    /**
     * This just takes the inputs for the Law of Cosines, and spits out the desired angle, gamma.
     *
     * <p>If the edges suggest the triangle is impossible thsi returns NaN.
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
     * @param pose The position and angle of the end effector
     * @return The angles of the joints
     */
    public static Triple<Rotation2d, Rotation2d, Rotation2d> inverseKinematics(Pose2d pose) {
        Translation2d translation = pose.getTranslation();
        double c = translation.getNorm();

        System.out.println(c);
        System.out.println(STAGE_1_LENGTH - STAGE_2_LENGTH);

        // Outside the outer circle of the donut
        if (c > STAGE_1_LENGTH + STAGE_2_LENGTH || Math.abs(c - (STAGE_1_LENGTH + STAGE_2_LENGTH)) < 10e-6) {
            Rotation2d r1 = translation.getAngle();
            Rotation2d r2 = Rotation2d.fromDegrees(180);
            Rotation2d r3 = pose.getRotation().minus(r1).minus(r2);
            return new Triple<>(Rotation2d.fromDegrees(90).minus(r1), r2, r3);
        }

        // Inside the inner circle of the donut
        if (c > STAGE_1_LENGTH - STAGE_2_LENGTH || Math.abs(c - (STAGE_1_LENGTH - STAGE_2_LENGTH)) < 10e-6) {
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
     * @param a The angle of the first joint
     * @param b The angle of the second joint
     * @param c The angle of the third joint
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

    /**
     * @param x_arm The x component of the distance from the base of the arm to the tip of the arm
     * @param y_arm The y component of the distance from the base of the arm to the tip of the arm
     */
    // Is the tip of the arm the end of stage 2, or the end of stage 3/the grabber?
    // I.E. should stage 3 be included in kinematics?

    public void boundaryEst(double x_arm, double y_arm) {
        if (x_arm > 5) {
            setTargetPos(new Pose2d());
        }

        if (y_arm > 5) {
            setTargetPos(new Pose2d());
        }
    }

    public void setTargetPos(Pose2d targetPose) {
        this.targetPose = targetPose;
    }

    @Override
    public void periodic() {
        // Do the math
        // Drive motors
    }
}
