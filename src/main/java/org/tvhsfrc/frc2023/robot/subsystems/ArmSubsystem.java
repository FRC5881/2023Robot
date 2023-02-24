package org.tvhsfrc.frc2023.robot.subsystems;

import static org.tvhsfrc.frc2023.robot.Constants.Arm.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.Pair;
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

    public void setStage1(double angle){
        stage1.getPIDController().setReference(angle * (GEARBOX_RATIO_STAGE_1 / 360d), CANSparkMax.ControlType.kPosition);
    }

    public void setStage2(double angle){
        stage1.getPIDController().setReference(angle * (GEARBOX_RATIO_STAGE_2 / 360d), CANSparkMax.ControlType.kPosition);
    }

    public void setStage3(double angle){
        stage1.getPIDController().setReference(angle * (GEARBOX_RATIO_STAGE_3 / 360d), CANSparkMax.ControlType.kPosition);
    }

    /**
     * This just takes the inputs for the Law of Cosines, and spits out the desired angle, gamma.
     *
     * @param a a leg
     * @param b a leg
     * @param c side opposite the returned angle (gamma)
     * @return Gives the value of gamma in radians
     */
    private static double lawOfCosines(double a, double b, double c) {
        return Math.acos((a * a + b * b - c) / (2 * a * b));
    }

    /**
     * This uses the lawOfCosines to find theta and beta. With theta, we find alpha
     *
     * @param translation
     * @return This returns alpha and beta
     */
    private static Pair<Rotation2d, Rotation2d> inverseKinematics(Translation2d translation) {
        double theta = lawOfCosines(STAGE_1_LENGTH, translation.getNorm(), STAGE_2_LENGTH);
        double alpha = translation.getAngle().getRadians() + theta;

        double beta = lawOfCosines(STAGE_1_LENGTH, STAGE_2_LENGTH, translation.getNorm());

        return new Pair<>(Rotation2d.fromRadians(alpha), Rotation2d.fromRadians(beta));
    }

    /** Take a and delta, return x and y as a Double Pair. */
    public static void kinematics(Rotation2d alpha, Rotation2d beta, Rotation2d theta) {

        double delta = Math.PI - (theta.getRadians() + beta.getRadians());
        double betaHorizontal = alpha.getRadians() + beta.getRadians() - Math.PI;

        double x_arm =
                (STAGE_1_LENGTH * alpha.getCos() + STAGE_2_LENGTH * Math.cos(betaHorizontal));
        double y_arm =
                (STAGE_1_LENGTH * alpha.getSin() + STAGE_2_LENGTH * Math.sin(betaHorizontal));
    }
    // Do we need/want Stage 3 included here?

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
