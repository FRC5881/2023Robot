package org.tvhsfrc.frc2023.robot.subsystems;

import static org.tvhsfrc.frc2023.robot.Constants.Arm.*;

import com.revrobotics.*;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import org.tvhsfrc.frc2023.robot.Constants;
import org.tvhsfrc.frc2023.robot.utils.RotationUtil;

public class ArmSubsystem extends SubsystemBase {
    private final DigitalInput stage1LimitSwitch =
            new DigitalInput(Constants.DIOConstants.STAGE_1_LIMIT_SWITCH);

    private final CANSparkMax stage2 =
            new CANSparkMax(
                    Constants.CANConstants.ARM_STAGE_2, CANSparkMaxLowLevel.MotorType.kBrushless);

    private boolean stage1Homed = true;
    private double stage2Setpoint = 0;

    public ArmSubsystem() {
        // Stage 2 motor controller setup
        setStage2P(STAGE_2_PID.p);
        setStage2I(STAGE_2_PID.i);
        setStage2D(STAGE_2_PID.d);

        stage2.setIdleMode(CANSparkMax.IdleMode.kBrake);
        stage2.getEncoder().setPositionConversionFactor(1 / GEARBOX_RATIO_STAGE_2);
        stage2.getEncoder().setVelocityConversionFactor(1 / GEARBOX_RATIO_STAGE_2);
        stage2.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);
        stage2.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
        stage2.setSoftLimit(
                CANSparkMax.SoftLimitDirection.kForward, (float) STAGE_2_LIMIT.getRotations());
        stage2.setSoftLimit(
                CANSparkMax.SoftLimitDirection.kReverse, (float) STAGE_2_HOME.getRotations());
        stage2.getEncoder().setPosition(STAGE_2_HOME.getRotations());
        stage2.getPIDController().setOutputRange(STAGE_2_MIN_OUTPUT, STAGE_2_MAX_OUTPUT);

        stage2.burnFlash();

        SmartDashboard.putData("Arm", this);

        setSetpoint(true, STAGE_2_HOME);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("isHome", stage1Homed);
        SmartDashboard.putBoolean("switch", stage1LimitSwitch.get());

        if (stage1Homed) {
        } else {
            // stage1.getPIDController()
            //         .setReference(STAGE_1_AWAY.getRotations(),
            // CANSparkMax.ControlType.kPosition);
        }

        stage2.getPIDController().setReference(stage2Setpoint, CANSparkMax.ControlType.kPosition);
    }

    public Rotation2d getStage2Setpoint() {
        return Rotation2d.fromRotations(stage2Setpoint);
    }

    /** returns true if the arm is close to the setpoint, within the tolerance */
    public boolean isAtSetPoint() {
        Pair<Rotation2d, Rotation2d> current = getRotations();

        boolean s1 =
                RotationUtil.withinTolerance(current.getFirst(), STAGE_1_HOME, STAGE_1_TOLERANCE);
        boolean s2 =
                RotationUtil.withinTolerance(
                        current.getSecond(), getStage2Setpoint(), STAGE_2_TOLERANCE);

        return s1 && s2;
    }

    /**
     * Sets the setpoint of the arm.
     *
     * @param stage1 the setpoint of the first stage
     * @param stage2 the setpoint of the second stage
     */
    public void setSetpoint(boolean stage1Homed, Rotation2d stage2) {
        this.stage1Homed = stage1Homed;
        this.stage2Setpoint =
                RotationUtil.clamp(stage2, STAGE_2_HOME, STAGE_2_LIMIT).getRotations();
    }

    /**
     * Set only stage 2 setpoint
     *
     * @param stage2 the setpoint of the second stage
     */
    public void setStage2Setpoint(Rotation2d stage2) {
        this.stage2Setpoint =
                RotationUtil.clamp(stage2, STAGE_2_HOME, STAGE_2_LIMIT).getRotations();
    }

    /**
     * Gets the current rotations of the arm.
     *
     * @return a triple of rotations
     */
    public Pair<Rotation2d, Rotation2d> getRotations() {
        Rotation2d r2 = Rotation2d.fromRotations(stage2.getEncoder().getPosition());
        return new Pair<>(new Rotation2d(), r2);
    }

    /** Gets the stage 2 position as a TrapezoidProfile.State */
    public TrapezoidProfile.State getStage2Position() {
        return new TrapezoidProfile.State(
                stage2.getEncoder().getPosition(), stage2.getEncoder().getVelocity());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("Stage 1 Limit Switch", () -> !stage1LimitSwitch.get(), null);

        builder.addDoubleProperty(
                "Stage 1", () -> this.getRotations().getFirst().getRotations(), null);
        builder.addDoubleProperty(
                "Stage 2", () -> this.getRotations().getSecond().getRotations(), null);

        builder.addDoubleProperty(
                "Stage 2 Velocity", () -> stage2.getEncoder().getVelocity(), null);

        builder.addDoubleProperty(
                "Stage 2 Voltage", () -> stage2.getBusVoltage() * stage2.getAppliedOutput(), null);

        builder.addDoubleProperty(
                "Stage 1 Setpoint",
                () -> {
                    if (stage1Homed) {
                        return 0.0;
                    } else {
                        return STAGE_1_AWAY.getRotations();
                    }
                },
                null);
        builder.addDoubleProperty("Stage 2 Setpoint", () -> stage2Setpoint, null);

        builder.addDoubleProperty("Stage 2 Temperature", stage2::getMotorTemperature, null);

        builder.addDoubleProperty("Stage 2 Output", stage2::getAppliedOutput, null);

        builder.addDoubleProperty("PID Stage 2 P", null, this::setStage2P);
        builder.addDoubleProperty("PID Stage 2 I", null, this::setStage2I);
        builder.addDoubleProperty("PID Stage 2 D", null, this::setStage2D);
    }

    public double getStage2P() {
        return stage2.getPIDController().getP();
    }

    public void setStage2P(double p) {
        stage2.getPIDController().setP(p);
    }

    public double getStage2I() {
        return stage2.getPIDController().getI();
    }

    public void setStage2I(double i) {
        stage2.getPIDController().setI(i);
    }

    public double getStage2D() {
        return stage2.getPIDController().getD();
    }

    public void setStage2D(double d) {
        stage2.getPIDController().setD(d);
    }
}
