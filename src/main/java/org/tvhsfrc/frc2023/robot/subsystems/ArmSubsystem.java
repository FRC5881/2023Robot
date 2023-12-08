package org.tvhsfrc.frc2023.robot.subsystems;

import static org.tvhsfrc.frc2023.robot.Constants.Arm.*;
import static org.tvhsfrc.frc2023.robot.Constants.CANConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import org.tvhsfrc.frc2023.robot.Constants.WAYPOINT;

public class ArmSubsystem extends SubsystemBase {
    /** Arm Motor controller, should only be driven using the `setVoltage(v)` method */
    private final CANSparkMax armMotor;

    public static final boolean MANUAL = true;

    /**
     * V = ks * sign(omega) + kg * cos(theta) + kv * omega
     *
     * <p>Takes units of radians
     *
     * <p>theta: arm angle. 0 degrees must correspond to parallel with the ground. Which means at
     * HOME the arm is at -180 degress omega: rotational velocity
     *
     * <p>Since we use a trapeziodal motion profile, and the arm acceleration is discontinuous, we
     * leave that term out.
     *
     * <p>We rely on the PID controller to give us acceleration.
     */
    private final ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0);

    /**
     * Arm PID Control.
     *
     * <p>P should be enough, PD might create better preformance
     */
    private final PIDController pidController = new PIDController(0, 0, 0);

    private TrapezoidProfile.State m_state;
    private TrapezoidProfile.State m_goal;

    public ArmSubsystem() {

        // Stage 2 motor controller setup
        armMotor = new CANSparkMax(CANConstants.ARM_STAGE_2, MotorType.kBrushless);

        armMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        armMotor.getEncoder().setPositionConversionFactor(1 / CONVERSION_FACTOR);
        armMotor.getEncoder().setVelocityConversionFactor(1 / CONVERSION_FACTOR);
        armMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        armMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        armMotor.setSoftLimit(
                CANSparkMax.SoftLimitDirection.kForward,
                (float) MathUtil.angleModulus(LIMIT.getRadians()));
        armMotor.setSoftLimit(
                CANSparkMax.SoftLimitDirection.kReverse,
                (float) MathUtil.angleModulus(LIMIT.getRadians()));
        armMotor.getEncoder().setPosition(HOME.getRadians());

        armMotor.burnFlash();

        m_state = new TrapezoidProfile.State(HOME.getRadians(), 0);
        setGoal(HOME);
    }

    @Override
    public void periodic() {
        if (!MANUAL) {
            var profile = new TrapezoidProfile(CONSTRAINTS, m_goal, m_state);
            m_state = profile.calculate(0.02);

            double ff = feedforward.calculate(m_state.position, m_state.velocity);
            double pid = pidController.calculate(getPosition(), m_state.position);

            double voltage = ff + pid;
            double effect = MathUtil.clamp(voltage, MIN_OUTPUT, MAX_OUTPUT);

            armMotor.setVoltage(effect);

            SmartDashboard.putNumber("Arm/Setpoint", m_state.position);
            SmartDashboard.putNumber("Arm/Feedforward", ff);
            SmartDashboard.putNumber("Arm/PID", pid);
            SmartDashboard.putNumber("Arm/Requested Voltage", voltage);
            SmartDashboard.putNumber("Arm/Effect", effect);
            SmartDashboard.putNumber(
                    "Arm/Real Voltage", armMotor.getAppliedOutput() * armMotor.getBusVoltage());
        } else {
            SmartDashboard.putNumber(
                    "Arm/Real Voltage", armMotor.getAppliedOutput() * armMotor.getBusVoltage());
            SmartDashboard.putNumber("Arm/Position (degrees)", getRotation().getDegrees());
            SmartDashboard.putNumber(
                    "Arm/Kg",
                    armMotor.getAppliedOutput()
                            * armMotor.getBusVoltage()
                            / getRotation().getCos());
        }
    }

    public void setVoltage(double v) {
        if (MANUAL) {
            armMotor.setVoltage(v);
        }
    }

    /**
     * Requests that the Arm targets a new goal.
     *
     * @param goal Rotation2d of the goal
     */
    public void setGoal(Rotation2d goal) {
        // Clamp the goal between soft limits
        double g = MathUtil.clamp(goal.getRadians(), HOME.getRadians(), LIMIT.getRadians());
        m_goal = new TrapezoidProfile.State(g, 0);

        SmartDashboard.putNumber("Arm/Goal", m_goal.position);
    }

    /**
     * @return the current goal
     */
    public Rotation2d getGoal() {
        return Rotation2d.fromRadians(m_goal.position);
    }

    /**
     * @return current arm position as reported by the encoder
     */
    private double getPosition() {
        return armMotor.getEncoder().getPosition();
    }

    /**
     * @return current arm position as reported by the encoder (as a Rotation2d)
     */
    public Rotation2d getRotation() {
        return Rotation2d.fromRadians(getPosition());
    }

    /**
     * @return current arm velocity as reported by the encoder
     */
    private double getVelocity() {
        return armMotor.getEncoder().getVelocity();
    }

    /**
     * @return current arm State as reported by the encoder
     */
    private TrapezoidProfile.State getState() {
        return new TrapezoidProfile.State(getPosition(), getVelocity());
    }

    /**
     * @return true if the Arm position is within the tolerance for being considered at the setpoint
     */
    public boolean atGoal() {
        return Math.abs(m_goal.position - getPosition()) < TOLERANCE.getRadians();
    }

    /**
     * Resets the simulated arm state to match the physical state of the arm. When the robot is
     * disabled, this method should be called.
     */
    public void resetState() {
        m_state = getState();
    }

    /**
     * Builds a new command that sets the Arm to target a new goal.
     *
     * @param goal goal
     * @param wait if true then this command blocks until the arm reaches the setpoint, otherwise it
     *     finishes instantly
     * @return a new command
     */
    public Command setArmGoalCommand(Rotation2d goal, boolean wait) {
        Command cSetGoal = runOnce(() -> setGoal(goal));
        if (wait) {
            return cSetGoal.andThen(Commands.waitUntil(this::atGoal));
        } else {
            return cSetGoal;
        }
    }

    /**
     * Builds a new command that sets the Arm to target a new goal.
     *
     * @param goal goal
     * @param wait if true then this command blocks until the arm reaches the setpoint, otherwise it
     *     finishes instantly
     * @return a new command
     */
    public Command setArmGoalCommand(WAYPOINT goal, boolean wait) {
        return setArmGoalCommand(goal.getAngle(), wait);
    }
}
