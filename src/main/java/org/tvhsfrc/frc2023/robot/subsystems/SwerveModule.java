package org.tvhsfrc.frc2023.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.tvhsfrc.frc2023.robot.Constants;
import org.tvhsfrc.frc2023.robot.Constants.SwerveModuleConstants;

/**
 * Models a single swerve module.
 *
 * <p>This class is responsible for controlling individual swerve modules by passing in wpilib
 * SwerveModuleState objects.
 */
public class SwerveModule extends SubsystemBase {
    // Two RevNeo motors per module.
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    // Additional absolute encoder for module heading.
    private final CANCoder turnEncoder;

    /**
     * PID controller for the turn motor. Using the turnEncoder as the feedback sensor.
     *
     * <p>If you are looking to turn this PID controller you should use Shuffleboard. In DriveTrain
     * the subsystem there are persisent sliders for the PID constants.
     */
    private final PIDController turnController = new PIDController(0, 0, 0);

    // The previous output of the turnController
    private double turnOutput;

    // Previous targeted state
    private SwerveModuleState state;

    /**
     * Creates a new SwerveModule.
     *
     * <p>After constructing a SwerveModule you should call setDrivePID and setTurnPID
     */
    public SwerveModule(SwerveModuleConstants moduleConfig) {
        setName(moduleConfig.name);

        // Drive motor configuration
        driveMotor =
                new CANSparkMax(moduleConfig.driveMotorCANID, CANSparkMax.MotorType.kBrushless);
        driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        driveMotor.setSmartCurrentLimit(Constants.Swerve.DRIVE_CURRENT_LIMIT);
        driveMotor
                .getEncoder()
                .setPositionConversionFactor(Constants.Swerve.DRIVE_CONVERSION_FACTOR);
        driveMotor
                .getEncoder()
                .setVelocityConversionFactor(Constants.Swerve.DRIVE_CONVERSION_FACTOR);

        // Turn motor configuration
        turnMotor = new CANSparkMax(moduleConfig.turnMotorCANID, CANSparkMax.MotorType.kBrushless);
        turnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        turnMotor.setSmartCurrentLimit(Constants.Swerve.TURN_CURRENT_LIMIT);

        // Turn encoder configuration.
        turnEncoder = new CANCoder(moduleConfig.turnEncoderCANID);
        CANCoderConfiguration encoderConfig = new CANCoderConfiguration();
        encoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        encoderConfig.magnetOffsetDegrees = moduleConfig.turnEncoderOffset;
        encoderConfig.sensorDirection = true; // clockwise -> positive
        encoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;

        ErrorCode error = turnEncoder.configAllSettings(encoderConfig);
        if (error != ErrorCode.OK) {
            System.out.println(
                    "Error configuring CANCoder " + moduleConfig.turnEncoderCANID + ":" + error);
        }

        // Turn PID controller configuration
        turnController.enableContinuousInput(0, 360);
        turnController.setSetpoint(0); // initially target 0 degrees
        turnController.setTolerance(0.25);

        resetEncoder();
    }

    /**
     * Kinematics.
     *
     * @param state The desired state of the module.
     */
    public void drive(SwerveModuleState state) {
        this.state = state;

        // If there is no meaningful speed, stop.
        // This prevents WPILIB from recentering the wheels when you release the controller.
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
        }

        // Optimize the state to minimize the turn angle by flipping the drive
        // direction.
        //
        // For example, if the angle is currently 180 degrees and the desired angle is
        // -180 degrees, we can flip the drive direction and set the angle to 0 degrees.
        Rotation2d currentAngle = Rotation2d.fromDegrees(turnEncoder.getPosition());
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, currentAngle);

        // Set the drive motor to target the desired speed.
        driveMotor
                .getPIDController()
                .setReference(optimizedState.speedMetersPerSecond, ControlType.kVelocity);

        // Set the turn pid controller to target the desired angle.
        turnController.setSetpoint(optimizedState.angle.getDegrees());

        // Update the turn motor with the turn pid controller output.
        turnOutput = turnController.calculate(turnEncoder.getAbsolutePosition());
        turnOutput = MathUtil.clamp(turnOutput, -1, 1);
        turnMotor.set(turnOutput);
    }

    /** Odometry (angle and speed) */
    public SwerveModuleState getState() {
        // Get the current angle from the turn encoder.
        Rotation2d currentAngle = Rotation2d.fromDegrees(turnEncoder.getAbsolutePosition());

        // Get the current speed from the drive encoder.
        double currentSpeed = driveMotor.getAbsoluteEncoder(Type.kDutyCycle).getVelocity();

        return new SwerveModuleState(currentSpeed, currentAngle);
    }

    /** Odometry (angle and distance) */
    public SwerveModulePosition getPosition() {
        // Get the current angle from the turn encoder.
        Rotation2d currentAngle = Rotation2d.fromDegrees(turnEncoder.getAbsolutePosition());

        // Get the distance traveled from the drive encoder.
        double position = driveMotor.getEncoder().getPosition();

        return new SwerveModulePosition(position, currentAngle);
    }

    /** Zeros the drive encoder */
    public void resetEncoder() {
        driveMotor.getEncoder().setPosition(0);
    }

    public void stop() {
        driveMotor.stopMotor();
        turnMotor.stopMotor();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType(getName());

        // measured speed
        builder.addDoubleProperty("Drive speed (mps)", driveMotor.getEncoder()::getVelocity, null);

        builder.addDoubleProperty(
                "Turn Absolute Position (degrees)", turnEncoder::getAbsolutePosition, null);

        builder.addDoubleProperty(
                "Turn PID output",
                () -> {
                    return turnOutput;
                },
                null);

        builder.addDoubleProperty("Drive applied output", driveMotor::getAppliedOutput, null);

        // setpoints
        builder.addDoubleProperty(
                "Drive setpoint (mps)",
                () -> {
                    return state.speedMetersPerSecond;
                },
                (speed) -> {
                    state.speedMetersPerSecond = speed;
                });

        builder.addDoubleProperty(
                "Turn setpoint (degrees)",
                state.angle::getDegrees,
                (angle) -> {
                    state.angle = Rotation2d.fromDegrees(angle);
                });
    }

    /**
     * Set drive motor P constant.
     *
     * @param kP
     */
    public void setDriveP(double kP) {
        // Only make the change if the value is different.
        if (kP == driveMotor.getPIDController().getP()) {
            return;
        }

        REVLibError error = driveMotor.getPIDController().setP(kP);
        if (error != REVLibError.kOk) {
            System.out.println(
                    "Error setting drive motor " + driveMotor.getDeviceId() + " kP:" + error);
        }
    }

    /**
     * Set drive motor I constant.
     *
     * @param kI
     */
    public void setDriveI(double kI) {
        // Only make the change if the value is different.
        if (kI == driveMotor.getPIDController().getI()) {
            return;
        }

        REVLibError error = driveMotor.getPIDController().setI(kI);
        if (error != REVLibError.kOk) {
            System.out.println(
                    "Error setting drive motor " + driveMotor.getDeviceId() + " kI:" + error);
        }
    }

    /**
     * Set drive motor D constant.
     *
     * @param kD
     */
    public void setDriveD(double kD) {
        // Only make the change if the value is different.
        if (kD == driveMotor.getPIDController().getD()) {
            return;
        }

        REVLibError error = driveMotor.getPIDController().setD(kD);
        if (error != REVLibError.kOk) {
            System.out.println(
                    "Error setting drive motor " + driveMotor.getDeviceId() + " kD:" + error);
        }
    }

    /**
     * Set drive motor PID constants.
     *
     * @param kP
     * @param kI
     * @param kD
     */
    public void setDrivePID(double kP, double kI, double kD) {
        setDriveP(kP);
        setDriveI(kI);
        setDriveD(kD);
    }

    /** Set turn motor P constant. */
    public void setTurnP(double kP) {
        turnController.setP(kP);
    }

    /** Set turn motor I constant. */
    public void setTurnI(double kI) {
        turnController.setI(kI);
    }

    /** Set turn motor D constant. */
    public void setTurnD(double kD) {
        turnController.setD(kD);
    }

    /**
     * Set turn motor PID constants.
     *
     * @param kP
     * @param kI
     * @param kD
     */
    public void setTurnPID(double kP, double kI, double kD) {
        setTurnP(kP);
        setTurnI(kI);
        setTurnD(kD);
    }
}
