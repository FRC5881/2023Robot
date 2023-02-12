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
        this.driveMotor =
                new CANSparkMax(moduleConfig.driveMotorCANID, CANSparkMax.MotorType.kBrushless);
        this.driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        this.driveMotor.setSmartCurrentLimit(Constants.Swerve.DRIVE_CURRENT_LIMIT);
        this.driveMotor
                .getEncoder()
                .setPositionConversionFactor(Constants.Swerve.DRIVE_CONVERSION_FACTOR);
        this.driveMotor
                .getEncoder()
                .setVelocityConversionFactor(Constants.Swerve.DRIVE_CONVERSION_FACTOR);

        // Turn motor configuration
        this.turnMotor =
                new CANSparkMax(moduleConfig.turnMotorCANID, CANSparkMax.MotorType.kBrushless);
        this.turnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        this.turnMotor.setSmartCurrentLimit(Constants.Swerve.TURN_CURRENT_LIMIT);

        // Turn encoder configuration.
        this.turnEncoder = new CANCoder(moduleConfig.turnEncoderCANID);
        CANCoderConfiguration encoderConfig = new CANCoderConfiguration();
        encoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        encoderConfig.magnetOffsetDegrees = moduleConfig.turnEncoderOffset;
        encoderConfig.sensorDirection = true; // clockwise -> positive
        encoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;

        ErrorCode error = this.turnEncoder.configAllSettings(encoderConfig);
        if (error != ErrorCode.OK) {
            System.out.println(
                    "Error configuring CANCoder " + moduleConfig.turnEncoderCANID + ":" + error);
        }

        // Turn PID controller configuration
        this.turnController.enableContinuousInput(0, 360);
        this.turnController.setSetpoint(0); // initially target 0 degrees
        this.turnController.setTolerance(0.25);

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
        Rotation2d currentAngle = Rotation2d.fromDegrees(this.turnEncoder.getPosition());
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, currentAngle);

        // Set the drive motor to target the desired speed.
        this.driveMotor
                .getPIDController()
                .setReference(optimizedState.speedMetersPerSecond, ControlType.kVelocity);

        // Set the turn pid controller to target the desired angle.
        this.turnController.setSetpoint(optimizedState.angle.getDegrees());

        // Update the turn motor with the turn pid controller output.
        this.turnOutput = this.turnController.calculate(this.turnEncoder.getAbsolutePosition());
        this.turnOutput = MathUtil.clamp(turnOutput, -1, 1);
        this.turnMotor.set(this.turnOutput);
    }

    /** Odometry (angle and speed) */
    public SwerveModuleState getState() {
        // Get the current angle from the turn encoder.
        Rotation2d currentAngle = Rotation2d.fromDegrees(this.turnEncoder.getAbsolutePosition());

        // Get the current speed from the drive encoder.
        double currentSpeed = this.driveMotor.getAbsoluteEncoder(Type.kDutyCycle).getVelocity();

        return new SwerveModuleState(currentSpeed, currentAngle);
    }

    /** Odometry (angle and distance) */
    public SwerveModulePosition getPosition() {
        // Get the current angle from the turn encoder.
        Rotation2d currentAngle = Rotation2d.fromDegrees(this.turnEncoder.getAbsolutePosition());

        // Get the distance traveled from the drive encoder.
        double position = this.driveMotor.getEncoder().getPosition();

        return new SwerveModulePosition(position, currentAngle);
    }

    /** Zeros the drive encoder */
    public void resetEncoder() {
        this.driveMotor.getEncoder().setPosition(0);
    }

    public void stop() {
        this.driveMotor.stopMotor();
        this.turnMotor.stopMotor();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType(getName());

        // measured speed
        builder.addDoubleProperty(
                "Drive speed (mps)", this.driveMotor.getEncoder()::getVelocity, null);

        builder.addDoubleProperty(
                "Turn Absolute Position (degrees)", this.turnEncoder::getAbsolutePosition, null);

        builder.addDoubleProperty(
                "Turn PID output",
                () -> {
                    return this.turnOutput;
                },
                null);

        builder.addDoubleProperty("Drive applied output", this.driveMotor::getAppliedOutput, null);

        // setpoints
        builder.addDoubleProperty(
                "Drive setpoint (mps)",
                () -> {
                    return this.state.speedMetersPerSecond;
                },
                (speed) -> {
                    this.state.speedMetersPerSecond = speed;
                });

        builder.addDoubleProperty(
                "Turn setpoint (degrees)",
                this.state.angle::getDegrees,
                (angle) -> {
                    this.state.angle = Rotation2d.fromDegrees(angle);
                });
    }

    /**
     * Set drive motor P constant.
     *
     * @param kP
     */
    public void setDriveP(double kP) {
        // Only make the change if the value is different.
        if (kP == this.driveMotor.getPIDController().getP()) {
            return;
        }

        REVLibError error = this.driveMotor.getPIDController().setP(kP);
        if (error != REVLibError.kOk) {
            System.out.println(
                    "Error setting drive motor " + this.driveMotor.getDeviceId() + " kP:" + error);
        }
    }

    /**
     * Set drive motor I constant.
     *
     * @param kI
     */
    public void setDriveI(double kI) {
        // Only make the change if the value is different.
        if (kI == this.driveMotor.getPIDController().getI()) {
            return;
        }

        REVLibError error = this.driveMotor.getPIDController().setI(kI);
        if (error != REVLibError.kOk) {
            System.out.println(
                    "Error setting drive motor " + this.driveMotor.getDeviceId() + " kI:" + error);
        }
    }

    /**
     * Set drive motor D constant.
     *
     * @param kD
     */
    public void setDriveD(double kD) {
        // Only make the change if the value is different.
        if (kD == this.driveMotor.getPIDController().getD()) {
            return;
        }

        REVLibError error = this.driveMotor.getPIDController().setD(kD);
        if (error != REVLibError.kOk) {
            System.out.println(
                    "Error setting drive motor " + this.driveMotor.getDeviceId() + " kD:" + error);
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
        this.setDriveP(kP);
        this.setDriveI(kI);
        this.setDriveD(kD);
    }

    /** Set turn motor P constant. */
    public void setTurnP(double kP) {
        this.turnController.setP(kP);
    }

    /** Set turn motor I constant. */
    public void setTurnI(double kI) {
        this.turnController.setI(kI);
    }

    /** Set turn motor D constant. */
    public void setTurnD(double kD) {
        this.turnController.setD(kD);
    }

    /**
     * Set turn motor PID constants.
     *
     * @param kP
     * @param kI
     * @param kD
     */
    public void setTurnPID(double kP, double kI, double kD) {
        this.setTurnP(kP);
        this.setTurnI(kI);
        this.setTurnD(kD);
    }
}
