package org.tvhsfrc.frc2023.robot.subsystems;

import org.tvhsfrc.frc2023.robot.Constants;
import org.tvhsfrc.frc2023.robot.Constants.SwerveModuleConstants;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Models a single swerve module.
 * 
 * This class is responsible for controlling individual swerve modules by
 * passing in wpilib SwerveModuleState objects.
 */
public class SwerveModule extends SubsystemBase {
    // Two RevNeo motors per module.
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    // Additional absolute encoder for module heading.
    private final CANCoder turnEncoder;

    // PID controller for the turn motor. Using the turnEncoder as the feedback
    // sensor.
    //
    // Input range is -180 to 180 degrees where 180 and -180 are equivalent.
    // Output range is -1 to 1.
    private final PIDController turnController;

    /**
     * Creates a new SwerveModule.
     * 
     * After constructing a SwerveModule you should call setDrivePID and setTurnPID
     */
    public SwerveModule(SwerveModuleConstants moduleConfig) {
        this.turnEncoder = new CANCoder(moduleConfig.turnEncoderCANID);

        this.driveMotor = new CANSparkMax(moduleConfig.driveMotorCANID, CANSparkMax.MotorType.kBrushless);
        this.driveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        this.driveMotor.setSmartCurrentLimit(Constants.Swerve.DRIVE_CURRENT_LIMIT);
        this.driveMotor.getAbsoluteEncoder(Type.kDutyCycle)
                .setPositionConversionFactor(Constants.Swerve.DRIVE_CONVERSION_FACTOR);
        this.driveMotor.getAbsoluteEncoder(Type.kDutyCycle)
                .setVelocityConversionFactor(Constants.Swerve.DRIVE_CONVERSION_FACTOR);

        this.turnMotor = new CANSparkMax(moduleConfig.turnEncoderCANID, CANSparkMax.MotorType.kBrushless);
        this.turnMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        this.turnMotor.setSmartCurrentLimit(Constants.Swerve.TURN_CURRENT_LIMIT);

        // Turn encoder configuration.
        CANCoderConfiguration encoderConfig = new CANCoderConfiguration();
        encoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        encoderConfig.magnetOffsetDegrees = moduleConfig.turnEncoderOffset;
        encoderConfig.sensorDirection = true; // clockwise -> positive
        encoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;

        ErrorCode error = this.turnEncoder.configAllSettings(encoderConfig);
        if (error != ErrorCode.OK) {
            System.out.println("Error configuring CANCoder " + moduleConfig.turnEncoderCANID + ":" + error);
        }

        this.turnController = new PIDController(0, 0, 0);
        this.turnController.enableContinuousInput(-180, 180);
    }

    /**
     * Kinematics. Sets the targets module state.
     * 
     * @param state The desired state of the module.
     */
    public void setModuleState(SwerveModuleState state) {
        // Optimize the state to minimize the turn angle by flipping the drive
        // direction.
        // For example, if the angle is currently 180 degrees and the desired angle is
        // -180 degrees, we can flip the drive direction and set the angle to 0 degrees.
        Rotation2d currentAngle = Rotation2d.fromDegrees(this.turnEncoder.getAbsolutePosition());
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, currentAngle);

        // Set the drive motor to target the desired speed.
        this.driveMotor.getPIDController().setReference(optimizedState.speedMetersPerSecond, ControlType.kVelocity);

        // Set the turn pid controller to target the desired angle.
        this.turnController.setSetpoint(optimizedState.angle.getDegrees());
    }

    /**
     * Odometry. Gets the current module state. (angle and speed)
     */
    public SwerveModuleState getModuleState() {
        // Get the current angle from the turn encoder.
        Rotation2d currentAngle = Rotation2d.fromDegrees(this.turnEncoder.getAbsolutePosition());

        // Get the current speed from the drive encoder.
        double currentSpeed = this.driveMotor.getAbsoluteEncoder(Type.kDutyCycle).getVelocity();

        return new SwerveModuleState(currentSpeed, currentAngle);
    }

    /**
     * Odometry. Get the current module position. (angle and distance)
     * 
     * @return The current module position.
     */
    public SwerveModulePosition getModulePosition() {
        // Get the current angle from the turn encoder.
        Rotation2d currentAngle = Rotation2d.fromDegrees(this.turnEncoder.getAbsolutePosition());

        // Get the distance traveled from the drive encoder.
        double position = this.driveMotor.getEncoder().getPosition();

        return new SwerveModulePosition(position, currentAngle);
    }

    /**
     * Periodically called by the subsystem.
     */
    @Override
    public void periodic() {
        // Update the turn motor with the turn pid controller output.
        double setpoint = this.turnController.calculate(this.turnEncoder.getAbsolutePosition());
        setpoint = MathUtil.clamp(setpoint, -1.0, 1.0);
        this.turnMotor.set(setpoint);
    }

    /**
     * Set drive motor P constant.
     * 
     * @param kP
     */
    public void setDriveP(double kP) {
        REVLibError error = this.driveMotor.getPIDController().setP(kP);
        if (error != REVLibError.kOk) {
            System.out.println("Error setting drive motor " + this.driveMotor.getDeviceId() + " kP:" + error);
        }
    }

    /**
     * Set drive motor I constant.
     * 
     * @param kI
     */
    public void setDriveI(double kI) {
        REVLibError error = this.driveMotor.getPIDController().setI(kI);
        if (error != REVLibError.kOk) {
            System.out.println("Error setting drive motor " + this.driveMotor.getDeviceId() + " kI:" + error);
        }
    }

    /**
     * Set drive motor D constant.
     * 
     * @param kD
     */
    public void setDriveD(double kD) {
        REVLibError error = this.driveMotor.getPIDController().setD(kD);
        if (error != REVLibError.kOk) {
            System.out.println("Error setting drive motor " + this.driveMotor.getDeviceId() + " kD:" + error);
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

    /**
     * Set turn motor P constant.
     */
    public void setTurnP(double kP) {
        this.turnController.setP(kP);
    }

    /**
     * Set turn motor I constant.
     */
    public void setTurnI(double kI) {
        this.turnController.setI(kI);
    }

    /**
     * Set turn motor D constant.
     */
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
