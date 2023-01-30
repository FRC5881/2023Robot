package org.tvhsfrc.frc2023.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.tvhsfrc.frc2023.robot.Constants;

public class DriveTrainSubsystem extends SubsystemBase {
    // Locations for the swerve drive modules relative to the robot center (meters).
    private final Translation2d m_frontLeftLocation =
            new Translation2d(
                    Constants.Swerve.DRIVETRAIN_TRACKWIDTH_METERS / 2,
                    Constants.Swerve.DRIVETRAIN_WHEELBASE_METERS / 2);
    private final Translation2d m_frontRightLocation =
            new Translation2d(
                    Constants.Swerve.DRIVETRAIN_TRACKWIDTH_METERS / 2,
                    -Constants.Swerve.DRIVETRAIN_WHEELBASE_METERS / 2);
    private final Translation2d m_backLeftLocation =
            new Translation2d(
                    -Constants.Swerve.DRIVETRAIN_TRACKWIDTH_METERS / 2,
                    Constants.Swerve.DRIVETRAIN_WHEELBASE_METERS / 2);
    private final Translation2d m_backRightLocation =
            new Translation2d(
                    -Constants.Swerve.DRIVETRAIN_TRACKWIDTH_METERS / 2,
                    -Constants.Swerve.DRIVETRAIN_WHEELBASE_METERS / 2);

    /**
     * The kinematics object for the swerve drive. This is used to convert a desired "chassis speed"
     * into individual module speeds.
     */
    private SwerveDriveKinematics m_kinematics =
            new SwerveDriveKinematics(
                    m_frontLeftLocation,
                    m_frontRightLocation,
                    m_backLeftLocation,
                    m_backRightLocation);

    /** Current desired chassis speed. */
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    // ----------------- //
    // Swerve Modules and NavX //
    // ----------------- //
    private final SwerveModule frontLeftModule =
            new SwerveModule(Constants.FRONT_LEFT_SWERVE_MODULE);
    private final SwerveModule frontRightModule =
            new SwerveModule(Constants.FRONT_RIGHT_SWERVE_MODULE);
    private final SwerveModule backLeftModule = new SwerveModule(Constants.BACK_LEFT_SWERVE_MODULE);
    private final SwerveModule backRightModule =
            new SwerveModule(Constants.BACK_RIGHT_SWERVE_MODULE);

    // TODO: What does 200 mean and is it correct?
    private final AHRS navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

    /** Odometry using encoders, gyro, and vision. */
    private SwerveDrivePoseEstimator poseEstimator =
            new SwerveDrivePoseEstimator(
                    m_kinematics,
                    navx.getRotation2d(),
                    new SwerveModulePosition[] {
                        frontLeftModule.getPosition(),
                        frontRightModule.getPosition(),
                        backLeftModule.getPosition(),
                        backRightModule.getPosition()
                    },
                    new Pose2d());

    public DriveTrainSubsystem() {
        // TODO: Does this shuffleboard layout look good?
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        tab.add(navx);

        tab.add(frontLeftModule);
        tab.add(frontRightModule);
        tab.add(backLeftModule);
        tab.add(backRightModule);
    }

    @Override
    public void periodic() {
        // Kinematics
        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(this.chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
                moduleStates, Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND);

        frontLeftModule.setModuleState(moduleStates[0]);
        frontRightModule.setModuleState(moduleStates[1]);
        backLeftModule.setModuleState(moduleStates[2]);
        backRightModule.setModuleState(moduleStates[3]);

        // Odometry
        poseEstimator.update(navx.getRotation2d(), getModulePositions());

        // TODO: Improve Odometry w/ Vision
    }

    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            backLeftModule.getPosition(),
            backRightModule.getPosition()
        };
    }

    /**
     * Sets the desired field oriented chassis speed. The actual module speeds are calculated in the
     * periodic method.
     *
     * @param chassisSpeeds WPILIB ChassisSpeeds object.
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
    }

    /**
     * Manually set the current field position. Best used for starting autonmous routines
     *
     * <p>Note: This has the effect of offseting the gyro angle being fed into the PoseEstimator.
     * The angle the PoseEstimator returns will be offset by the difference of what the gyro reads
     * and the angle to the `pose` when you call this function.
     *
     * <p>You should avoid using the gyro angle and the PoseEstimator angle simulatnously, unless if
     * you can assert that the gyro and the pose have matching angles.
     *
     * @param pose
     */
    public void setPose(Pose2d pose) {
        this.poseEstimator.resetPosition(this.navx.getRotation2d(), getModulePositions(), pose);
    }
}
