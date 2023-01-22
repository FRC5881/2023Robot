package org.tvhsfrc.frc2023.robot.subsystems;

import org.tvhsfrc.frc2023.robot.Constants;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrainSubsystem extends SubsystemBase {
    // Locations for the swerve drive modules relative to the robot center. In
    // meters.
    private final Translation2d m_frontLeftLocation = new Translation2d(
            Constants.Swerve.DRIVETRAIN_TRACKWIDTH_METERS / 2, Constants.Swerve.DRIVETRAIN_WHEELBASE_METERS / 2);
    private final Translation2d m_frontRightLocation = new Translation2d(
            Constants.Swerve.DRIVETRAIN_TRACKWIDTH_METERS / 2, -Constants.Swerve.DRIVETRAIN_WHEELBASE_METERS / 2);
    private final Translation2d m_backLeftLocation = new Translation2d(
            -Constants.Swerve.DRIVETRAIN_TRACKWIDTH_METERS / 2, Constants.Swerve.DRIVETRAIN_WHEELBASE_METERS / 2);
    private final Translation2d m_backRightLocation = new Translation2d(
            -Constants.Swerve.DRIVETRAIN_TRACKWIDTH_METERS / 2, -Constants.Swerve.DRIVETRAIN_WHEELBASE_METERS / 2);

    /**
     * The kinematics object for the swerve drive. This is used to convert a desired
     * "chassis speed" into individual module speeds.
     */
    private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    /**
     * Current desired cYeah go ahedhassis speed.
     */
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    /**
     * Odometry using encoders, gyro, and vision.
     */
    private SwerveDrivePoseEstimator poseEstimator;

    // ----------------- //
    // Swerve Modules and NavX //
    // ----------------- //
    private final SwerveModule frontLeftModule = new SwerveModule(Constants.FRONT_LEFT_SWERVE_MODULE);
    private final SwerveModule frontRightModule = new SwerveModule(Constants.FRONT_RIGHT_SWERVE_MODULE);
    private final SwerveModule backLeftModule = new SwerveModule(Constants.BACK_LEFT_SWERVE_MODULE);
    private final SwerveModule backRightModule = new SwerveModule(Constants.BACK_RIGHT_SWERVE_MODULE);

    private final AHRS navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

    @Override
    public void periodic() {
        // Calculate the desired module speeds from the desired chassis speeds.
        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(this.chassisSpeeds);

        // Set the module speeds.
        frontLeftModule.setModuleState(moduleStates[0]);
        frontRightModule.setModuleState(moduleStates[1]);
        backLeftModule.setModuleState(moduleStates[2]);
        backRightModule.setModuleState(moduleStates[3]);

        // Update the odometry objects.
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[] {
                frontLeftModule.getModulePosition(),
                frontRightModule.getModulePosition(),
                backLeftModule.getModulePosition(),
                backRightModule.getModulePosition()
        };

        poseEstimator.update(navx.getRotation2d(), modulePositions);
    }

    /**
     * Sets the desired field oriented chassis speed. The actual module speeds are
     * calculated in the periodic method.
     * 
     * @param chassisSpeeds WPILIB ChassisSpeeds object.
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
    }
}
