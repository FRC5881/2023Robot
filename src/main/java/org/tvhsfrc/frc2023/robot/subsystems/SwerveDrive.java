package org.tvhsfrc.frc2023.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import org.tvhsfrc.frc2023.robot.Constants;

public class SwerveDrive {
    // SwerveModule subsystems
    private final SwerveModule frontLeftModule =
            new SwerveModule(Constants.FRONT_LEFT_SWERVE_MODULE);
    private final SwerveModule frontRightModule =
            new SwerveModule(Constants.FRONT_RIGHT_SWERVE_MODULE);
    private final SwerveModule backLeftModule = new SwerveModule(Constants.BACK_LEFT_SWERVE_MODULE);
    private final SwerveModule backRightModule =
            new SwerveModule(Constants.BACK_RIGHT_SWERVE_MODULE);

    public SwerveDrive() {
        resetModules();

        initShuffleboard();
    }

    /** Reset module positions */
    protected void resetModules() {
        frontLeftModule.resetEncoder();
        frontRightModule.resetEncoder();
        backLeftModule.resetEncoder();
        backRightModule.resetEncoder();
    }

    /** Odometry (angle and distance) */
    protected SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            backLeftModule.getPosition(),
            backRightModule.getPosition(),
        };
    }

    /** Odometry (angle and speed) */
    protected SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            frontLeftModule.getState(),
            frontRightModule.getState(),
            backLeftModule.getState(),
            backRightModule.getState(),
        };
    }

    /** Odometry (ChassisSpeeds) */
    protected ChassisSpeeds getChassisSpeeds() {
        return Constants.Swerve.KINEMATICS.toChassisSpeeds(getModuleStates());
    }

    /**
     * Drive the robot.
     *
     * @param chassisSpeeds WPILIB ChassisSpeeds object.
     */
    protected void drive(ChassisSpeeds chassisSpeeds) {
        // Kinematics
        SwerveModuleState[] moduleStates =
                Constants.Swerve.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
                moduleStates, Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND);

        frontLeftModule.drive(moduleStates[0]);
        frontRightModule.drive(moduleStates[1]);
        backLeftModule.drive(moduleStates[2]);
        backRightModule.drive(moduleStates[3]);
    }

    /** Stops all modules */
    protected void stop() {
        frontLeftModule.stop();
        frontRightModule.stop();
        backLeftModule.stop();
        backRightModule.stop();
    }

    /// ------------
    /// SHUFFLEBOARD
    /// ------------

    // Drivetrain shuffleboard tab
    private final ShuffleboardTab tab = Shuffleboard.getTab("Swerve Drive");

    private GenericEntry driveKp;
    private GenericEntry driveKi;
    private GenericEntry driveKd;
    private GenericEntry turnKp;
    private GenericEntry turnKi;
    private GenericEntry turnKd;

    public void initShuffleboard() {
        // Module widgets
        final int width = 3;
        final int height = 2;

        tab.add("Back Left", backLeftModule).withSize(width, height).withPosition(0, 0);
        tab.add("Back Right", backRightModule).withSize(width, height).withPosition(width, 0);
        tab.add("Front Left", frontLeftModule).withSize(width, height).withPosition(0, height);
        tab.add("Front Right", frontRightModule)
                .withSize(width, height)
                .withPosition(width, height);

        ShuffleboardLayout turnPIDList =
                tab.getLayout("Turn PID", BuiltInLayouts.kList).withPosition(7, 0).withSize(1, 2);
        ShuffleboardLayout drivePIDList =
                tab.getLayout("Drive PID", BuiltInLayouts.kList).withPosition(6, 0).withSize(1, 2);

        driveKp =
                drivePIDList
                        .addPersistent("kP", 0.0076116)
                        .withWidget(BuiltInWidgets.kTextView)
                        .getEntry("double");

        driveKi =
                drivePIDList
                        .addPersistent("kI", 0)
                        .withWidget(BuiltInWidgets.kTextView)
                        .getEntry("double");

        driveKd =
                drivePIDList
                        .addPersistent("kD", 0)
                        .withWidget(BuiltInWidgets.kTextView)
                        .getEntry("double");

        turnKp =
                turnPIDList
                        .addPersistent("kP", 0.005)
                        .withWidget(BuiltInWidgets.kTextView)
                        .getEntry("double");

        turnKi =
                turnPIDList
                        .addPersistent("kI", 0)
                        .withWidget(BuiltInWidgets.kTextView)
                        .getEntry("double");

        turnKd =
                turnPIDList
                        .addPersistent("kD", 0.0001)
                        .withWidget(BuiltInWidgets.kTextView)
                        .getEntry("double");
    }

    protected void updatePID() {
        frontLeftModule.setDrivePID(
                driveKp.getDouble(0.0), driveKi.getDouble(0.0), driveKd.getDouble(0.0));
        frontLeftModule.setTurnPID(
                turnKp.getDouble(0.0), turnKi.getDouble(0.0), turnKd.getDouble(0.0));
        frontRightModule.setDrivePID(
                driveKp.getDouble(0.0), driveKi.getDouble(0.0), driveKd.getDouble(0.0));
        frontRightModule.setTurnPID(
                turnKp.getDouble(0.0), turnKi.getDouble(0.0), turnKd.getDouble(0.0));
        backLeftModule.setDrivePID(
                driveKp.getDouble(0.0), driveKi.getDouble(0.0), driveKd.getDouble(0.0));
        backLeftModule.setTurnPID(
                turnKp.getDouble(0.0), turnKi.getDouble(0.0), turnKd.getDouble(0.0));
        backRightModule.setDrivePID(
                driveKp.getDouble(0.0), driveKi.getDouble(0.0), driveKd.getDouble(0.0));
        backRightModule.setTurnPID(
                turnKp.getDouble(0.0), turnKi.getDouble(0.0), turnKd.getDouble(0.0));
    }
}
