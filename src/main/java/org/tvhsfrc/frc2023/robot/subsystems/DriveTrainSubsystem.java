package org.tvhsfrc.frc2023.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.IOException;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.tvhsfrc.frc2023.robot.Constants;

public class DriveTrainSubsystem extends SubsystemBase {
    /// NavX connected over MXP
    private final AHRS navx = new AHRS(SPI.Port.kMXP);

    // SwerveModule subsystems
    private final SwerveModule frontLeftModule =
            new SwerveModule(Constants.FRONT_LEFT_SWERVE_MODULE);
    private final SwerveModule frontRightModule =
            new SwerveModule(Constants.FRONT_RIGHT_SWERVE_MODULE);
    private final SwerveModule backLeftModule = new SwerveModule(Constants.BACK_LEFT_SWERVE_MODULE);
    private final SwerveModule backRightModule =
            new SwerveModule(Constants.BACK_RIGHT_SWERVE_MODULE);

    /** Desired chassis speed. */
    private ChassisSpeeds chassisSpeeds;

    /** Odometry produced from the swerve drive */
    private final SwerveDrivePoseEstimator poseEstimator;

    // ------
    // Vision
    // ------
    private PhotonCamera cam;

    private PhotonPoseEstimator photonPoseEstimator;

    /**
     * Must be chosen on boot.
     *
     * <p>If true the Camera connection is initialized, vision data is collected, and data is
     * incorpated into the Pose Estimator
     *
     * <p>If false then all of that is skipped.
     *
     * <p>During setup if there is a problem this will be set false and you should not trust any
     * vision objects. They may be null.
     */
    private boolean useVision = true;

    /** PoseEstimator */
    public DriveTrainSubsystem() {
        initShuffleboard();

        // Zero the gyro after it finishes calibrating
        new Thread(
                () -> {
                    try {
                        Thread.sleep(1000);
                        _zeroHeading();
                    } catch (Exception e) {
                    }
                });

        // PoseEstimator starts at 0,0 angle 0
        poseEstimator =
                new SwerveDrivePoseEstimator(
                        Constants.Swerve.KINEMATICS,
                        new Rotation2d(),
                        getModulePositions(),
                        new Pose2d());

        if (useVision) {
            try {
                // https://docs.photonvision.org/en/latest/docs/programming/photonlib/robot-pose-estimator.html
                AprilTagFieldLayout aprilTagFieldLayout =
                        AprilTagFieldLayout.loadFromResource(Constants.Vision.FIELD_LAYOUT);
                cam = new PhotonCamera(Constants.Vision.CAMERA_NAME);

                photonPoseEstimator =
                        new PhotonPoseEstimator(
                                aprilTagFieldLayout,
                                PoseStrategy.LOWEST_AMBIGUITY,
                                cam,
                                Constants.Vision.CAMERA_TRANSFORM);
            } catch (IOException e) {
                System.err.println("Failed to load photo vision field layout: " + e.getMessage());
                useVision = false;
            }
        }
    }

    @Override
    public void periodic() {
        ticks += 1;

        // Update using odometry
        poseEstimator.update(getRotation2d(), getModulePositions());

        // Update using vision measurement
        if (useVision) {
            getVisionPose()
                    .ifPresent(
                            (measurement) -> {
                                poseEstimator.addVisionMeasurement(
                                        measurement.estimatedPose.toPose2d(),
                                        measurement.timestampSeconds);
                            });
        }

        updateShuffleboard();
    }

    private void _zeroHeading() {
        navx.reset();
    }

    private void _calibrate() {
        navx.calibrate();
    }

    /** Returns the current gyro heading. In degrees between 0 and 360. */
    public double getHeading() {
        return Math.IEEEremainder(navx.getAngle(), 360);
    }

    /** Returns the current gyro heading. As a Rotation2d. */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    /**
     * Sets the desired field oriented chassis speed. The actual module speeds are calculated in the
     * periodic method.
     *
     * @param chassisSpeeds WPILIB ChassisSpeeds object.
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;

        // Kinematics
        SwerveModuleState[] moduleStates =
                Constants.Swerve.KINEMATICS.toSwerveModuleStates(this.chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(
                moduleStates, Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND);

        frontLeftModule.drive(moduleStates[0]);
        frontRightModule.drive(moduleStates[1]);
        backLeftModule.drive(moduleStates[2]);
        backRightModule.drive(moduleStates[3]);
    }

    /** Stops all modules */
    public void _stop() {
        frontLeftModule.stop();
        frontRightModule.stop();
        backLeftModule.stop();
        backRightModule.stop();
    }

    /** Reset module positions */
    public void resetModules() {
        frontLeftModule.resetEncoder();
        frontRightModule.resetEncoder();
        backLeftModule.resetEncoder();
        backRightModule.resetEncoder();
    }

    /** Get current module positions */
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            backLeftModule.getPosition(),
            backRightModule.getPosition(),
        };
    }

    /**
     * setPose will manually reset the PoseEstimator with the given pose.
     *
     * <p>This might be quickly overwritten if PhotonVision detects a tag.
     *
     * <p>This is most helpful for initializing autonmous commands.
     */
    public void setPose(Pose2d pose) {
        resetModules();
        poseEstimator.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    /** Our best possible pose estimate. */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /** Must only be called if "useVision" is true */
    public Optional<EstimatedRobotPose> getVisionPose() {
        return photonPoseEstimator.update();
    }

    // public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    //     photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    //     return photonPoseEstimator.update();
    // }

    /// ----------------
    /// COMMAND BUILDERS
    /// ----------------

    /**
     * Creates a new command that will follow a trajectory. This command does <b>not</b> end will a
     * call to `stop`. If you want the robot to come to a stop after the completion of this
     * trajectory you should sequence it with `this::halt()`
     *
     * @param path A trajectory created by PathPlanner
     * @param isFirstPath Initialize odometry for the first path during auto
     */
    public Command followTrajectory(PathPlannerTrajectory path, boolean isFirstPath) {
        // TODO: Tune these pid controllers
        PIDController xController = new PIDController(0, 0, 0);
        PIDController yController = new PIDController(0, 0, 0);
        PIDController thetaController = new PIDController(0, 0, 0);

        return Commands.sequence(
                this.runOnce(
                        () -> {
                            if (isFirstPath) {
                                this.setPose(path.getInitialHolonomicPose());
                            }
                        }),
                new PPSwerveControllerCommand(
                        path,
                        this::getPose,
                        xController,
                        yController,
                        thetaController,
                        this::drive,
                        this));
    }

    /** Creates a command to stop the motors */
    public InstantCommand stop() {
        return new InstantCommand(this::_stop, this);
    }

    /** Creates a command to zero the gyro */
    public InstantCommand zeroHeading() {
        return new InstantCommand(this::_zeroHeading, this);
    }

    public Command calibrate() {
        if (!this.navx.isCalibrating()) {
            return new InstantCommand(this::_calibrate, this);
        } else {
            return Commands.none();
        }
    }

    /**
     * TODO:
     *
     * <p>Creates a command that balances the robot on the charging station
     */
    public Command balance() {
        return Commands.none();
    }

    /// ------------
    /// SHUFFLEBOARD
    /// ------------

    // Drivetrain shuffleboard tab
    private final ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    // Many DoubleEntries are used to allow for easy access to PID values.
    private final GenericEntry driveKp =
            tab.addPersistent("Drive kP", 0)
                    .withWidget(BuiltInWidgets.kTextView)
                    .getEntry("double");

    private final GenericEntry driveKi =
            tab.addPersistent("Drive kI", 0)
                    .withWidget(BuiltInWidgets.kTextView)
                    .getEntry("double");

    private final GenericEntry driveKd =
            tab.addPersistent("Drive kD", 0)
                    .withWidget(BuiltInWidgets.kTextView)
                    .getEntry("double");

    private final GenericEntry turnKp =
            tab.addPersistent("Turn kP", 0).withWidget(BuiltInWidgets.kTextView).getEntry("double");

    private final GenericEntry turnKi =
            tab.addPersistent("Turn kI", 0).withWidget(BuiltInWidgets.kTextView).getEntry("double");

    private final GenericEntry turnKd =
            tab.addPersistent("Turn kD", 0).withWidget(BuiltInWidgets.kTextView).getEntry("double");

    private int ticks = 0;

    private void initShuffleboard() {
        tab.add(navx);

        tab.add(frontLeftModule);
        tab.add(frontRightModule);
        tab.add(backLeftModule);
        tab.add(backRightModule);
    }

    // Send and receive values from shuffleboard.
    private void updateShuffleboard() {
        if (ticks % 40 == 0) {
            frontLeftModule.setDrivePID(
                    driveKp.getDouble(0.0), driveKi.getDouble(0.0), driveKd.getDouble(0.0));
            frontLeftModule.setTurnPID(
                    turnKp.getDouble(0.0), turnKi.getDouble(0.0), turnKd.getDouble(0.0));
        }

        if (ticks % 40 == 10) {
            frontRightModule.setDrivePID(
                    driveKp.getDouble(0.0), driveKi.getDouble(0.0), driveKd.getDouble(0.0));
            frontRightModule.setTurnPID(
                    turnKp.getDouble(0.0), turnKi.getDouble(0.0), turnKd.getDouble(0.0));
        }

        if (ticks % 40 == 20) {
            backLeftModule.setDrivePID(
                    driveKp.getDouble(0.0), driveKi.getDouble(0.0), driveKd.getDouble(0.0));
            backLeftModule.setTurnPID(
                    turnKp.getDouble(0.0), turnKi.getDouble(0.0), turnKd.getDouble(0.0));
        }

        if (ticks % 40 == 30) {
            backRightModule.setDrivePID(
                    driveKp.getDouble(0.0), driveKi.getDouble(0.0), driveKd.getDouble(0.0));
            backRightModule.setTurnPID(
                    turnKp.getDouble(0.0), turnKi.getDouble(0.0), turnKd.getDouble(0.0));
        }
    }
}
