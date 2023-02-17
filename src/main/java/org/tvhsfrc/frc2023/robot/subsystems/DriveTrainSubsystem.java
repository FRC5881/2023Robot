package org.tvhsfrc.frc2023.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.tvhsfrc.frc2023.robot.Constants;
import org.tvhsfrc.frc2023.robot.Utils.GyroProxy;
import org.tvhsfrc.frc2023.robot.Utils.GyroProxy.Axis;

public class DriveTrainSubsystem extends SubsystemBase {
    private Vision vision;
    private SwerveDrive drive;

    /** PoseEstimator that incorpates vision and odometry */
    private final SwerveDrivePoseEstimator poseEstimator;

    /**
     * If true, the poseEstimator will incorporate vision and odometry measurements. If false, the
     * poseEstimator will only incorporate odometry measurements.
     *
     * <p>This can be used to prevent unwanted jerkiness when starting a match.
     */
    private boolean useVision = false;

    /**
     * navx
     *
     * <p>TODO: Update to new navx
     */
    private final AHRS navx = new AHRS(SPI.Port.kMXP);

    public DriveTrainSubsystem() {
        setName("Drivetrain");

        if (useVision) {
            vision = new Vision();
        }
        drive = new SwerveDrive();

        poseEstimator =
                new SwerveDrivePoseEstimator(
                        Constants.Swerve.KINEMATICS,
                        new Rotation2d(),
                        drive.getModulePositions(),
                        new Pose2d());

        initShuffleboard();

        // Load the PID values once at the start of the subsystem
        drive.updatePID();
    }

    @Override
    public void periodic() {
        // Update using odometry
        poseEstimator.update(getRotation2d(), drive.getModulePositions());

        // Update using vision
        if (useVision && vision.getEstimatedPose().isPresent()) {
            poseEstimator.addVisionMeasurement(
                    vision.getEstimatedPose().get().estimatedPose.toPose2d(),
                    vision.getEstimatedPose().get().timestampSeconds);
        }

        field.setRobotPose(poseEstimator.getEstimatedPosition());
    }

    // --------------
    // Pose Estimator
    // --------------

    /**
     * Returns our best estimate of the robot's position on the field.
     *
     * @return Pose2d of the robot
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Sets our initial guess of robot's position on the field. This could be quickly overridden by
     * vision, therefore it is recommended to only use this method at the start of the match.
     *
     * @param pose
     */
    public void setPose(Pose2d pose) {
        drive.resetModules();
        poseEstimator.resetPosition(getRotation2d(), drive.getModulePositions(), pose);
    }

    // ------
    // Swerve
    // ------

    /**
     * orientedDrive allows the robot to drive in a field relative way.
     *
     * @param xSpeed positive x is away from alliance wall
     * @param ySpeed positive y is left when standing behind the alliance wall
     * @param omega is radians per second to rotate the robot
     */
    public void orientedDrive(double xSpeed, double ySpeed, double omega) {
        drive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omega, getRotation2d()));
    }

    /**
     * drive commands the robot to drive in a robot relative way.
     *
     * @param speeds ChassisSpeeds object to drive the robot. X is forward/backward, Y is
     *     left/right, omega is radians per second
     */
    public void drive(ChassisSpeeds speeds) {
        drive.drive(speeds);
    }

    /** Stops the robot. */
    public void stop() {
        drive.stop();
    }

    // ----
    // GYRO
    // ----
    private void zeroHeading() {
        navx.reset();
    }

    private void calibrateGyro() {
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
     * Returns the Rotation3D of the gyro. This can be used to balance the robot on the charging
     * station.
     */
    public Rotation3d getRotation3d() {
        return new Rotation3d(
                new Quaternion(
                        navx.getQuaternionW(),
                        navx.getQuaternionX(),
                        navx.getQuaternionY(),
                        navx.getQuaternionZ()));
    }

    /// ----------------
    /// COMMAND BUILDERS
    /// ----------------

    /**
     * Creates a new command that will follow a trajectory. This command does <b>not</b> end with a
     * call to {@link #stop()}. If you want the robot to come to a stop after the completion of this
     * trajectory you should sequence with {@link #cStop()} or ensure that the last point in the
     * trajectory is a point that will cause the robot to stop.
     *
     * @param path A trajectory created by PathPlanner
     * @return A command that will follow the trajectory
     */
    public CommandBase cFollowTrajectory(PathPlannerTrajectory path) {
        // TODO: Tune these pid controllers
        PIDController xController = new PIDController(1, 0, 0);
        PIDController yController = new PIDController(1, 0, 0);
        PIDController thetaController = new PIDController(1, 0, 0);

        return new PPSwerveControllerCommand(
                path,
                this::getPose,
                xController,
                yController,
                thetaController,
                this.drive::drive,
                this);
    }

    /** Creates a command to stop the motors */
    public InstantCommand cStop() {
        return new InstantCommand(this.drive::stop, this);
    }

    /** Creates a command to zero the gyro */
    public InstantCommand cZeroHeading() {
        return new InstantCommand(this::zeroHeading, this);
    }

    /**
     * Creates a command to calibrates the navx. The command will run until the navx is calibrated.
     *
     * <p>If the navx is already calibrating this command will wait until the navx is calibrated.
     */
    public CommandBase cCalibrateGyro() {
        CommandBase cWait =
                Commands.waitUntil(
                        () -> {
                            return !navx.isCalibrating();
                        });

        if (!navx.isCalibrating()) {
            return runOnce(this::calibrateGyro).andThen(cWait);
        } else {
            return cWait;
        }
    }

    /**
     * TODO:
     *
     * <p>Creates a command that balances the robot on the charging station
     */
    public CommandBase cBalance() {
        return Commands.none();
    }

    /**
     * This command will update PID values from Shuffleboard. This comamnd will almost certianly
     * cause a loop-overrun
     */
    private CommandBase cUpdatePID() {
        return new InstantCommand(this.drive::updatePID, this);
    }

    /**
     * This command will update set the robot's pose estimate to the given pose.
     *
     * <p>It is recommended to only use this command at the start of the match, as it will be
     * quickly overridden by vision.
     */
    public InstantCommand cSetPose(Pose2d pose) {
        return new InstantCommand(() -> setPose(pose), this);
    }

    /** Creates a command that enables vision measurements. */
    public InstantCommand cEnableVision() {
        return new InstantCommand(() -> useVision = true, this);
    }

    /** Creates a command that disables vision measurements. */
    public InstantCommand cDisableVision() {
        return new InstantCommand(() -> useVision = false, this);
    }

    private Field2d field = new Field2d();

    /// ------------
    /// SHUFFLEBOARD
    /// ------------
    private void initShuffleboard() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        // Add subsystem
        tab.add(this);

        tab.add("Field", field);

        // 3 axis Gyro
        ShuffleboardLayout gyro = tab.getLayout("Gyro", BuiltInLayouts.kGrid).withSize(5, 2);
        gyro.add("Yaw", new GyroProxy(navx, Axis.YAW)).withPosition(0, 0);
        gyro.add("Pitch", new GyroProxy(navx, Axis.PITCH)).withPosition(1, 0);
        gyro.add("Roll", new GyroProxy(navx, Axis.ROLL)).withPosition(2, 0);

        // Command list
        ShuffleboardLayout commands = tab.getLayout("Commands", BuiltInLayouts.kList);
        commands.add("Stop", cStop());
        commands.add("Calibrate", cCalibrateGyro());
        commands.add("Zero Heading", cZeroHeading());
        commands.add("Reset Pose", cSetPose(new Pose2d(new Translation2d(4, 4), new Rotation2d())));
        commands.add("Enable Vision", cEnableVision());
        commands.add("Disable Vision", cDisableVision());

        Shuffleboard.getTab("Swerve Drive").add("Update PID", cUpdatePID()).withPosition(6, 2);
    }
}
