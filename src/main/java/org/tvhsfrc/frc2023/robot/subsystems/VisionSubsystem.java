package org.tvhsfrc.frc2023.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.IOException;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.tvhsfrc.frc2023.robot.Constants;

/**
 * Vision subsystem, uses PhotonVision pipeline to estimate robot pose
 *
 * <p>https://docs.photonvision.org/en/latest/docs/programming/photonlib/robot-pose-estimator.html
 */
public class VisionSubsystem extends SubsystemBase {
    private PhotonCamera cam;

    private AprilTagFieldLayout fieldLayout;
    private String layoutName;

    private Optional<PhotonPoseEstimator> poseEstimator = Optional.empty();

    /** The last pose estimation that was successful. This is what should be used for driving. */
    private Optional<EstimatedRobotPose> poseEstimation = Optional.empty();

    public VisionSubsystem() {
        setName("Vision");

        // connect();

        // initShuffleboard();
    }

    /** Initialize camera and pose estimator */
    private void connect() {
        cam = new PhotonCamera(Constants.Vision.CAMERA_NAME);

        fieldLayout = loadOfficalLayout();
        if (fieldLayout == null) {
            // Try to load the official layout, again
            fieldLayout = loadOfficalLayout();
        }

        if (fieldLayout != null) {
            poseEstimator =
                    Optional.of(
                            new PhotonPoseEstimator(
                                    fieldLayout,
                                    PoseStrategy.LOWEST_AMBIGUITY,
                                    cam,
                                    Constants.Vision.CAMERA_TRANSFORM));
        }
    }

    /** Disconnect from camera and pose estimator */
    private void disconnect() {
        if (cam != null) {
            cam.close();
            cam = null;
        }
        poseEstimator = Optional.empty();
    }

    /**
     * This _should_ never fail. But, this has been written to not crash the robot if it does.
     *
     * @return AprilTagFieldLayout or null
     */
    private AprilTagFieldLayout loadOfficalLayout() {
        try {
            AprilTagFieldLayout layout =
                    AprilTagFieldLayout.loadFromResource(Constants.Vision.FIELD_LAYOUT);
            if (poseEstimator.isPresent()) {
                poseEstimator.get().setFieldTags(layout);
            }
            this.layoutName = "Official";
            return layout;
        } catch (IOException e) {
            System.err.println("Failed to offical april tag field layout: " + e.getMessage());
            return null;
        }
    }

    private AprilTagFieldLayout loadCustomLayout(String name) {
        try {
            AprilTagFieldLayout layout = new AprilTagFieldLayout(name);
            if (poseEstimator.isPresent()) {
                poseEstimator.get().setFieldTags(layout);
            }
            this.layoutName = name;
            return layout;
        } catch (IOException e) {
            System.err.println("Failed to load custom april tag field layout: " + e.getMessage());
            return null;
        }
    }

    /** Gets the successful pose estimation. This method is quick. */
    public Optional<EstimatedRobotPose> getEstimatedPose() {
        return poseEstimation;
    }

    @Override
    public void periodic() {
        /*if (poseEstimator.isPresent()) {
            Optional<EstimatedRobotPose> pose = poseEstimator.get().update();
            if (pose.isPresent()) {
                poseEstimation = pose;
                cam.setLED(VisionLEDMode.kOn);
            } else {
                cam.setLED(VisionLEDMode.kBlink);
            }
        } else {
            cam.setLED(VisionLEDMode.kOff);
        }*/
    }

    // ----------------
    // Command Builders
    // ----------------

    /**
     * This command will update the field layout to be the one selected by Shuffleboard.
     *
     * @return the command
     */
    public CommandBase cSetLayout() {
        return runOnce(
                () -> {
                    String name = layoutEntry.getString("Official");
                    if (name.equals("Official")) {
                        fieldLayout = loadOfficalLayout();
                    } else {
                        fieldLayout = loadCustomLayout(name);
                    }
                });
    }

    /**
     * Creates a command will connect to the camera and start the pose estimator.
     *
     * @return the command
     */
    public CommandBase cConnect() {
        return runOnce(this::connect);
    }

    /**
     * Creates a command will disconnect from the camera and stop the pose estimator.
     *
     * @return the command
     */
    public CommandBase cDisconnect() {
        return runOnce(this::disconnect);
    }

    // --------------
    // SmartDashboard
    // --------------
    private GenericEntry layoutEntry;

    public void initShuffleboard() {
        // Shuffleboard
        /*ShuffleboardTab tab = Shuffleboard.getTab("Vision");

        tab.addString("Camera Name", () -> cam.getName()).withWidget(BuiltInWidgets.kTextView);
        tab.addBoolean("Camera Connected", () -> cam.isConnected())
                .withWidget(BuiltInWidgets.kBooleanBox);
        tab.addBoolean("Has Target", () -> cam.getLatestResult().hasTargets())
                .withWidget(BuiltInWidgets.kBooleanBox);
        tab.addNumber("Target Count", () -> cam.getLatestResult().getTargets().size())
                .withWidget(BuiltInWidgets.kTextView);

        tab.addNumber("X", () -> poseEstimation.map(p -> p.estimatedPose.getX()).orElse(0.0));
        tab.addNumber("Y", () -> poseEstimation.map(p -> p.estimatedPose.getY()).orElse(0.0));
        tab.addNumber("Z", () -> poseEstimation.map(p -> p.estimatedPose.getZ()).orElse(0.0));
        tab.addNumber(
                "Yaw",
                () ->
                        poseEstimation
                                .map(p -> p.estimatedPose.getRotation().toRotation2d().getDegrees())
                                .orElse(0.0));

        layoutEntry = tab.add("Layout", layoutName).getEntry();

        tab.add("Set Layout", this.cSetLayout());*/
    }
}
