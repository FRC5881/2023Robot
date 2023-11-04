package org.tvhsfrc.frc2023.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;

public class RotationUtil {
    private RotationUtil() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /** Clamps a rotation between a minimum and maximum rotation. */
    public static Rotation2d clamp(Rotation2d rotation, Rotation2d min, Rotation2d max) {
        double angle = rotation.getRadians();
        double minAngle = min.getRadians();
        double maxAngle = max.getRadians();

        if (angle < minAngle) {
            return Rotation2d.fromRadians(minAngle);
        } else if (angle > maxAngle) {
            return Rotation2d.fromRadians(maxAngle);
        } else {
            return rotation;
        }
    }

    /** Returns true if the two rotations are within a certain tolerance of each other. */
    public static boolean withinTolerance(
            Rotation2d rotation1, Rotation2d rotation2, Rotation2d tolerance) {
        return Math.abs(rotation1.getRadians() - rotation2.getRadians()) < tolerance.getRadians();
    }
}
