package org.tvhsfrc.frc2023.robot.subsystems;

import static org.tvhsfrc.frc2023.robot.Constants.Arm.STAGE_1_LENGTH;
import static org.tvhsfrc.frc2023.robot.Constants.Arm.STAGE_2_LENGTH;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    private Pose2d targetPos;

    /**
     * This just takes the inputs for the Law of Cosines, and spits out the desired angle, gamma.
     * @param a a leg
     * @param b a leg
     * @param c side opposite the returned angle (gamma)
     * @return Gives the value of gamma in radians
     */
    public static double lawOfCosines(double a, double b, double c) {
        return Math.acos((a*a + b*b - c)/(2*a*b));
    }

    private static Pair<Rotation2d, Rotation2d> inverseKinematics(Translation2d translation) {
        double theta = lawOfCosines(STAGE_1_LENGTH, translation.getNorm(), STAGE_2_LENGTH);
        double alpha =  translation.getAngle().getRadians() + theta;

        double beta = lawOfCosines(STAGE_1_LENGTH, STAGE_2_LENGTH, translation.getNorm());

        return new Pair<>(Rotation2d.fromRadians(alpha), Rotation2d.fromRadians(beta));
    }

    private static Translation2d kinematics(Rotation2d alpha, Rotation2d beta) {
        return null;
    }

    public void setTargetPos(Pose2d targetPos) {
        this.targetPos = targetPos;
    }

    public Pose2d getTargetPos() {
        return targetPos;
    }
}
