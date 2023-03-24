package org.tvhsfrc.frc2023.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.ArrayList;
import org.tvhsfrc.frc2023.robot.Constants.WayPoint;
import org.tvhsfrc.frc2023.robot.subsystems.ArmSubsystem;
import org.tvhsfrc.frc2023.robot.utils.Triple;

public class ArmTrajectory extends SequentialCommandGroup {
    private final ArmSubsystem armSubsystem;

    public ArmTrajectory(ArmSubsystem armSubsystem, ArrayList<WayPoint> points) {
        this.armSubsystem = armSubsystem;

        //
        for (int i = 0; i < points.size() - 1; i++) {
            Triple<Rotation2d, Rotation2d, Rotation2d> position = points.get(i).position;

            addCommands(
                    new SetArmSetpoint(
                            armSubsystem,
                            position.getA(),
                            position.getB(),
                            position.getC(),
                            false));
        }

        Triple<Rotation2d, Rotation2d, Rotation2d> position =
                points.get(points.size() - 1).position;
        addCommands(
                new SetArmSetpoint(
                        armSubsystem, position.getA(), position.getB(), position.getC(), true));
    }
}
