// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.tvhsfrc.frc2023.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Optional;
import org.tvhsfrc.frc2023.robot.Constants.Autonomous;
import org.tvhsfrc.frc2023.robot.subsystems.DriveTrainSubsystem;

public final class Autos {
    public static Command doNothing() {
        return Commands.none();
    }

    public static Optional<Command> loadPath(
            String pathName, DriveTrainSubsystem driveTrainSubsystem) {
        var path =
                PathPlanner.loadPath(
                        pathName,
                        new PathConstraints(Autonomous.MAX_SPEED, Autonomous.MAX_ACCELERATION));

        if (path == null) {
            return Optional.empty();
        } else {
            return Optional.of(driveTrainSubsystem.followTrajectory(path, true));
        }
    }

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
