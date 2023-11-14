// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.tvhsfrc.frc2023.robot.commands.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import org.tvhsfrc.frc2023.robot.Constants.WAYPOINT;
import org.tvhsfrc.frc2023.robot.commands.intake.IntakeIn;
import org.tvhsfrc.frc2023.robot.subsystems.ArmSubsystem;
import org.tvhsfrc.frc2023.robot.subsystems.IntakeSubsystem;
import org.tvhsfrc.frc2023.robot.subsystems.SwerveSubsystem;

public final class Autos {
    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    public static CommandBase doNothing() {
        return Commands.none();
    }

    public static CommandBase autoline(SwerveSubsystem swerve) {
        return Commands.run(
                        () -> swerve.drive(new Translation2d(-1.0, 0), 0, true, false, true),
                        swerve)
                .withTimeout(5);
    }

    public static CommandBase scoreAndLine(
            SwerveSubsystem swerve, ArmSubsystem arm, IntakeSubsystem intake) {
        return Commands.sequence(
                arm.setArmGoalCommand(WAYPOINT.MID_CUBE, true).withTimeout(2),
                new IntakeIn(intake).withTimeout(3),
                Commands.parallel(
                        Commands.run(
                                        () ->
                                                swerve.drive(
                                                        new Translation2d(1.0, 0),
                                                        0,
                                                        true,
                                                        false,
                                                        true),
                                        swerve)
                                .withTimeout(5),
                        arm.setArmGoalCommand(WAYPOINT.HOME, true).withTimeout(2)));
    }

    public static CommandBase score(
            SwerveSubsystem swerve, ArmSubsystem arm, IntakeSubsystem intake) {
        return Commands.sequence(
                arm.setArmGoalCommand(WAYPOINT.MID_CUBE, true).withTimeout(2),
                new IntakeIn(intake).withTimeout(3),
                arm.setArmGoalCommand(WAYPOINT.HOME, true).withTimeout(2));
    }
}
;
