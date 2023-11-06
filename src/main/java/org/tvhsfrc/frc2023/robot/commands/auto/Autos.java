// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.tvhsfrc.frc2023.robot.commands.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
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
        return Commands.deadline(
                Commands.waitSeconds(5),
                Commands.run(
                        () -> swerve.drive(new Translation2d(-1.0, 0), 0, true, false, true),
                        swerve));
    }

    public static CommandBase scoreAndLine(
            SwerveSubsystem swerve, ArmSubsystem arm, IntakeSubsystem intake) {
        return Commands.sequence(
                Commands.deadline(
                        Commands.waitSeconds(2),
                        Commands.run(
                                () -> arm.setStage2Setpoint(Rotation2d.fromRotations(0.19)), arm)),
                Commands.deadline(Commands.waitSeconds(3), new IntakeIn(intake)),
                Commands.deadline(
                        Commands.waitSeconds(5),
                        Commands.run(
                                () -> swerve.drive(new Translation2d(1.0, 0), 0, true, false, true),
                                swerve)),
                Commands.deadline(
                        Commands.waitSeconds(2),
                        Commands.run(
                                () -> arm.setStage2Setpoint(Rotation2d.fromRotations(0)), arm)));
    }

    public static CommandBase score(
            SwerveSubsystem swerve, ArmSubsystem arm, IntakeSubsystem intake) {
        return Commands.sequence(
                Commands.deadline(
                        Commands.waitSeconds(2),
                        Commands.run(
                                () -> arm.setStage2Setpoint(Rotation2d.fromRotations(0.19)), arm)),
                Commands.deadline(Commands.waitSeconds(3), new IntakeIn(intake)),
                Commands.deadline(
                        Commands.waitSeconds(2),
                        Commands.run(
                                () -> arm.setStage2Setpoint(Rotation2d.fromRotations(0)), arm)));
    }
}
;
