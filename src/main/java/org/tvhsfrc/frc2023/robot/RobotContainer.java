// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.tvhsfrc.frc2023.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.io.File;
import org.tvhsfrc.frc2023.robot.Constants.OperatorConstants;
import org.tvhsfrc.frc2023.robot.Constants.WAYPOINT;
import org.tvhsfrc.frc2023.robot.commands.arm.ArmDriveCommand;
import org.tvhsfrc.frc2023.robot.commands.arm.ArmWaypoint;
import org.tvhsfrc.frc2023.robot.commands.auto.Autos;
import org.tvhsfrc.frc2023.robot.commands.drive.RelativeRelativeDrive;
import org.tvhsfrc.frc2023.robot.commands.intake.IntakeIn;
import org.tvhsfrc.frc2023.robot.commands.intake.IntakeOut;
import org.tvhsfrc.frc2023.robot.subsystems.ArmSubsystem;
import org.tvhsfrc.frc2023.robot.subsystems.IntakeSubsystem;
import org.tvhsfrc.frc2023.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    public final SwerveSubsystem swerveSubsystem =
            new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    public final ArmSubsystem arm = new ArmSubsystem();
    public final IntakeSubsystem intake = new IntakeSubsystem();

    // Driver controller
    private final CommandPS4Controller controller =
            new CommandPS4Controller(OperatorConstants.DRIVER_CONTROLLER_PORT);

    // roboRIO "User" button
    Trigger userButton = new Trigger(RobotController::getUserButton);

    private final SendableChooser<String> sendableChooser = new SendableChooser<>();

    private final String kAutoline = "autoline";
    private final String kNothing = "nothing";

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();

        sendableChooser.setDefaultOption(kNothing, kNothing);
        sendableChooser.addOption(kAutoline, kAutoline);

        SmartDashboard.putData(sendableChooser);
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        // ------ Driving ------ //
        controller.touchpad().onTrue(new InstantCommand(swerveSubsystem::zeroGyro));

        RelativeRelativeDrive drive =
                new RelativeRelativeDrive(
                        swerveSubsystem,
                        () -> deadband(controller.getLeftY(), 0.10),
                        () -> deadband(controller.getLeftX(), 0.10),
                        () -> deadband(controller.getRightX(), 0.10));

        swerveSubsystem.setDefaultCommand(drive);

        // Manual arm control
        arm.setDefaultCommand(
                new ArmDriveCommand(
                        arm,
                        () -> {
                            double left = (controller.getRawAxis(3) + 1) / 2.0;
                            double right = (controller.getRawAxis(4) + 1) / 2.0;

                            return right - left;
                        }));

        controller.circle().onTrue(new ArmWaypoint(arm, WAYPOINT.HOME));
        controller.povDown().onTrue(new ArmWaypoint(arm, WAYPOINT.LOW_CUBE));
        controller.povLeft().onTrue(new ArmWaypoint(arm, WAYPOINT.MID_CUBE));
        controller.povUp().onTrue(new ArmWaypoint(arm, WAYPOINT.HIGH_CUBE));
        controller.triangle().onTrue(new ArmWaypoint(arm, WAYPOINT.DOUBLE_SUBSTATION_CUBE));

        controller.L1().whileTrue(new IntakeIn(intake));
        controller.R1().whileTrue(new IntakeOut(intake));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        switch (sendableChooser.getSelected()) {
            case kNothing:
                return Autos.doNothing();
            case kAutoline:
                return Autos.autoline(swerveSubsystem);
            default:
                return Autos.doNothing();
        }
    }

    private static double deadband(double value, double deadband) {
        return Math.abs(value) > deadband ? value : 0.0;
    }
}
