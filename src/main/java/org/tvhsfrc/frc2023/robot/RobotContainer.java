// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.tvhsfrc.frc2023.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.io.File;
import org.tvhsfrc.frc2023.robot.Constants.OperatorConstants;
import org.tvhsfrc.frc2023.robot.commands.Autos;
import org.tvhsfrc.frc2023.robot.commands.drive.AbsoluteDrive;
import org.tvhsfrc.frc2023.robot.commands.drive.AbsoluteFieldDrive;
import org.tvhsfrc.frc2023.robot.commands.drive.TeleopDrive;
import org.tvhsfrc.frc2023.robot.subsystems.SwerveSubsystem;
import org.tvhsfrc.frc2023.robot.subsystems.VacuumSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    // private final DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem();
    private final SwerveSubsystem swerveSubsystem =
            new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    private final VacuumSubsystem vacuumSubsystem = new VacuumSubsystem();

    // Driver controller
    private final XboxController driverController =
            new XboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

    // ROBORIO "User" button
    Trigger userButton = new Trigger(RobotController::getUserButton);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();

        AbsoluteDrive closedAbsoluteDrive =
                new AbsoluteDrive(
                        swerveSubsystem,
                        // Applies deadbands and inverts controls because joysticks
                        // are back-right positive while robot
                        // controls are front-left positive
                        () -> deadband(driverController.getLeftY()),
                        () -> deadband(driverController.getLeftX()),
                        () -> -driverController.getRightX(),
                        () -> -driverController.getRightY(),
                        false);

        AbsoluteFieldDrive closedFieldAbsoluteDrive =
                new AbsoluteFieldDrive(
                        swerveSubsystem,
                        () -> deadband(driverController.getLeftY()),
                        () -> deadband(driverController.getLeftX()),
                        () -> driverController.getRawAxis(2),
                        false);
        TeleopDrive simClosedFieldRel =
                new TeleopDrive(
                        swerveSubsystem,
                        () -> deadband(driverController.getLeftY()),
                        () -> deadband(driverController.getLeftX()),
                        () -> driverController.getRawAxis(2),
                        () -> true,
                        false,
                        true);
        TeleopDrive closedFieldRel =
                new TeleopDrive(
                        swerveSubsystem,
                        () -> deadband(driverController.getLeftY()),
                        () -> deadband(driverController.getLeftX()),
                        () -> -driverController.getRawAxis(3),
                        () -> true,
                        false,
                        true);

        swerveSubsystem.setDefaultCommand(
                !RobotBase.isSimulation() ? closedAbsoluteDrive : closedFieldAbsoluteDrive);
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
        new JoystickButton(driverController, 8)
                .onTrue((new InstantCommand(swerveSubsystem::zeroGyro)));
        // new JoystickButton(driverController, 3).onTrue(new
        // InstantCommand(drivebase::addFakeVisionReading));

        // userButton.onTrue(driveTrainSubsystem.calibrate());

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return Autos.doNothing();
    }

    private static double deadband(double value) {
        return deadband(value, 0.07);
    }

    private static double deadband(double value, double deadband) {
        return Math.abs(value) > deadband ? value : 0.0;
    }

    private static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, 0.07);

        // Square the axis
        value = Math.copySign(value * value, value);

        return value;
    }
}
