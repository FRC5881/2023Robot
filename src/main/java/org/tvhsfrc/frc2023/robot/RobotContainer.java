// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.tvhsfrc.frc2023.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
import org.tvhsfrc.frc2023.robot.Constants.OperatorConstants;
import org.tvhsfrc.frc2023.robot.commands.Autos;
import org.tvhsfrc.frc2023.robot.commands.DefaultDriveCommand;
import org.tvhsfrc.frc2023.robot.subsystems.DriveTrainSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    public final DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem();
    // private final VacuumSubsystem vacuumSubsystem = new VacuumSubsystem();

    // Driver controller
    private final CommandXboxController driverController =
            new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

    // ROBORIO "User" button
    Trigger userButton = new Trigger(RobotController::getUserButton);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
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

        DoubleSupplier vx =
                () ->
                        modifyAxis(driverController.getLeftY())
                                * Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND;
        DoubleSupplier vy =
                () ->
                        -modifyAxis(driverController.getLeftX())
                                * Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND;
        DoubleSupplier angle =
                () ->
                        -modifyAxis(driverController.getRightX())
                                * Constants.Swerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

        driveTrainSubsystem.setDefaultCommand(
                new DefaultDriveCommand(driveTrainSubsystem, vx, vy, angle));

        // Resets the field heading
        driverController.x().onTrue(driveTrainSubsystem.cZeroHeading());

        userButton.onTrue(driveTrainSubsystem.cCalibrateGyro());

        // driverController.a().toggleOnTrue(new VacuumToggleCommand(vacuumSubsystem));
        // driverController.b().whileTrue(new HoldVacuumCommand(vacuumSubsystem));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return Autos.loadPath("test", driveTrainSubsystem).get();
    }

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    private static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, 0.07);

        // Square the axis
        value = Math.copySign(value * value, value);

        return value;
    }
}
