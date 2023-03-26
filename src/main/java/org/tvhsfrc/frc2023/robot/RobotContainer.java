// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.tvhsfrc.frc2023.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.io.File;
import java.util.Optional;
import org.tvhsfrc.frc2023.robot.Constants.Arm.ARM_TARGET;
import org.tvhsfrc.frc2023.robot.Constants.OperatorConstants;
import org.tvhsfrc.frc2023.robot.commands.arm.ArmDriveCommand;
import org.tvhsfrc.frc2023.robot.commands.arm.ArmNext;
import org.tvhsfrc.frc2023.robot.commands.arm.VacuumCommand;
import org.tvhsfrc.frc2023.robot.commands.auto.Autos;
import org.tvhsfrc.frc2023.robot.commands.drive.AbsoluteDrive;
import org.tvhsfrc.frc2023.robot.commands.drive.AbsoluteFieldDrive;
import org.tvhsfrc.frc2023.robot.commands.drive.TeleopDrive;
import org.tvhsfrc.frc2023.robot.subsystems.ArmSubsystem;
import org.tvhsfrc.frc2023.robot.subsystems.SwerveSubsystem;
import org.tvhsfrc.frc2023.robot.subsystems.VacuumSubsystem;
import org.tvhsfrc.frc2023.robot.subsystems.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final Optional<VisionSubsystem> visionSubsystem = Optional.empty();
    // private final Optional<VisionSubsystem> visionSubsystem = Optional.of(new VisionSubsystem());

    private final SwerveSubsystem swerveSubsystem =
            new SwerveSubsystem(
                    new File(Filesystem.getDeployDirectory(), "swerve"), visionSubsystem);
    private final ArmSubsystem arm = new ArmSubsystem();
    private final PowerDistribution pdh =
            new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
    private final VacuumSubsystem vacuumSubsystem = new VacuumSubsystem(pdh);

    // Driver controller
    private final CommandXboxController driverController =
            new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
    private final CommandPS4Controller armController =
            new CommandPS4Controller(OperatorConstants.ARM_CONTROLLER_PORT);

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
        driverController.start().onTrue(new InstantCommand(swerveSubsystem::zeroGyro));

        AbsoluteDrive closedAbsoluteDrive =
                new AbsoluteDrive(
                        swerveSubsystem,
                        // Applies deadbands and inverts controls because joysticks
                        // are back-right positive while robot
                        // controls are front-left positive
                        () -> deadband(driverController.getLeftY()),
                        () -> deadband(driverController.getLeftX()),
                        () -> driverController.getRightX(),
                        () -> -driverController.getRightY(),
                        false);

        AbsoluteFieldDrive closedFieldAbsoluteDrive =
                new AbsoluteFieldDrive(
                        swerveSubsystem,
                        () -> deadband(driverController.getLeftY()),
                        () -> deadband(driverController.getLeftX()),
                        () -> deadband(driverController.getRightX()),
                        false);

        TeleopDrive closedFieldRel =
                new TeleopDrive(
                        swerveSubsystem,
                        () -> deadband(driverController.getLeftY()),
                        () -> deadband(driverController.getLeftX()),
                        () -> -driverController.getRawAxis(4),
                        () -> true,
                        false,
                        true);

        swerveSubsystem.setDefaultCommand(closedFieldRel);

        // POV Down cycle arm targets
        armController.povUp().onTrue(new InstantCommand(() -> arm.cycleArmTarget(false)));
        armController.povDown().onTrue(new InstantCommand(() -> arm.cycleArmTarget(true)));

        // POV Left/Right cycle arm mode (cube vs cone)
        armController.povLeft().onTrue(new InstantCommand(arm::toggleGamePiece));
        armController.povRight().onTrue(new InstantCommand(arm::toggleGamePiece));

        // X button brings the arm to the next target
        armController.cross().onTrue(new ArmNext(arm));
        armController
                .circle()
                .onTrue(
                        Commands.sequence(
                                new InstantCommand(() -> arm.setArmTarget(ARM_TARGET.HOME)),
                                new ArmNext(arm)));

        // Touchpad toggles vacuum
        armController.touchpad().onTrue(new VacuumCommand(vacuumSubsystem));

        // Manual arm control
        arm.setDefaultCommand(
                new ArmDriveCommand(
                        arm,
                        () -> -deadband(armController.getRawAxis(1)),
                        () -> -deadband(armController.getRawAxis(5)),
                        () -> {
                            double left = (armController.getRawAxis(3) + 1) / 2.0;
                            double right = (armController.getRawAxis(4) + 1) / 2.0;

                            return right - left;
                        }));

        // new JoystickButton(driverController, XboxController.Button.kX.value).onTrue(arm.cHome());
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
}
