// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.tvhsfrc.frc2023.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.io.File;
import java.util.HashMap;
import java.util.Optional;
import org.tvhsfrc.frc2023.robot.Constants.Arm.ARM_TARGET;
import org.tvhsfrc.frc2023.robot.Constants.OperatorConstants;
import org.tvhsfrc.frc2023.robot.commands.arm.ArmDriveCommand;
import org.tvhsfrc.frc2023.robot.commands.arm.ArmNext;
import org.tvhsfrc.frc2023.robot.commands.auto.Autos;
import org.tvhsfrc.frc2023.robot.commands.auto.Tests;
import org.tvhsfrc.frc2023.robot.commands.drive.RelativeRelativeDrive;
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
    private final CommandPS4Controller driverController =
            new CommandPS4Controller(OperatorConstants.DRIVER_CONTROLLER_PORT);

    private final CommandPS4Controller armController =
            new CommandPS4Controller(OperatorConstants.ARM_CONTROLLER_PORT);

    //     private final CommandXboxController armController =
    //     new CommandXboxController(OperatorConstants.ARM_CONTROLLER_PORT);

    // ROBORIO "User" button
    Trigger userButton = new Trigger(RobotController::getUserButton);

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
        // ------ Driver Controller ------ //
        driverController.touchpad().onTrue(new InstantCommand(swerveSubsystem::zeroGyro));

        RelativeRelativeDrive drive =
                new RelativeRelativeDrive(
                        swerveSubsystem,
                        () -> deadband(driverController.getLeftY(), 0.15),
                        () -> deadband(driverController.getLeftX(), 0.15),
                        () -> deadband(driverController.getRawAxis(2), 0.15));

        swerveSubsystem.setDefaultCommand(drive);

        setupArmController(armController);
    }

    public void setupArmController(CommandXboxController controller) {
        // POV Left goes to Floor pickup
        controller.povLeft().onTrue(new InstantCommand(() -> arm.setArmTarget(ARM_TARGET.FLOOR)));

        // POV Down goes to score Low
        controller.povDown().onTrue(new InstantCommand(() -> arm.setArmTarget(ARM_TARGET.LOW)));

        // POV Right goes to score Mid
        controller.povRight().onTrue(new InstantCommand(() -> arm.setArmTarget(ARM_TARGET.MID)));

        // POV Up goes to score High
        controller.povUp().onTrue(new InstantCommand(() -> arm.setArmTarget(ARM_TARGET.HIGH)));

        // Back moves the arm to take a cone or cube of the slide part of teh double substation.
        controller
                .back()
                .onTrue(new InstantCommand(() -> arm.setArmTarget(ARM_TARGET.DOUBLE_SUBSTATION)));

        // Square button sets mode to Cube
        controller.x().onTrue(new InstantCommand(arm::gamePieceCube));

        // Triangle button sets mode to Cone
        controller.y().onTrue(new InstantCommand(arm::gamePieceCone));

        // Cross button tells the arm to move to the Waypoint
        controller.b().onTrue(new ArmNext(arm));

        // Circle button sends the arm to the HOME Waypoint
        controller.a().onTrue(arm.cGoToWaypoint(ARM_TARGET.HOME));

        // Left bumper turns vacuum on
        controller
                .leftBumper()
                .onTrue(Commands.sequence(new InstantCommand(vacuumSubsystem::vacuum)));

        // Right bumper turns vacuum off
        controller
                .rightBumper()
                .onTrue(Commands.sequence(new InstantCommand(vacuumSubsystem::dump)));

        // Manual arm control
        arm.setDefaultCommand(
                new ArmDriveCommand(
                        arm,
                        () -> -deadband(controller.getRawAxis(1), 0.2),
                        () -> -deadband(controller.getRawAxis(5), 0.2),
                        () -> {
                            double left = controller.getRawAxis(2);
                            double right = controller.getRawAxis(3);

                            return deadband(right - left, 0.2);
                        }));
    }

    public void setupArmController(CommandPS4Controller controller) {
        // POV Left goes to Floor pickup
        controller.povLeft().onTrue(new InstantCommand(() -> arm.setArmTarget(ARM_TARGET.FLOOR)));

        // POV Down goes to score Low
        controller.povDown().onTrue(new InstantCommand(() -> arm.setArmTarget(ARM_TARGET.LOW)));

        // POV Right goes to score Mid
        controller.povRight().onTrue(new InstantCommand(() -> arm.setArmTarget(ARM_TARGET.MID)));

        // POV Up goes to score High
        controller.povUp().onTrue(new InstantCommand(() -> arm.setArmTarget(ARM_TARGET.HIGH)));

        // Touchpad moves the arm to take a cone or cube of the slide part of teh double substation.
        // armController.share().onTrue(new InstantCommand(() ->
        // arm.setArmTarget(ARM_TARGET.DOUBLE_SUBSTATION)));
        controller
                .touchpad()
                .onTrue(new InstantCommand(() -> arm.setArmTarget(ARM_TARGET.DOUBLE_SUBSTATION)));

        // Square button sets mode to Cube
        controller.square().onTrue(new InstantCommand(arm::gamePieceCube));

        // Triangle button sets mode to Cone
        controller.triangle().onTrue(new InstantCommand(arm::gamePieceCone));

        // Cross button tells the arm to move to the Waypoint
        controller.cross().onTrue(new ArmNext(arm));

        // Circle button sends the arm to the HOME Waypoint
        controller.circle().onTrue(arm.cGoToWaypoint(ARM_TARGET.HOME));

        // Left bumper turns vacuum on
        controller.L1().onTrue(Commands.sequence(new InstantCommand(vacuumSubsystem::vacuum)));

        // Right bumper turns vacuum off
        controller.R1().onTrue(Commands.sequence(new InstantCommand(vacuumSubsystem::dump)));

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

    private final String kAutoline = "autoline";
    private final String kNothing = "nothing";

    private String selected;

    private final SendableChooser<String> sendableChooser = new SendableChooser<>();

    private static double deadband(double value) {
        return deadband(value, 0.07);
    }

    private static double deadband(double value, double deadband) {
        return Math.abs(value) > deadband ? value : 0.0;
    }
}
