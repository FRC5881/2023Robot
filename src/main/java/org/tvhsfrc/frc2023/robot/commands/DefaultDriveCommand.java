package org.tvhsfrc.frc2023.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;
import org.tvhsfrc.frc2023.robot.subsystems.DriveTrainSubsystem;

public class DefaultDriveCommand extends CommandBase {
    private final DriveTrainSubsystem drivetrainSubsystem;

    private final DoubleSupplier xSpeedSupplier;
    private final DoubleSupplier ySpeedSupplier;
    private final DoubleSupplier rotationSupplier;

    public DefaultDriveCommand(
            DriveTrainSubsystem drivetrainSubsystem,
            DoubleSupplier xSpeedSupplier,
            DoubleSupplier ySpeedSupplier,
            DoubleSupplier rotationSupplier) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        this.rotationSupplier = rotationSupplier;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of
        // field-oriented movement
        drivetrainSubsystem.orientedDrive(
                xSpeedSupplier.getAsDouble(),
                ySpeedSupplier.getAsDouble(),
                rotationSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
