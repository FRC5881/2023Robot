package org.tvhsfrc.frc2023.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;
import org.tvhsfrc.frc2023.robot.subsystems.SwerveSubsystem;

/**
 * A drive command that provides field relative translation with 1 joystick and absolute rotation
 * with another axis.
 */
public class RelativeAbsoluteDrive extends CommandBase {
    private final SwerveSubsystem swerve;
    private final DoubleSupplier xInput, yInput, xHeading, yHeading;

    public RelativeAbsoluteDrive(
            SwerveSubsystem swerve,
            DoubleSupplier xInput,
            DoubleSupplier yInput,
            DoubleSupplier xHeading,
            DoubleSupplier yHeading) {
        this.swerve = swerve;
        this.xInput = xInput;
        this.yInput = yInput;
        this.xHeading = xHeading;
        this.yHeading = yHeading;
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds =
                swerve.getTargetSpeeds(
                        xInput.getAsDouble(),
                        yInput.getAsDouble(),
                        xHeading.getAsDouble(),
                        yHeading.getAsDouble());

        swerve.setChassisSpeeds(speeds);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
