package org.tvhsfrc.frc2023.robot.commands.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;
import org.tvhsfrc.frc2023.robot.subsystems.SwerveSubsystem;

/**
 * A drive command that provides field relative translation with 1 joystick and relative rotation
 * with another axis.
 */
public class RelativeRelativeDrive extends CommandBase {
    private final SwerveSubsystem swerve;
    private final DoubleSupplier vx, vy, omega;

    public RelativeRelativeDrive(
            SwerveSubsystem swerve, DoubleSupplier vx, DoubleSupplier vy, DoubleSupplier omega) {
        this.swerve = swerve;
        this.vx = vx;
        this.vy = vy;
        this.omega = omega;
        addRequirements(swerve);
    }

    @Override
    public void execute() {
        Translation2d translation =
                new Translation2d(vx.getAsDouble(), vy.getAsDouble())
                        .times(swerve.getSwerveDriveConfiguration().maxSpeed);

        swerve.drive(translation, omega.getAsDouble() * 2 * Math.PI, true, false, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
