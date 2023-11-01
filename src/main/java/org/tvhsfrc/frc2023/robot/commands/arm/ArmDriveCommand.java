package org.tvhsfrc.frc2023.robot.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;
import org.tvhsfrc.frc2023.robot.subsystems.ArmSubsystem;

public class ArmDriveCommand extends CommandBase {
    private final ArmSubsystem armSubsystem;

    private final DoubleSupplier stage1DoubleSupplier;
    private final DoubleSupplier stage2DoubleSupplier;

    // In rotations per 20ms
    private static final double STAGE_1_RATE = (10.0 / 360) * 0.02;
    private static final double STAGE_2_RATE = (120.0 / 360) * 0.02;

    public ArmDriveCommand(
            ArmSubsystem armSubsystem,
            DoubleSupplier stage1DoubleSupplier,
            DoubleSupplier stage2DoubleSupplier) {

        this.armSubsystem = armSubsystem;
        this.stage1DoubleSupplier = stage1DoubleSupplier;
        this.stage2DoubleSupplier = stage2DoubleSupplier;

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        Rotation2d stage1 =
                Rotation2d.fromRotations(stage1DoubleSupplier.getAsDouble() * STAGE_1_RATE);
        Rotation2d stage2 =
                Rotation2d.fromRotations(stage2DoubleSupplier.getAsDouble() * STAGE_2_RATE);

        armSubsystem.addSetpoint(stage1, stage2);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
