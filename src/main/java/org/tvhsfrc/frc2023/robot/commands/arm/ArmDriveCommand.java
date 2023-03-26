package org.tvhsfrc.frc2023.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;
import org.tvhsfrc.frc2023.robot.subsystems.ArmSubsystem;

public class ArmDriveCommand extends CommandBase {
    private final ArmSubsystem armSubsystem;

    private double stage1delta = 0;
    private double stage2delta = 0;
    private double stage3delta = 0;

    private final DoubleSupplier stage1DoubleSupplier;
    private final DoubleSupplier stage2DoubleSupplier;
    private final DoubleSupplier stage3DoubleSupplier;

    // In rotations per 20ms
    private final double STAGE_1_RATE = (1.0 / 360) * 0.05;
    private final double STAGE_2_RATE = (1.0 / 360) * 0.05;
    private final double STAGE_3_RATE = (2.5 / 360) * 0.05;

    public ArmDriveCommand(
            ArmSubsystem armSubsystem,
            DoubleSupplier stage1DoubleSupplier,
            DoubleSupplier stage2DoubleSupplier,
            DoubleSupplier stage3DoubleSupplier) {
        this.armSubsystem = armSubsystem;
        this.stage1DoubleSupplier = stage1DoubleSupplier;
        this.stage2DoubleSupplier = stage2DoubleSupplier;
        this.stage3DoubleSupplier = stage3DoubleSupplier;

        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        stage1delta = stage1DoubleSupplier.getAsDouble() * STAGE_1_RATE;
        stage2delta = stage2DoubleSupplier.getAsDouble() * STAGE_2_RATE;
        stage3delta = stage3DoubleSupplier.getAsDouble() * STAGE_3_RATE;

        armSubsystem.setStage1Rotations(armSubsystem.getStage1Rotations() + stage1delta);
        armSubsystem.setStage2Rotations(armSubsystem.getStage2Rotations() + stage2delta);
        armSubsystem.setStage3Rotations(armSubsystem.getStage3Rotations() + stage3delta);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
