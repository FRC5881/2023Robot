package org.tvhsfrc.frc2023.robot.commands.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;
import org.tvhsfrc.frc2023.robot.Constants;
import org.tvhsfrc.frc2023.robot.subsystems.ArmSubsystem;

public class ArmDriveCommand extends CommandBase {
    private final ArmSubsystem armSubsystem;

    private double stage1 = 0;
    private double stage2 = 0;

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
    public void initialize() {
        // Reset the deltas
        stage1 = armSubsystem.getStage1SetPoint();
        stage2 = armSubsystem.getStage2SetPoint();
    }

    @Override
    public void execute() {
        stage1 += stage1DoubleSupplier.getAsDouble() * STAGE_1_RATE;
        stage2 += stage2DoubleSupplier.getAsDouble() * STAGE_2_RATE;

        // clamp the values
        stage1 = MathUtil.clamp(stage1, Constants.Arm.STAGE_1_HOME, Constants.Arm.STAGE_1_LIMIT);
        stage2 = MathUtil.clamp(stage2, Constants.Arm.STAGE_2_HOME, Constants.Arm.STAGE_2_LIMIT);

        armSubsystem.setStage1Rotations(stage1);
        armSubsystem.setStage2Rotations(stage2);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
