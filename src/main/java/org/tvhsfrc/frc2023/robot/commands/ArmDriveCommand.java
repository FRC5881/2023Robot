package org.tvhsfrc.frc2023.robot.commands;

import java.util.function.DoubleSupplier;

import org.tvhsfrc.frc2023.robot.subsystems.ArmSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmDriveCommand extends CommandBase {
    private final ArmSubsystem armSubsystem;

    private double stage1delta = 0;
    private double stage2delta = 0;
    private double stage3delta = 0;

    private final DoubleSupplier stage1;
    private final DoubleSupplier stage2;
    private final DoubleSupplier stage3;

    // In rotations per 20ms
    private final double STAGE_1_RATE = (1.0 / 360) * 0.05;
    private final double STAGE_2_RATE = (1.0 / 360) * 0.05;
    private final double STAGE_3_RATE = (2.5 / 360) * 0.05;

    public ArmDriveCommand(ArmSubsystem armSubsystem, DoubleSupplier stage1, DoubleSupplier stage2,
            DoubleSupplier stage3) {
        this.armSubsystem = armSubsystem;
        this.stage1 = stage1;
        this.stage2 = stage2;
        this.stage3 = stage3;
        
        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        stage1delta = stage1.getAsDouble() * STAGE_1_RATE;
        stage2delta = stage2.getAsDouble() * STAGE_2_RATE;
        stage3delta = stage3.getAsDouble() * STAGE_3_RATE;

        armSubsystem.setStage1Rotations(armSubsystem.getStage1Rotations() + stage1delta);
        armSubsystem.setStage2Rotations(armSubsystem.getStage2Rotations() + stage2delta);
        armSubsystem.setStage3Rotations(armSubsystem.getStage3Rotations() + stage3delta);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
