package org.tvhsfrc.frc2023.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.tvhsfrc.frc2023.robot.subsystems.ArmSubsystem;
import org.tvhsfrc.frc2023.robot.utils.Triple;

public class SetArmSetpoint extends CommandBase {
    private final ArmSubsystem armSubsystem;

    private final Rotation2d stage1Rotations;
    private final Rotation2d stage2Rotations;
    private final Rotation2d stage3Rotations;

    private final boolean hold;

    public SetArmSetpoint(
            ArmSubsystem armSubsystem,
            double stage1Rotations,
            double stage2Rotations,
            double stage3Rotations,
            boolean hold) {
        this.armSubsystem = armSubsystem;

        this.stage1Rotations = Rotation2d.fromRotations(stage1Rotations);
        this.stage2Rotations = Rotation2d.fromRotations(stage2Rotations);
        this.stage3Rotations = Rotation2d.fromRotations(stage3Rotations);

        this.hold = hold;
    }

    public SetArmSetpoint(
            ArmSubsystem armSubsystem,
            Rotation2d stage1Rotations,
            Rotation2d stage2Rotations,
            Rotation2d stage3Rotations,
            boolean hold) {
        this.armSubsystem = armSubsystem;

        this.stage1Rotations = stage1Rotations;
        this.stage2Rotations = stage2Rotations;
        this.stage3Rotations = stage3Rotations;

        this.hold = hold;
    }

    public SetArmSetpoint(
            ArmSubsystem armSubsystem,
            Triple<Rotation2d, Rotation2d, Rotation2d> angles,
            boolean hold) {
        this(armSubsystem, angles.getA(), angles.getB(), angles.getC(), hold);
    }

    @Override
    public void initialize() {
        armSubsystem.setStage1Rotations(this.stage1Rotations);
        armSubsystem.setStage2Rotations(this.stage2Rotations);
        armSubsystem.setStage3Rotations(this.stage3Rotations);
    }

    @Override
    public boolean isFinished() {
        if (hold) {
            return false;
        } else {
            return armSubsystem.isAtSetpoint();
        }
    }
}
