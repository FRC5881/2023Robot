package org.tvhsfrc.frc2023.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.tvhsfrc.frc2023.robot.subsystems.ArmSubsystem;

public class Stage1ArmZero extends CommandBase {
    private final ArmSubsystem armSubsystem;

    public Stage1ArmZero(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.setStage1(0.0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
