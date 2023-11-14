package org.tvhsfrc.frc2023.robot.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;
import org.tvhsfrc.frc2023.robot.subsystems.ArmSubsystem;

public class ArmDriveCommand extends CommandBase {
    private final ArmSubsystem arm;
    private final DoubleSupplier supplier;

    private static final double RATE = (120.0 / 360) * 0.02;

    public ArmDriveCommand(ArmSubsystem arm, DoubleSupplier supplier) {
        this.arm = arm;
        this.supplier = supplier;
        addRequirements(arm);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        Rotation2d previous = arm.getGoal();
        Rotation2d delta = Rotation2d.fromRotations(supplier.getAsDouble() * RATE);

        arm.setGoal(previous.plus(delta));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
