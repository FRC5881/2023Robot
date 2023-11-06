package org.tvhsfrc.frc2023.robot.commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;
import org.tvhsfrc.frc2023.robot.subsystems.ArmSubsystem;

public class ArmDriveCommand extends CommandBase {
    private final ArmSubsystem arm;
    private final DoubleSupplier supplier;

    private static final double STAGE_2_RATE = (120.0 / 360) * 0.02;
    private Rotation2d previous;

    public ArmDriveCommand(ArmSubsystem arm, DoubleSupplier supplier) {
        this.arm = arm;
        this.supplier = supplier;
        addRequirements(arm);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        previous = arm.getStage2Setpoint();
        Rotation2d stage2 =
                previous.plus(Rotation2d.fromRotations(supplier.getAsDouble() * STAGE_2_RATE));
        arm.setStage2Setpoint(stage2);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
