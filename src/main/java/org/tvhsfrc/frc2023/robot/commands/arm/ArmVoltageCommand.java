package org.tvhsfrc.frc2023.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;
import org.tvhsfrc.frc2023.robot.subsystems.ArmSubsystem;

public class ArmVoltageCommand extends CommandBase {
    private final ArmSubsystem arm;
    private final DoubleSupplier delta;
    private static final double SCALE = 0.02 * 0.1;

    private double voltage;

    public ArmVoltageCommand(ArmSubsystem arm, DoubleSupplier delta) {
        this.arm = arm;
        this.delta = delta;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        voltage += delta.getAsDouble() * SCALE;
        arm.setVoltage(voltage);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
