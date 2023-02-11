package org.tvhsfrc.frc2023.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.tvhsfrc.frc2023.robot.Constants;

public class VacuumSubsystem extends SubsystemBase {
    private final CANSparkMax vacuum1 =
            new CANSparkMax(
                    Constants.CANConstants.VACUUM_ONE, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax vacuum2 =
            new CANSparkMax(
                    Constants.CANConstants.VACUUM_TWO, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax vacuum3 =
            new CANSparkMax(
                    Constants.CANConstants.VACUUM_THREE, CANSparkMaxLowLevel.MotorType.kBrushless);

    private boolean isEnabled;

    public VacuumSubsystem() {
        vacuum2.follow(vacuum1);
        vacuum3.follow(vacuum1);

        vacuum1.getPIDController().setP(0.00005);
        vacuum1.getPIDController().setI(0.0);
        vacuum1.getPIDController().setD(0.002);
        vacuum1.getPIDController().setFF(0.0001);
        vacuum1.setIdleMode(CANSparkMax.IdleMode.kCoast);
        /*
         * Original motor was rated for 5500 RPM.
         * Our motor is rated for ~11000 RPM.
         * 0.6 current limiting keeps it from exceeding the original rating by an excessive amount.
         */
        vacuum1.getPIDController().setOutputRange(0, Constants.VacuumConstants.maxOutput);
        vacuum1.setClosedLoopRampRate(0.5);
    }

    @Override
    public void periodic() {
        if (isEnabled) {
            vacuum1.getPIDController()
                    .setReference(
                            Constants.VacuumConstants.VacuumVelocity,
                            CANSparkMax.ControlType.kVelocity);
        } else {
            vacuum1.getPIDController().setReference(0.0, CANSparkMax.ControlType.kVelocity);
        }
    }

    /**
     * Toggles the vacuum on and off.
     */
    public void toggle() {
        isEnabled = !isEnabled;
    }

    public void enable() {
        isEnabled = true;
    }

    public void disable() {
        isEnabled = false;
    }
}
