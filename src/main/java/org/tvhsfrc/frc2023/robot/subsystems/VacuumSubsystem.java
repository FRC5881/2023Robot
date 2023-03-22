package org.tvhsfrc.frc2023.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.tvhsfrc.frc2023.robot.Constants;
import org.tvhsfrc.frc2023.robot.commands.vacuum.VacuumDisableCommand;
import org.tvhsfrc.frc2023.robot.commands.vacuum.VacuumEnableCommand;
import org.tvhsfrc.frc2023.robot.commands.vacuum.VacuumToggleCommand;

public class VacuumSubsystem extends SubsystemBase {
    private final PowerDistribution pdh;

    private final CANSparkMax vacuum1 =
            new CANSparkMax(
                    Constants.CANConstants.VACUUM_ONE, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax vacuum2 =
            new CANSparkMax(
                    Constants.CANConstants.VACUUM_TWO, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax vacuum3 =
            new CANSparkMax(
                    Constants.CANConstants.VACUUM_THREE, CANSparkMaxLowLevel.MotorType.kBrushless);

    /**
     * State of the subsystem: true if we're running the vacuum, false if we're not.
     *
     * <p>Changing this value will not cause the vacuums to start or stop. This is only used for
     * bookkeeping.
     */
    private boolean state = false;

    public VacuumSubsystem(PowerDistribution pdh) {
        this.pdh = pdh;

        vacuum1.getPIDController().setP(0.00005);
        vacuum1.getPIDController().setI(0.0);
        vacuum1.getPIDController().setD(0.002);
        vacuum1.getPIDController().setFF(0.0001);
        vacuum1.setIdleMode(CANSparkMax.IdleMode.kCoast);
        vacuum1.getPIDController().setOutputRange(0, Constants.Vacuum.MAX_OUTPUT);
        vacuum1.setClosedLoopRampRate(0.5);

        vacuum2.follow(vacuum1);
        vacuum3.follow(vacuum1);

        ShuffleboardTab tab = Shuffleboard.getTab("Vacuum");

        tab.add(new VacuumToggleCommand(this));
        tab.add(new VacuumEnableCommand(this));
        tab.add(new VacuumDisableCommand(this, Constants.Vacuum.PURGE_TIME));

        tab.add(this);

        suction(false);
        purge(false);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("Vacuum state", () -> state, null);
        builder.addBooleanProperty(
                "PDH switchable channel", () -> pdh.getSwitchableChannel(), null);
        builder.addDoubleProperty("Applied Output", () -> vacuum1.getAppliedOutput(), null);
    }

    /**
     * Run or stop the vacuum motors. Sending true will start the motors and create suction. Sending
     * false will stop the motors.
     *
     * @param shouldSuck
     */
    public void suction(boolean shouldSuck) {
        if (shouldSuck) {
            vacuum1.getPIDController()
                    .setReference(Constants.Vacuum.VELOCITY, CANSparkMax.ControlType.kVelocity);
        } else {
            vacuum1.stopMotor();
        }
    }

    /**
     * Opens or closes the dump valve.
     *
     * @param shouldDump true to open the valve (release vacuum), false to close it (hold vacuum)
     */
    public void purge(boolean shouldDump) {
        pdh.setSwitchableChannel(shouldDump);
    }

    /**
     * Set the state of the vacuum subsystem.
     *
     * <p>This method is only used to record state changes. Calling this fuction will not cause the
     * vacuum to start or stop.
     *
     * @param state true to enable, false to disable
     */
    public void setState(boolean state) {
        this.state = state;
    }

    /**
     * Get the state of the vacuum subsystem.
     *
     * @return true if the vacuum is enabled, false if it is disabled
     */
    public boolean getState() {
        return state;
    }
}
