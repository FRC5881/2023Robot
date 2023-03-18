package org.tvhsfrc.frc2023.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.tvhsfrc.frc2023.robot.Constants;
import org.tvhsfrc.frc2023.robot.Robot;
import org.tvhsfrc.frc2023.robot.commands.VacuumDisableCommand;
import org.tvhsfrc.frc2023.robot.commands.VacuumEnableCommand;

public class VacuumSubsystem extends SubsystemBase {
    private final PowerDistribution pdh;
    private final SimDeviceSim pdhSim = new SimDeviceSim("Power Distribution Switch");

    private final CANSparkMax vacuum1 =
            new CANSparkMax(
                    Constants.CANConstants.VACUUM_ONE, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax vacuum2 =
            new CANSparkMax(
                    Constants.CANConstants.VACUUM_TWO, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final CANSparkMax vacuum3 =
            new CANSparkMax(
                    Constants.CANConstants.VACUUM_THREE, CANSparkMaxLowLevel.MotorType.kBrushless);

    private final SimDeviceSim motorsSim = new SimDeviceSim("Vacuum Motors");

    /** state of the subsystem: true if we're running the vacuum, false if we're not */
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

        tab.add("cToggle", cToggle());
        tab.add(new VacuumEnableCommand(this));
        tab.add(new VacuumDisableCommand(this, Constants.Vacuum.DUMP_TIME));

        tab.add(this);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("Vacuum state", () -> state, null);

        if (Robot.isReal()) {
            builder.addBooleanProperty(
                    "PDH switchable channel", () -> pdh.getSwitchableChannel(), null);
            builder.addDoubleProperty("Applied Output", () -> vacuum1.getAppliedOutput(), null);
        } else {
            builder.addBooleanProperty(
                    "PDH switchable channel", () -> pdhSim.getBoolean("Value").get(), null);
            builder.addDoubleProperty(
                    "Applied Output", () -> motorsSim.getDouble("Applied Output").get(), null);
        }
    }

    /**
     * Run or stop the vacuum motors.
     *
     * @param shouldSuck
     */
    public void suck(boolean shouldSuck) {
        if (Robot.isReal()) {
            if (shouldSuck) {
                vacuum1.getPIDController()
                        .setReference(
                                Constants.Vacuum.VACUUM_VELOCITY,
                                CANSparkMax.ControlType.kVelocity);
            } else {
                vacuum1.stopMotor();
            }
        } else {
            motorsSim.getDouble("Applied Output").set(shouldSuck ? 1 : 0);
        }
    }

    /**
     * Opens or closes the dump valve.
     *
     * @param shouldDump true to open the valve, false to close it
     */
    public void dump(boolean shouldDump) {
        if (Robot.isReal()) {
            pdh.setSwitchableChannel(shouldDump);
        } else {
            pdhSim.getBoolean("Value").set(shouldDump);
        }
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

    /**
     * Creates a command that toggles the vacuum subsystem.
     *
     * @see VacuumEnableCommand
     * @see VacuumDisableCommand
     */
    public CommandBase cToggle() {
        return Commands.either(
                new VacuumDisableCommand(this, Constants.Vacuum.DUMP_TIME),
                new VacuumEnableCommand(this),
                () -> state);
    }
}
