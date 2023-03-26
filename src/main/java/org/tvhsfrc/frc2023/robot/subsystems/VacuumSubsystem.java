package org.tvhsfrc.frc2023.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.tvhsfrc.frc2023.robot.Constants;
import org.tvhsfrc.frc2023.robot.Robot;
import org.tvhsfrc.frc2023.robot.commands.VacuumCommand;

public class VacuumSubsystem extends SubsystemBase {
    public enum State {
        VACUUM,
        DUMP
    }

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

    private final Timer dumpTimer = new Timer();

    /** state of the subsystem: true if we're running the vacuum, false if we're not */
    private State state = State.DUMP;

    public VacuumSubsystem(PowerDistribution pdh) {
        this.pdh = pdh;

        /*vacuum1.getPIDController().setP(0.00005);
        vacuum1.getPIDController().setI(0.0);
        vacuum1.getPIDController().setD(0.002);
        vacuum1.getPIDController().setFF(0.0001);
        vacuum1.setIdleMode(CANSparkMax.IdleMode.kCoast);
        vacuum1.getPIDController().setOutputRange(0, Constants.Vacuum.MAX_OUTPUT);
        vacuum1.setClosedLoopRampRate(0.5);*/

        vacuum1.setIdleMode(CANSparkMax.IdleMode.kCoast);
        vacuum2.setIdleMode(CANSparkMax.IdleMode.kCoast);
        vacuum3.setIdleMode(CANSparkMax.IdleMode.kCoast);

        pdh.setSwitchableChannel(false);

        ShuffleboardTab tab = Shuffleboard.getTab("Vacuum");

        tab.add(new VacuumCommand(this));

        tab.add(this);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Vacuum state", () -> state.toString(), null);

        if (Robot.isReal()) {
            builder.addBooleanProperty("PDH switchable channel", pdh::getSwitchableChannel, null);
            builder.addDoubleProperty("Vacuum 1 Applied Output", vacuum1::getAppliedOutput, null);
            builder.addDoubleProperty("Vacuum 2 Applied Output", vacuum2::getAppliedOutput, null);
            builder.addDoubleProperty("Vacuum 3 Applied Output", vacuum3::getAppliedOutput, null);
        } else {
            builder.addBooleanProperty(
                    "PDH switchable channel", () -> pdhSim.getBoolean("Value").get(), null);
            builder.addDoubleProperty(
                    "Applied Output", () -> motorsSim.getDouble("Applied Output").get(), null);
        }
    }

    @Override
    public void periodic() {
        if (state.equals(State.DUMP)
                && dumpTimer.hasElapsed(Constants.Vacuum.DUMP_TIME)
                && pdh.getSwitchableChannel()) {
            pdh.setSwitchableChannel(false);
        }
    }

    @Override
    public void simulationPeriodic() {
        if (state.equals(State.DUMP)
                && dumpTimer.hasElapsed(Constants.Vacuum.DUMP_TIME)
                && pdh.getSwitchableChannel()) {
            pdhSim.getBoolean("Value").set(false);
        }
    }

    /** Run the vacuum motors. */
    public void vacuum() {
        setState(State.VACUUM);
        if (Robot.isReal()) {
            pdh.setSwitchableChannel(false);
            vacuum1.set(Constants.Vacuum.MAX_OUTPUT);
            vacuum2.set(Constants.Vacuum.MAX_OUTPUT);
            vacuum3.set(Constants.Vacuum.MAX_OUTPUT);
        } else {
            pdhSim.getBoolean("Value").set(false);
            motorsSim.getDouble("Applied Output").set(1);
        }
    }

    /** Opens the dump valve and stops the motors. */
    public void dump() {
        setState(State.DUMP);
        if (Robot.isReal()) {
            vacuum1.stopMotor();
            vacuum2.stopMotor();
            vacuum3.stopMotor();
            pdh.setSwitchableChannel(true);
            dumpTimer.restart();
        } else {
            pdhSim.getBoolean("Value").set(true);
            motorsSim.getDouble("Applied Output").set(0);
        }
    }

    /**
     * Set the state of the vacuum subsystem.
     *
     * <p>This method is only used to record state changes. Calling this function will not cause the
     * vacuum to start or stop.
     *
     * @param state State enum of the subsystem
     */
    private void setState(State state) {
        this.state = state;
    }

    /**
     * Get the state of the vacuum subsystem.
     *
     * @return State enum
     */
    public State getState() {
        return state;
    }
}
