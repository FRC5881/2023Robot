package org.tvhsfrc.frc2023.robot.Utils;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.interfaces.Gyro;

/**
 * GyroProxy is used to treat our NavX gyro as 3 separate gyros. This is useful for sending data to
 * the SmartDashboard.
 *
 * <p>Most of the methods do nothing, only getAngle() is implemented.
 */
public class GyroProxy implements Gyro, Sendable {
    // enum
    public enum Axis {
        YAW,
        PITCH,
        ROLL
    }

    private final AHRS navx;
    private final Axis axis;

    public GyroProxy(AHRS navx, Axis axis) {
        this.navx = navx;
        this.axis = axis;
    }

    @Override
    public void calibrate() {}

    @Override
    public double getAngle() {
        switch (axis) {
            case YAW:
                return navx.getYaw();
            case PITCH:
                return navx.getPitch();
            case ROLL:
                return navx.getRoll();
            default:
                return 0;
        }
    }

    @Override
    public double getRate() {
        return 0;
    }

    @Override
    public void reset() {}

    @Override
    public void close() throws Exception {}

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Gyro");
        builder.addDoubleProperty("Value", this::getAngle, null);
    }
}
