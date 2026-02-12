package frc.robot.subsystems.ShooterHood;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularAcceleration;

/**
 * IO contract for the hooded shooter mechanism.
 *
 * <p>Provides a data snapshot type and methods the subsystem uses to command hardware
 * and refresh sensor readings. Implementations handle the platform-specific details
 * (real hardware or simulation).
 */
public interface ShooterHoodIO {
    /**
     * Snapshot of hood sensor and electrical readings populated by {@link #updateData()}.
     */
    @AutoLog
    public class ShooterHoodData {
        public Rotation2d angle = Rotation2d.fromDegrees(0);
        public AngularVelocity velocity = RadiansPerSecond.of(0.0);
        public AngularAcceleration accel = RadiansPerSecondPerSecond.of(0.0);
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    /**
     * Command the motor with a target voltage (in volts).
     *
     * @param volts voltage to apply to the hood motor
     */
    public void setVoltage(double volts);

    /**
     * Set the motor controller idle mode (e.g., coast or brake).
     *
     * @param idleMode the desired idle mode for the motor controller
     */
    public void setMotorIdleMode(IdleMode idleMode);

    /**
     * Refresh sensor readings and update the {@link ShooterHoodData} instance.
     * Implementations should populate angle, velocity, accel, appliedVolts, and currentAmps.
     */
    public void updateData();
}