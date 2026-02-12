package frc.robot.subsystems.ShooterHood.real;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.MiscUtils;
import frc.robot.utils.OptixSpark;
import frc.robot.config.HoodedShooterConfig.HoodedShooterSpecs;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.CurrentLimits;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.ShooterHood.ShooterHoodIO;

/**
 * Real hardware implementation of the HoodedShooter IO.
 *
 * Responsibilities:
 * - Drive the real motor controller for the hood and read sensors.
 * - Convert motor native units into Rotation2d / angular velocity represented in the shared data object.
 *
 * Important notes:
 * - The constructor currently declares a local OptixSpark that shadows the class field;
 *   this mirrors the existing code but should be cleaned up in the future to avoid confusion.
 */
public class ShooterHoodReal implements ShooterHoodIO {
    
    private ShooterHoodData data;

    OptixSpark hoodedShooterMotor = OptixSpark.ofSparkMax(RobotConfig.CAN.HOODED_SHOOTER_MOTOR_ID);

    public AngularVelocity prevVelocity = RadiansPerSecond.of(0);
    static double prevUpdateS = 0;

    /**
     * Hardware constructor: configure motor conversion factors, current limits, idle mode, and inversion.
     *
     * @param hoodedShooterData shared data object that will be updated each loop
     */
    public ShooterHoodReal(ShooterHoodData hoodedShooterData) {
        data = hoodedShooterData;

                OptixSpark hoodedShooterMotor = OptixSpark.ofSparkMax(RobotConfig.CAN.ARM_MOTOR_ID);
        hoodedShooterMotor
                .setPositionConversionFactor(HoodedShooterSpecs.GEARING * 2.0 * Math.PI)
                .setSmartCurrentLimit(CurrentLimits.DEFAULT_MED)
                .setIdleMode(IdleMode.kBrake)
                .setInverted(HoodedShooterSpecs.IS_INVERTED);

        hoodedShooterMotor.apply();
    }

    /**
     * Send a clamped voltage to the underlying motor controller.
     *
     * @param volts desired voltage (will be clamped for safety)
     */
    @Override
    public void setVoltage(double volts) {
        double clampedVolts = MiscUtils.voltageClamp(volts);

        hoodedShooterMotor.setVoltage(clampedVolts);
    }

    /**
     * Set motor idle mode (brake/coast). Passed to motor hardware.
     *
     * @param idlemode idle mode to set on the motor controller
     */
    @Override
    public void setMotorIdleMode(IdleMode idlemode) {
        hoodedShooterMotor.setIdleMode(idlemode);
    }

    /**
     * Read sensor values from hardware and update the shared data object.
     *
     * This method converts raw motor position/velocity into Rotation2d and angular velocity
     * and computes a simple accel estimate. It also records current and applied volts.
     */
    @Override
    public void updateData() {
        double currentTimeS = Timer.getTimestamp();
        double deltaTimeS = prevUpdateS == 0 ? 0.02 : currentTimeS - prevUpdateS;

        data.angle = Rotation2d.fromRadians(hoodedShooterMotor.getPosition());

        data.velocity = RadiansPerSecond.of(hoodedShooterMotor.getVelocity());
        data.accel = data.velocity.minus(prevVelocity).div(Seconds.of(deltaTimeS));

        data.currentAmps = hoodedShooterMotor.getCurrent();
        data.appliedVolts = hoodedShooterMotor.getAppliedVolts();
    }
}
