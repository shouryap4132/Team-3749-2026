package frc.robot.subsystems.ShooterHood.sim;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.config.HoodedShooterConfig;
import frc.robot.config.RobotConfig;
import frc.robot.subsystems.ShooterHood.ShooterHoodIO;
import frc.robot.utils.MiscUtils;

/**
 * Simulation implementation of the HoodedShooter IO.
 *
 * Responsibilities:
 * - Simulate a single-jointed arm representing the hood using WPILib's
 * SingleJointedArmSim.
 * - Provide angle, velocity, and current draw readings compatible with the real
 * IO.
 *
 * Notes:
 * - setMotorIdleMode(...) has no effect in this simulation; call is retained
 * for API compatibility.
 * - updateData() steps the sim by the nominal loop time and updates the shared
 * data structure.
 */
public class ShooterHoodSim implements ShooterHoodIO {
    // this has a new MOI parameter, just go ask build for it. It basically
    // describes how hard an arm is to rotate
    private final SingleJointedArmSim hoodedShooterSim = new SingleJointedArmSim(DCMotor.getNEO(1),
            HoodedShooterConfig.HoodedShooterSpecs.GEARING,
            HoodedShooterConfig.HoodedShooterSpecs.MOMENT_OF_INERTIA_KG_M2,
            HoodedShooterConfig.HoodedShooterSpecs.HOODED_SHOOTER_LENGTH.in(Meters),
            HoodedShooterConfig.HoodedShooterSpecs.MIN_ANGLE.getRadians(),
            HoodedShooterConfig.HoodedShooterSpecs.MAX_ANGLE.getRadians(),
            HoodedShooterConfig.HoodedShooterSpecs.SIMULATE_GRAVITY,
            HoodedShooterConfig.HoodedShooterSpecs.START_ANGLE.getRadians());

    private ShooterHoodData data;

    static double prevUpdateS = 0;

    public ShooterHoodSim(ShooterHoodData hoodedShooterData) {
        data = hoodedShooterData;
    }

    @Override
    public void setVoltage(double volts) {
        // Input voltage is clamped to realistic range and applied to the sim.
        double clampedVolts = MiscUtils.voltageClamp(volts);
        hoodedShooterSim.setInputVoltage(clampedVolts);
    }

    @Override
    public void setMotorIdleMode(IdleMode idleMode) {
        // Idle mode is not simulated; this is a no-op for compatibility.
        System.out.println("[HoodedShooterSim] setMotorIdleMode called with " + idleMode
                + ", but this has no effect in simulation.");
    }

    @Override
    public void updateData() {
        // Step simulation and update data.angle/velocity/current accordingly.
        hoodedShooterSim.update(RobotConfig.General.NOMINAL_LOOP_TIME_S);

        // when creating Rotation2ds, always use fromRadians or fromDegrees to be
        // explicit
        data.angle = Rotation2d.fromRadians(hoodedShooterSim.getAngleRads());
        data.velocity = RadiansPerSecond.of(hoodedShooterSim.getVelocityRadPerSec());

        // The following data is not useful in simulation:
        data.currentAmps = hoodedShooterSim.getCurrentDrawAmps();
        data.accel = RadiansPerSecondPerSecond.of(0);
        data.appliedVolts = hoodedShooterSim.getInput().get(0, 0);
    }
}