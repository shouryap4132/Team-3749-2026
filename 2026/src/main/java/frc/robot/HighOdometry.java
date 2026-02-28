package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.config.RobotConfig;
import frc.robot.subsystems.swerve.gyro.GyroDataAutoLogged;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.PigeonGyro;
import org.littletonrobotics.junction.Logger;

public class HighOdometry {

    private final Object odometryLock = new Object();

    private final Notifier odometryNotifier;

    private final GyroDataAutoLogged gyroData = new GyroDataAutoLogged();
    private final GyroIO gyro = new PigeonGyro(gyroData);

    private Rotation2d heading = new Rotation2d();
    private double yawRad;
    private double pitchRad;
    private double rollRad;

    public HighOdometry() {
        odometryNotifier = new Notifier(this::updateData);
        odometryNotifier.startPeriodic(
            RobotConfig.General.HIGH_FREQ_LOOP_TIME_S
        );
    }

    /** Runs on the Notifier thread */
    private void updateData() {
        synchronized (odometryLock) {
            gyro.updateData();

            yawRad   = gyroData.orientation.getZ();
            pitchRad = gyroData.orientation.getY();
            rollRad  = gyroData.orientation.getX();

            heading = Rotation2d.fromRadians(yawRad);
        }
    }

    /** Called from Robot.java (safe, no hardware access) */
    public void periodic() {
        synchronized (odometryLock) {
            Logger.processInputs("HighOdometry/Gyro", gyroData);
        }
    }

    public Rotation2d getHeading() {
        synchronized (odometryLock) {
            return heading;
        }
    }

    public double getYawRad() {
        synchronized (odometryLock) {
            return yawRad;
        }
    }

    public double getPitchRad() {
        synchronized (odometryLock) {
            return pitchRad;
        }
    }

    public double getRollRad() {
        synchronized (odometryLock) {
            return rollRad;
        }
    }

    public void resetGyro() {
        synchronized (odometryLock) {
            gyro.reset();
        }
    }

    public void log(){
        Logger.processInputs("Swerve/GyroData", gyroData);
    }
}