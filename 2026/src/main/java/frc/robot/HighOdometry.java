package frc.robot;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.config.RobotConfig;
import frc.robot.subsystems.swerve.gyro.GyroDataAutoLogged;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.PigeonGyro;
import frc.robot.subsystems.swerve.gyro.GyroIO.GyroData;


public class HighOdometry {

    private final Object odometryLock = new Object(); 

    private Notifier odometryNotifier;

    
    private final GyroDataAutoLogged gyroData = new GyroDataAutoLogged();

    private final GyroIO gyro = new PigeonGyro(gyroData);


    private ChassisSpeeds desiredChassisSpeeds = new ChassisSpeeds();


    public HighOdometry(){
        odometryNotifier = new Notifier(this::updateData);
        odometryNotifier.startPeriodic(RobotConfig.General.HIGH_FREQ_LOOP_TIME_S);
    }

    public void setOdometry(Pose2d pose) {
        synchronized (odometryLock) {
            Rotation2d gyroHeading =
                Rotation2d.fromRadians(gyroData.orientation.getZ());
        }
    }

    public void updateData() {
        synchronized (odometryLock) {
            gyro.updateData();
        }
    }
    
}
