package frc.robot.subsystems.IntakeArm;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/**
 * IO interface for the IntakeArm subsystem
 * 
 * @author Anish Gupta
 * @author Rigved Gaddam
 * @author Seonyoo Pak
 */
public interface IntakeArmIO {
    @AutoLog
    public class IntakeArmData {
        public Angle angle = Degrees.of(0);
        public AngularVelocity velocity = RadiansPerSecond.of(0.0);
        public AngularAcceleration accel = RadiansPerSecondPerSecond.of(0.0);
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
    }

    public void setVoltage(double volts);

    public void setMotorIdleMode(IdleMode idleMode);

    public void updateData();
}
