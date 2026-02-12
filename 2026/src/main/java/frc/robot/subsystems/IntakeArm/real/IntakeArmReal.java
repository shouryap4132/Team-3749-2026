package frc.robot.subsystems.IntakeArm.real;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.*;

import edu.wpi.first.math.geometry.Rotation2d;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.config.IntakeArmConfig.IntakeArmSpecs;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.CurrentLimits;
import frc.robot.subsystems.IntakeArm.IntakeArmIO;
import frc.robot.subsystems.IntakeArm.IntakeArmDataAutoLogged;
import frc.robot.utils.MiscUtils;
import frc.robot.utils.OptixSpark;

/**
 * Implementation of IntakeArmIO for a real intake arm
 * 
 * @author Anish Gupta
 * @author Rigved Gaddam
 * @author Seonyoo Pak
 */
public class IntakeArmReal implements IntakeArmIO {
    private IntakeArmDataAutoLogged data;

    private final OptixSpark intakeMotor;
    private final AbsoluteEncoder absoluteEncoder;

    static double prevUpdateS = 0;

    /**
     * Constructor to make a new real intake arm
     * 
     * @param armData The IntakeArmData object to log data to
     */
    public IntakeArmReal(IntakeArmDataAutoLogged armData) {
        data = armData;

        intakeMotor = OptixSpark.ofSparkMax(RobotConfig.CAN.INTAKE_ARM_MOTOR_ID);
        intakeMotor
                .setPositionConversionFactor((1.0 / IntakeArmSpecs.GEARING) * 2.0 * Math.PI)
                .setSmartCurrentLimit(CurrentLimits.DEFAULT_MED)
                .setIdleMode(IdleMode.kBrake)
                .setInverted(IntakeArmSpecs.IS_INVERTED);

        intakeMotor.apply();

        absoluteEncoder = intakeMotor.getAbsoluteEncoder();
        Angle offset = Rotations.of(absoluteEncoder.getPosition()).minus(IntakeArmSpecs.ABSOLUTE_ENCODER_OFFSET);

        intakeMotor.setPosition(offset.in(Radians));
    }

    /**
     * Set the voltage to the intake arm motor
     * 
     * @param volts The number of volts to set
     */
    @Override
    public void setVoltage(double volts) {
        intakeMotor.setVoltage(MiscUtils.voltageClamp(volts));
    }

    /**
     * Set what the motor does in idle
     * 
     * @param idleMode kCoast or kBrake
     */
    @Override
    public void setMotorIdleMode(IdleMode idleMode) {
        intakeMotor.setIdleMode(idleMode);
    }

    /**
     * Update the intake arm data
     */
    @Override
    public void updateData() {
        double currentTimeS = Timer.getTimestamp();
        double deltaTimeS = prevUpdateS == 0 ? 0.02 : currentTimeS - prevUpdateS;

        data.angle = Radians.of(intakeMotor.getPosition());

        AngularVelocity prevVelocity = data.velocity.copy();
        data.velocity = RadiansPerSecond.of(intakeMotor.getVelocity());
        data.accel = data.velocity.minus(prevVelocity).div(Seconds.of(deltaTimeS));

        data.currentAmps = intakeMotor.getCurrent();
        data.appliedVolts = intakeMotor.getAppliedVolts();
        
        prevUpdateS = currentTimeS;
    }

}
