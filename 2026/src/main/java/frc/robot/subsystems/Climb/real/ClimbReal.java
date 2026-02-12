// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climb.real;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.config.ClimbConfig.ClimbSpecs;
import frc.robot.config.RobotConfig;
import frc.robot.config.RobotConfig.CurrentLimits;
import frc.robot.subsystems.Climb.ClimbDataAutoLogged;
import frc.robot.subsystems.Climb.ClimbIO;
import frc.robot.utils.MiscUtils;
import frc.robot.utils.OptixSpark;

public class ClimbReal implements ClimbIO {
    private final ClimbDataAutoLogged data;

    private final OptixSpark leftMotor;
    private final OptixSpark rightMotor;

    private double prevUpdateS = 0;

    public ClimbReal(ClimbDataAutoLogged climbData) {
        this.data = climbData;

        // Linear Conversion Math:
        // Position Factor = (1 / Gearing) * 2 * PI * Radius
        double positionConversion = (1.0 / ClimbSpecs.GEARING)
                * (2.0 * Math.PI * ClimbSpecs.DRUM_RADIUS.in(Meters));

        // Velocity Factor = positionConversion / 60 (to convert RPM to Units/Sec)
        double velocityConversion = positionConversion / 60.0;

        // Initialize Left Motor (Leader)
        leftMotor = OptixSpark.ofSparkMax(RobotConfig.CAN.CLIMB_LEFT_ID);
        rightMotor = OptixSpark.ofSparkMax(RobotConfig.CAN.CLIMB_RIGHT_ID);

        leftMotor
                .setPositionConversionFactor(positionConversion)
                .setVelocityConversionFactor(velocityConversion)
                .setSmartCurrentLimit(CurrentLimits.DEFAULT_HIGH)
                .setIdleMode(IdleMode.kBrake)
                .setInverted(ClimbSpecs.LEFT_MOTOR_INVERTED);

        // Initialize Right Motor (Follower)
        rightMotor
                .follow(leftMotor, ClimbSpecs.RIGHT_MOTOR_INVERTED); // Uses the wrapper's built-in follow method

        // Push settings to the SparkMax hardware
        leftMotor.apply();
        rightMotor.apply(leftMotor.getConfig());
    }

    @Override
    public void setVoltage(double volts) {
        // We only set the leader; the hardware handles the follower.
        leftMotor.setVoltage(MiscUtils.voltageClamp(volts));
    }

    @Override
    public void setMotorIdleMode(IdleMode mode) {
        leftMotor.setIdleMode(mode);
        rightMotor.setIdleMode(mode);

        leftMotor.apply();
        rightMotor.apply();
    }

    @Override
    public void updateData() {
        double currentTimeS = Timer.getTimestamp();
        double deltaTimeS = prevUpdateS == 0 ? 0.02 : currentTimeS - prevUpdateS;
        prevUpdateS = currentTimeS;

        // 1. Position: includes the physical vertical offset of the climber mount
        data.height = Meters.of(leftMotor.getPosition());

        // 2. Velocity & Acceleration:
        LinearVelocity prevVelocity = data.velocity.copy();
        data.velocity = MetersPerSecond.of(leftMotor.getVelocity());
        data.accel = data.velocity.minus(prevVelocity).div(Seconds.of(deltaTimeS));

        // 3. Electrical Data:
        data.leftAppliedVolts = leftMotor.getAppliedVolts();
        data.rightAppliedVolts = rightMotor.getAppliedVolts();
        data.leftCurrentAmps = leftMotor.getCurrent();
        data.rightCurrentAmps = rightMotor.getCurrent();
    }
}