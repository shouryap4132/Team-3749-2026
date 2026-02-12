package frc.robot.subsystems.roller.real;

import java.util.ArrayList;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.config.RobotConfig;
import frc.robot.config.RollerConfig.RollerImplementations;
import frc.robot.config.RollerConfig.RollerConfigBase;
import frc.robot.subsystems.roller.RollerIO;
import frc.robot.utils.MiscUtils;
import frc.robot.utils.OptixSpark;

/**
 * Implementation of RollerIO for multiple SparkMax motors
 * 
 * @author Anish Gupta
 * @author Rigved Gaddam
 * @author Seonyoo Pak
 */
public class RollerReal implements RollerIO {

    // ArrayList of all the OptixSpark motors to control
    private ArrayList<OptixSpark> motors = new ArrayList<>();

    /**
     * Constructor to make a new RollerReal object
     * 
     * @param implementation The implementation of the roller to construct
     */
    public RollerReal(RollerImplementations implementation) {

        RollerConfigBase config = implementation.config;

        for (int i = 0; i < config.canIds.length; i++) {
            int canId = config.canIds[i];
            OptixSpark motor = OptixSpark.ofSparkMax(canId);
            motor.setPositionConversionFactor(2 * Math.PI / config.gearRatio);
            motor.setVelocityConversionFactor((2 * Math.PI / config.gearRatio) / 60.0);
            motor.setInverted(config.inverted);

            // Follow the first motor in the list if it exists
            if (i > 0) {
                motor.follow(motors.get(0));
            }

            motor.apply();

            // add it to the motors arraylist
            motors.add(motor);
        }
    }

    /**
     * Update the RollerData of the roller motors
     * 
     * @param data RollerData object to update
     */
    @Override
    public void updateData(RollerData data) {

        if (!data.isInitialized) {
            data.initArrays(motors.size());
        }

        for (int i = 0; i < motors.size(); i++) {

            data.rollerAppliedVolts.set(i, motors.get(i).getAppliedVolts());
            double previousVelocity = data.rollerVelocityRadPerSec.get(i);
            data.rollerVelocityRadPerSec.set(i, motors.get(i).getVelocity());
            data.rollerTempCelsius.set(i, motors.get(i).getTemperature());
            data.currentAmps.set(i, motors.get(i).getCurrent());
            data.rollerPositionRad.set(i, motors.get(i).getPosition());
            data.acceleration.set(i,
                    (data.rollerVelocityRadPerSec.get(i) - previousVelocity) / RobotConfig.General.NOMINAL_LOOP_TIME_S);

        }

    }

    /**
     * Set voltage to all of the motors in the roller group
     * 
     * @param volts The amount of volts to send
     */
    @Override
    public void setVoltage(double volts) {
        motors.get(0).setVoltage(MiscUtils.voltageClamp(volts));
    }

    /**
     * Set what the motors do in idle
     * 
     * @param idleMode kCoast or kBrake
     */
    @Override
    public void setMotorIdleMode(IdleMode idleMode) {
        for (OptixSpark motor : motors) {
                motor.setIdleMode(idleMode);
                motor.apply();
        }
    }
}