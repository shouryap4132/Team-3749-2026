package frc.robot.subsystems.roller;

import java.util.ArrayList;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/**
 * IO for roller subsystem
 * 
 * @author Anish Gupta
 * @author Rigved Gaddam
 * @author Seonyoo Pak
 */

public interface RollerIO {
    public static class RollerData {
        public boolean isInitialized = false;
        public int numMotors;
        public ArrayList<Double> rollerAppliedVolts = new ArrayList<Double>();
        public ArrayList<Double> rollerVelocityRadPerSec = new ArrayList<Double>();
        public ArrayList<Double> rollerTempCelsius = new ArrayList<Double>();
        public ArrayList<Double> currentAmps = new ArrayList<Double>();
        public ArrayList<Double> rollerPositionRad = new ArrayList<Double>();
        public ArrayList<Double> acceleration = new ArrayList<Double>();

        /**
         * Initialize the arrays in RollerData
         * 
         * @param numMotors The number of motors
         */
        public void initArrays(int numMotors) {
            if (!isInitialized){
                this.numMotors = numMotors;
                for (int i = 0; i < numMotors; i++) {
                    rollerAppliedVolts.add(0.0);
                    rollerVelocityRadPerSec.add(0.0);
                    rollerTempCelsius.add(0.0);
                    currentAmps.add(0.0);
                    rollerPositionRad.add(0.0);
                    acceleration.add(0.0);
                }
                this.isInitialized = true;
            }
        }
    }

    public default void updateData(RollerData data) {

    }

    public default void setVoltage(double rollerVolts) {

    }

    public default void setMotorIdleMode(IdleMode idleMode) {

    };

    public default void updateSimulation(double dtseconds) {

    }
}