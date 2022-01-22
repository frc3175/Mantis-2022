package com.team3175.frc2022.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team3175.frc2022.lib.math.Conversions;
import com.team3175.frc2022.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {

    private TalonFX m_feederFalcon;
    //private SubsystemManager m_manager;

    public Feeder() {

        /*m_manager = new SubsystemManager(m_feederFalcon, 
                                         CTREConfigs.feederFXConfig, 
                                         Constants.INVERT_FEEDER, 
                                         Constants.FEEDER_NEUTRAL_MODE, 
                                         "feeder"); */

        m_feederFalcon = new TalonFX(Constants.FEEDER_FALCON);
        
    }

    public void feederRun(double rpm) {
        double falcon_units_100_ms = Conversions.RPMToFalcon(rpm, 1.0);
        m_feederFalcon.set(ControlMode.Velocity, falcon_units_100_ms);
    }

    public void resetEncoders() {
        m_feederFalcon.setSelectedSensorPosition(0);
    }

    public double getEncoderPos() {
        return m_feederFalcon.getSelectedSensorPosition();
    }

    /*public SubsystemManager getManager() {
        return m_manager;
    } */

    
    
}
