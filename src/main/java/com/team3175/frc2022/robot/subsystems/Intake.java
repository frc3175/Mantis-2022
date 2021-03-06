package com.team3175.frc2022.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team3175.frc2022.robot.CTREConfigs;
import com.team3175.frc2022.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    
    private final TalonFX m_intakeFalcon = new TalonFX(Constants.INTAKE_FALCON);

    /**
     * 
     * Sets intake speed in PercentOutput mode, takes negative or positive values
     * 
     * @param power PercentOutput at which the intake motor should run, negative if should be reversed
     * 
     */

    public void intakeCargo(double power) {
        double absolutePower = Math.abs(power);
        if(power > 0) {
            m_intakeFalcon.setInverted(Constants.INVERT_INTAKE);
            m_intakeFalcon.set(ControlMode.PercentOutput, absolutePower);
        } else if(power < 0) {
            m_intakeFalcon.setInverted(Constants.RE_INVERT_INTAKE);
            m_intakeFalcon.set(ControlMode.PercentOutput, absolutePower);
        } else {
            m_intakeFalcon.set(ControlMode.PercentOutput, 0);
        }
    }

    /**
     * 
     * Resets falcon integrated encoder to absolute
     * 
     */

    public void resetEncoders() {
        m_intakeFalcon.setSelectedSensorPosition(0);
    }

    /**
     * 
     * @return Absolute position of falcon integrated encoder
     * 
     */

    public double getEncoders() {
        return m_intakeFalcon.getSelectedSensorPosition();
    }

    /**
     * 
     * Sets intake PercentOutput to 0
     * 
     */

    public void stopIntake() {
        m_intakeFalcon.set(ControlMode.PercentOutput, 0);
    }

    /**
     * 
     * @return if the intake velocity is above 0
     * 
     */

    public boolean isIntakeRunning() {
        if(m_intakeFalcon.getSelectedSensorVelocity() > 0) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * 
     * @return falcon temperature
     * 
     */

    public double getTemp() {
        return m_intakeFalcon.getTemperature();
    }

    /**
     * 
     * @return current draw of the falcon (amps)
     * 
     */

    public double getCurrent() {
        return m_intakeFalcon.getSupplyCurrent();
    }

    /**
     * 
     * @return if the falcon is drawing any voltage
     * 
     */

    public boolean isAlive() {
        return (m_intakeFalcon.getBusVoltage() != 0.0);
    }

    /**
     * 
     * @return velocity of the motor in native falcon units
     * 
     */

    public double getVelocity() {
        return m_intakeFalcon.getSelectedSensorVelocity();
    }

    /**
     * 
     * Configure the intake motor to the config set in CTREConfigs
     * 
     */

    public void configIntakeMotor() {
        m_intakeFalcon.configFactoryDefault();
        m_intakeFalcon.configAllSettings(CTREConfigs.intakeFXConfig);
        m_intakeFalcon.setInverted(Constants.INVERT_INTAKE);
        m_intakeFalcon.setNeutralMode(Constants.INTAKE_NEUTRAL_MODE);
        /*m_intakeFalcon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10000);
        m_intakeFalcon.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 9000);
        m_intakeFalcon.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 14000); */
    }


}
