package com.team3175.frc2022.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team3175.frc2022.robot.CTREConfigs;
import com.team3175.frc2022.robot.Constants;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

    private final TalonFX m_climberFalcon = new TalonFX(Constants.CLIMBER_FALCON);
    private boolean isDone = false;
    private boolean isLocked = true;

    private final DoubleSolenoid m_solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.CLIMBER_SOLENOID_LEFT, Constants.CLIMBER_SOLENOID_RIGHT);

    public Climber() {
        configureClimberMotor();
        unlockPneumatics();
        //lockPneumatics();
    }

    /**
     * 
     * Automatically brings climbers up
     * 
     * @param setpoint Final encoder position when the climbers are up
     * @param speed PercentOutput to climb at
     * 
     */

    public void climbUp(double setpoint, double speed) {
        m_climberFalcon.setInverted(Constants.INVERT_CLIMBER);
        unlockPneumatics();
        if(getClimberEncoder() < setpoint) {
            m_climberFalcon.set(ControlMode.PercentOutput, speed);
        } else {
            m_climberFalcon.set(ControlMode.PercentOutput, 0);
        }
    }

    /**
     * 
     * Automatically pulls the robot up by pulling climbers down
     * 
     * @param setpoint Final encoder position when the climbers are down
     * @param speed PercentOutput to climb at
     * 
     */

    public void climbDown(double setpoint, double speed) {
        m_climberFalcon.setInverted(Constants.RE_INVERT_CLIMBER);
        if(getClimberEncoder() < setpoint) {
            m_climberFalcon.set(ControlMode.PercentOutput, speed);
            isDone = false;
        } else {
            overrideStop();
            isDone = true;
        }
    }

    /**
     * 
     * @return if the automated climb is complete or not
     * 
     */

    public boolean isDone() {
        return isDone;
    }

    /**
     * 
     * Override the isDone variable and set if the climb is done or not
     * 
     * @param done whether or not the automated climb is complete
     * 
     */

    public void setDone(boolean done) {
        isDone = done;
    }

    /**
     * 
     * Manually brings the climbers up, only used in emergencies
     * 
     * @param speed PercentOutput to climb
     * 
     */

    public void overrideUp(double speed) {
        m_climberFalcon.setInverted(Constants.INVERT_CLIMBER);
        m_climberFalcon.set(ControlMode.PercentOutput, speed);
    }

    /**
     * 
     * Manually brings the climbers down, only used in emergencies
     * 
     * @param speed PercentOutput to climb
     * 
     */

    public void overrideDown(double speed) {
        m_climberFalcon.setInverted(Constants.RE_INVERT_CLIMBER);
        m_climberFalcon.set(ControlMode.PercentOutput, speed);
    }

    /**
     * 
     * Estop the climber
     * 
     */

    public void overrideStop() {
        m_climberFalcon.set(ControlMode.PercentOutput, 0);
    }

    /**
     * 
     * @return Position of the climber encoder
     * 
     */

    public double getClimberEncoder() {
        return m_climberFalcon.getSelectedSensorPosition();
    }

    /**
     * 
     * Set climber integrated encoder to 0
     * 
     */

    public void resetEncoders() {
        m_climberFalcon.setSelectedSensorPosition(0);
    }

    /**
     * 
     * @return falcon temperature
     * 
     */
    public double getTemp() {
        return m_climberFalcon.getTemperature();
    }

    /**
     * 
     * @return current draw of the falcon (amps)
     * 
     */
    public double getCurrent() {
        return m_climberFalcon.getSupplyCurrent();
    }

    /**
     * 
     * @return if the falcon is drawing any voltage
     * 
     */
    public boolean isAlive() {
        return (m_climberFalcon.getBusVoltage() != 0.0);
    }

        /**
     * 
     * @return velocity of the motor in native falcon units
     * 
     */
    public double getVelocity() {
        return m_climberFalcon.getSelectedSensorVelocity();
    }

    /**
     * 
     * Set the pneumatic brake to the locked position
     * 
     */

    public void lockPneumatics() {
        m_solenoid.set(Constants.CLIMBER_LOCK);
        isLocked = true;
    }

    /**
     * 
     * Set the pneumatic brake to the unlocked position
     * 
     */

    public void unlockPneumatics() {
       m_solenoid.set(Constants.CLIMBER_UNLOCK);
       isLocked = false;
    }

    /**
     * 
     * Configure the climber motor to the config set in CTREConfigs
     * 
     */

    public void configureClimberMotor() {
        m_climberFalcon.configFactoryDefault();
        m_climberFalcon.configAllSettings(CTREConfigs.climberFXConfig);
        m_climberFalcon.setNeutralMode(Constants.CLIMBER_NEUTRAL_MODE);
        m_climberFalcon.setInverted(Constants.INVERT_CLIMBER);
    }

    public boolean isClimberLocked() {
        return isLocked;
    }
    
}
