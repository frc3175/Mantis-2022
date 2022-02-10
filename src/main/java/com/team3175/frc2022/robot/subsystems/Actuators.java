package com.team3175.frc2022.robot.subsystems;

import com.team3175.frc2022.robot.Constants;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Actuators extends SubsystemBase{

    DoubleSolenoid m_actuators = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.ACTUATORS_LEFT, Constants.ACTUATORS_RIGHT);

    private boolean isDown = false;

    /**
     * 
     * Puts intake into down position
     * 
     */

    public void actuate(){

        m_actuators.set(Value.kReverse);
        isDown = true;


    }

    /**
     * 
     * Puts intake into up position
     * 
     */

    public void actuateBack() {

        m_actuators.set(Value.kForward);
        isDown = false;

    }

    /**
     * 
     * @return if the intake is down
     * 
     */
    public boolean isIntakeDown() {
        return isDown;
    }

} 
