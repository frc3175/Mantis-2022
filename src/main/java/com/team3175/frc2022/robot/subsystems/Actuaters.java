package com.team3175.frc2022.robot.subsystems;

import com.team3175.frc2022.robot.Constants;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Actuaters extends SubsystemBase{

    DoubleSolenoid m_actuaters = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.ACTUATERS_LEFT, Constants.ACTUATERS_RIGHT);

    public void actuate(){

        m_actuaters.toggle();
    }

}
