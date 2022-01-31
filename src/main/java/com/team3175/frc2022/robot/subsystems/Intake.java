package com.team3175.frc2022.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team3175.frc2022.robot.CTREConfigs;
import com.team3175.frc2022.robot.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    
    private final TalonFX m_intakeFalcon = new TalonFX(Constants.INTAKE_FALCON);

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

    public void configIntakeMotor() {
        m_intakeFalcon.configFactoryDefault();
        m_intakeFalcon.configAllSettings(CTREConfigs.intakeFXConfig);
        m_intakeFalcon.setInverted(Constants.INVERT_INTAKE);
        m_intakeFalcon.setNeutralMode(Constants.INTAKE_NEUTRAL_MODE);
    }

}
