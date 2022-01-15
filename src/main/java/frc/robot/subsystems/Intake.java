package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    
    private final TalonFX m_intakeFalcon = new TalonFX(Constants.INTAKE_FALCON);

    public void intakeCargo(double power) {
        if(power > 0) {
            m_intakeFalcon.setInverted(Constants.INVERT_INTAKE);
            m_intakeFalcon.set(ControlMode.PercentOutput, power);
        } else if(power < 0) {
            m_intakeFalcon.setInverted(Constants.RE_INVERT_INTAKE);
            m_intakeFalcon.set(ControlMode.PercentOutput, power);
        } else {
            m_intakeFalcon.set(ControlMode.PercentOutput, 0);
        }   
    }

}
