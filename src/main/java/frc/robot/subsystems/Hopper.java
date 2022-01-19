package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.CTREConfigs;
import frc.robot.Constants;

public class Hopper extends SubsystemBase {

    private TalonFX m_hopperFalcon = new TalonFX(Constants.HOPPER);

    public Hopper() {
        configHopperMotor();
    }

    public void hopperRun(double rpm) {
        double falcon_units_100_ms = Conversions.RPMToFalcon(rpm, 1.0);
        m_hopperFalcon.set(ControlMode.Velocity, falcon_units_100_ms);
    }

    public void agitateHopper(double rpm) {
        double falcon_units_100_ms = Conversions.RPMToFalcon(rpm, 1.0);
        m_hopperFalcon.set(ControlMode.Velocity, -falcon_units_100_ms);
    }

    public void resetEncoders() {
        m_hopperFalcon.setSelectedSensorPosition(0);
    }

    public double getEncoderPos() {
        return m_hopperFalcon.getSelectedSensorPosition();
    }

    private void configHopperMotor() {
        m_hopperFalcon.configFactoryDefault();
        m_hopperFalcon.configAllSettings(CTREConfigs.hopperFXConfig);
        m_hopperFalcon.setInverted(Constants.INVERT_HOPPER);
        m_hopperFalcon.setNeutralMode(Constants.HOPPER_NEUTRAL_MODE);
    }

    
    
}
