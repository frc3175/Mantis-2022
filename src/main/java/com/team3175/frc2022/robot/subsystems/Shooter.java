package com.team3175.frc2022.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.team3175.frc2022.lib.math.Conversions;
import com.team3175.frc2022.robot.CTREConfigs;
import com.team3175.frc2022.robot.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private final TalonFX m_leftShooterFalcon = new TalonFX(Constants.LEFT_SHOOTER_FALCON);
    private final TalonFX m_rightShooterFalcon = new TalonFX(Constants.RIGHT_SHOOTER_FALCON);

    public Shooter() {

        //m_rightShooterFalcon.follow(m_leftShooterFalcon);

        configShooterFalcons();

    }

    public void shoot(double rpm) {

        //m_rightShooterFalcon.set(ControlMode.PercentOutput, 0.5);
        //m_leftShooterFalcon.set(ControlMode.PercentOutput, 0.5);
        SmartDashboard.putNumber("shooter velocity", getLeftVelocity());

        double target_velocity_units_100ms = Conversions.RPMToFalcon(rpm, 1.0);
        m_leftShooterFalcon.set(TalonFXControlMode.Velocity, target_velocity_units_100ms);
        m_rightShooterFalcon.set(TalonFXControlMode.Velocity, target_velocity_units_100ms);

        SmartDashboard.putNumber("shooter", target_velocity_units_100ms);

    }

    public void stopShooter() {
        m_leftShooterFalcon.set(ControlMode.PercentOutput, 0);
        m_rightShooterFalcon.set(ControlMode.PercentOutput, 0);
    }

    public double getLeftVelocity() {
        return m_leftShooterFalcon.getSelectedSensorVelocity();
    }

    public double getRightVelocity() {
        return m_rightShooterFalcon.getSelectedSensorVelocity();
    }

    public void resetEncoders() {
        m_leftShooterFalcon.setSelectedSensorPosition(0);
        m_rightShooterFalcon.setSelectedSensorPosition(0);
    }

    public boolean rightFalconAtSetpoint(double rpm) {
        boolean isAtSetpoint = (rpm - getRightVelocity()) < Constants.SHOOTER_ERROR ? true : false;
        return isAtSetpoint;
    }

    public boolean leftFalconAtSetpoint(double rpm) {
        boolean isAtSetpoint = (rpm - getLeftVelocity()) < Constants.SHOOTER_ERROR ? true : false;
        return isAtSetpoint;
    }
    
    private void configShooterFalcons() {
        m_leftShooterFalcon.configFactoryDefault();
        m_leftShooterFalcon.configAllSettings(CTREConfigs.shooterFXConfig);
        m_leftShooterFalcon.setInverted(Constants.INVERT_LEFT_SHOOTER);
        m_leftShooterFalcon.setNeutralMode(Constants.SHOOTER_NEUTRAL_MODE);
        m_rightShooterFalcon.configFactoryDefault();
        m_rightShooterFalcon.configAllSettings(CTREConfigs.shooterFXConfig);
        m_rightShooterFalcon.setInverted(Constants.INVERT_RIGHT_SHOOTER);
        m_rightShooterFalcon.setNeutralMode(Constants.SHOOTER_NEUTRAL_MODE);
    }

}
