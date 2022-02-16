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

        configShooterFalcons();

        m_leftShooterFalcon.follow(m_rightShooterFalcon);

    }

    /**
     * 
     * Spins up the shooter and pushes setpoint to SmartDashboard
     * 
     * @param rpm Setpoint of the shooter in revolutions per minute of the falcon
     * 
     */

    public void shoot(double rpm) {

        double target_velocity_units_100ms = Conversions.RPMToFalcon(rpm, 1.0);
        m_rightShooterFalcon.set(TalonFXControlMode.Velocity, target_velocity_units_100ms);
        m_leftShooterFalcon.set(TalonFXControlMode.Velocity, target_velocity_units_100ms);

        SmartDashboard.putNumber("shooter setpoint falcon units*", target_velocity_units_100ms);
        SmartDashboard.putNumber("shooter setpoint rpm*", Conversions.falconToRPM(target_velocity_units_100ms, 1.0));

    }

    /**
     * 
     * Stops the shooter by setting it to 0 Percent Output
     * 
     */

    public void stopShooter() {
        m_rightShooterFalcon.set(ControlMode.PercentOutput, 0);
        m_leftShooterFalcon.set(ControlMode.PercentOutput, 0);
    }

    /**
     * 
     * @return Velocity of the left falcon in RPM
     * 
     */

    public double getLeftVelocityRPM() {
        double falconUnits = m_leftShooterFalcon.getSelectedSensorVelocity();
        double rpm = Conversions.falconToRPM(falconUnits, 1.0);
        return rpm;
    }

    /**
     * 
     * @return Velocity of the right falcon in RPM
     * 
     */

    public double getRightVelocityRPM() {
        double falconUnits = m_rightShooterFalcon.getSelectedSensorVelocity();
        double rpm = Conversions.falconToRPM(falconUnits, 1.0);
        return rpm;
    }

    /**
     * 
     * @return Velocity of the left shooter falcon in falcon units per 100 ms
     * 
     */

    public double getLeftVelocityFalcon() {
        return m_leftShooterFalcon.getSelectedSensorVelocity();
    }

    /**
     * 
     * @return Velocity of the right shooter falcon in falcon units per 100 ms
     * 
     */

    public double getRightVelocityFalcon() {
        return m_rightShooterFalcon.getSelectedSensorVelocity();
    }

    /**
     * 
     * Sets the current position of both encoders to 0
     * 
     */

    public void resetEncoders() {
        m_leftShooterFalcon.setSelectedSensorPosition(0);
        m_rightShooterFalcon.setSelectedSensorPosition(0);
    }

    /**
     * 
     * Returns true if the right falcon is at the goal setpoint
     * 
     * @param rpm Setpoint of the motor in RPM
     * @return if the falcon is within 10 RPM of the setpoint
     * 
     */

    public boolean rightFalconAtSetpoint(double rpm) {
        boolean isAtSetpoint = (rpm - getRightVelocityRPM()) < Constants.SHOOTER_ERROR ? true : false;
        return isAtSetpoint;
    }

    /**
     * 
     * Returns true if the left falcon is at the goal setpoint
     * 
     * @param rpm Setpoint of the motor in RPM
     * @return if the falcon is within 10 RPM of the setpoint
     * 
     */

    public boolean leftFalconAtSetpoint(double rpm) {
        boolean isAtSetpoint = (rpm - getLeftVelocityRPM()) < Constants.SHOOTER_ERROR ? true : false;
        return isAtSetpoint;
    }

    /**
     * 
     * Gets an average of the left and right encoders
     * 
     * @return left and right encoder positions averaged together
     * 
     */

    public double getEncoders() {
        double left = m_leftShooterFalcon.getSelectedSensorPosition();
        double right = m_rightShooterFalcon.getSelectedSensorPosition();
        double total = (left + right);
        return total / 2;
    }

    /**
     * 
     * @return left encoder pos
     * 
     */
    public double getLeftEncoder() {
        return m_leftShooterFalcon.getSelectedSensorPosition();
    }

    /**
     * 
     * @return right encoder pos
     * 
     */
    public double getRightEncoder() {
        return m_rightShooterFalcon.getSelectedSensorPosition();
    }

    /**
     * 
     * @return falcon temperature
     * 
     */
    public double getLeftTemp() {
        return m_leftShooterFalcon.getTemperature();
    }

    /**
     * 
     * @return falcon temperature
     * 
     */
    public double getRightTemp() {
        return m_rightShooterFalcon.getTemperature();
    }

    /**
     * 
     * @return current draw of the falcon (amps)
     * 
     */
    public double getLeftCurrent() {
        return m_leftShooterFalcon.getSupplyCurrent();
    }

    /**
     * 
     * @return current draw of the falcon (amps)
     * 
     */
    public double getRightCurrent() {
        return m_rightShooterFalcon.getSupplyCurrent();
    }

    /**
     * 
     * @return if the falcon is drawing any voltage
     * 
     */
    public boolean isLeftAlive() {
        return (m_leftShooterFalcon.getBusVoltage() != 0.0);
    }

    /**
     * 
     * @return if the falcon is drawing any voltage
     * 
     */
    public boolean isRightAlive() {
        return (m_rightShooterFalcon.getBusVoltage() != 0.0);
    }

    /**
     * 
     * Sets both shooter falcons to the configuration set in CTREConfigs.java
     * 
     */
    
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
