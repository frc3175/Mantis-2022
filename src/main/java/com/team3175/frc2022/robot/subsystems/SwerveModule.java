package com.team3175.frc2022.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.team3175.frc2022.lib.math.Conversions;
import com.team3175.frc2022.lib.util.CTREModuleState;
import com.team3175.frc2022.robot.Constants;
import com.team3175.frc2022.robot.Robot;

public class SwerveModule {
    public int m_moduleNumber;
    private double m_offset;
    private TalonFX m_azimuthMotor;
    private TalonFX m_driveMotor;
    private CANCoder m_canCoder;
    private double m_lastAngle;
    private boolean m_turningInverted;
    private boolean m_driveInverted;
    private boolean m_canCoderInverted;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.DRIVE_S, Constants.DRIVE_V, Constants.DRIVE_A);

    public SwerveModule(int moduleNumber, double offset, int azimuthMotor, int driveMotor, int canCoder, boolean azimuthInverted, boolean driveInverted, boolean canCoderInverted){
        m_moduleNumber = moduleNumber;
        m_offset = offset;
        m_turningInverted = azimuthInverted;
        m_driveInverted = driveInverted;
        m_canCoderInverted = canCoderInverted;

        m_canCoder = new CANCoder(canCoder);
        m_azimuthMotor = new TalonFX(azimuthMotor);
        m_driveMotor = new TalonFX(driveMotor);

        configCanCoder();
        configTurningMotor();
        configDriveMotor();   

        m_lastAngle = getState().angle.getDegrees();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean openLoop) {

        desiredState = CTREModuleState.optimize(desiredState, getState().angle);

        if(openLoop){
            m_driveMotor.set(ControlMode.PercentOutput, desiredState.speedMetersPerSecond / Constants.MAX_SPEED);
        } else {
            m_driveMotor.set(ControlMode.Velocity, 
                             Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.WHEEL_CIRCUMFERENCE, Constants.DRIVE_GEAR_RATIO), 
                             DemandType.ArbitraryFeedForward, 
                             feedforward.calculate(desiredState.speedMetersPerSecond));
        }

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.MAX_SPEED * 0.01)) ? m_lastAngle : desiredState.angle.getDegrees(); //Prevent rotating module if speed is less then 1%. Prevents Jittering.
       
        m_azimuthMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle, Constants.AZIMUTH_GEAR_RATIO));

        m_lastAngle = angle;
    }

    private void resetToAbsolute() {
        m_azimuthMotor.setSelectedSensorPosition(Conversions.degreesToFalcon(getCanCoder().getDegrees() - m_offset, Constants.AZIMUTH_GEAR_RATIO));
    }

    private void configCanCoder() {        
        m_canCoder.configFactoryDefault();
        m_canCoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
        m_canCoder.configSensorDirection(m_canCoderInverted);
    }

    private void configTurningMotor() {
        m_azimuthMotor.configFactoryDefault();
        m_azimuthMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        m_azimuthMotor.setInverted(m_turningInverted);
        m_azimuthMotor.setNeutralMode(Constants.AZIMUTH_NEUTRAL_MODE);
        resetToAbsolute();
    }

    private void configDriveMotor() {        
        m_driveMotor.configFactoryDefault();
        m_driveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        m_driveMotor.setInverted(m_driveInverted);
        m_driveMotor.setNeutralMode(Constants.DRIVE_NEUTRAL_MODE);
        m_driveMotor.setSelectedSensorPosition(0);
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(m_canCoder.getAbsolutePosition());
    }

    public double getDriveEncoder() {
        return m_driveMotor.getSelectedSensorPosition();
    }

    public SwerveModuleState getState() {

        double velocity = Conversions.falconToMPS(m_driveMotor.getSelectedSensorVelocity(), Constants.WHEEL_CIRCUMFERENCE, Constants.DRIVE_GEAR_RATIO);
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(m_azimuthMotor.getSelectedSensorPosition(), Constants.AZIMUTH_GEAR_RATIO));
        return new SwerveModuleState(velocity, angle);

    }

    //not actually using this anymore
    public void keepModuleWhereItIs(int moduleNumber) {
        m_azimuthMotor.setSelectedSensorPosition(Conversions.degreesToFalcon(
                                                 (SmartDashboard.getNumber("Mod " + moduleNumber + " Azimuth angle", 0) - m_offset), 
                                                 Constants.AZIMUTH_GEAR_RATIO));
    }

    public void zeroModule() {
        m_azimuthMotor.setSelectedSensorPosition(Conversions.degreesToFalcon(m_offset, Constants.AZIMUTH_GEAR_RATIO));
    }


}
