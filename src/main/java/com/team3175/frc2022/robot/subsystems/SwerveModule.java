package com.team3175.frc2022.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

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

    /**
     * 
     * The constructor for each swerve module on the robot
     * 
     * @param moduleNumber FL -> 0 FR -> 1 BL -> 2 BR -> 3
     * @param offset Cancoder offset for the module
     * @param azimuthMotor Azimuth motor CAN ID
     * @param driveMotor Drive motor CAN ID
     * @param canCoder CANCoder CAN ID
     * @param azimuthInverted Is the azimuth motor inverted in the forward direction
     * @param driveInverted Is the drive motor inverted in the forward direction
     * @param canCoderInverted Is the CANCoder inverted in the forward direction
     * 
     */

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

    /**
     * 
     * Set the state of the individual module to the desired position 
     * 
     * @param desiredState the SwerveModuleState for the individual module to go to
     * @param openLoop should almost always be false, whether driving is open loop or not
     * 
     */

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

    /**
     * 
     * Sets the azimuth integrated encoder to absolute based on the current CANCoder position
     * 
     */

    private void resetToAbsolute() {
        m_azimuthMotor.setSelectedSensorPosition(Conversions.degreesToFalcon(getCanCoder().getDegrees() - m_offset, Constants.AZIMUTH_GEAR_RATIO));
    }

    /**
     * 
     * Configures the CANCoder to the configuration set in CTREConfigs.java
     * 
     */

    private void configCanCoder() {        
        m_canCoder.configFactoryDefault();
        m_canCoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
        m_canCoder.configSensorDirection(m_canCoderInverted);
    }

    /**
     * 
     * Configures the Azimuth motor to the configuration set in CTREConfigs.java
     * 
     */

    private void configTurningMotor() {
        m_azimuthMotor.configFactoryDefault();
        m_azimuthMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        m_azimuthMotor.setInverted(m_turningInverted);
        m_azimuthMotor.setNeutralMode(Constants.AZIMUTH_NEUTRAL_MODE);
        resetToAbsolute();
    }

    /**
     * 
     * Configures the Drive motor to the configuration set in CTREConfigs.java
     * 
     */

    private void configDriveMotor() {        
        m_driveMotor.configFactoryDefault();
        m_driveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        m_driveMotor.setInverted(m_driveInverted);
        m_driveMotor.setNeutralMode(Constants.DRIVE_NEUTRAL_MODE);
        m_driveMotor.setSelectedSensorPosition(0);
    }

    /**
     * 
     * @return Rotation2d CANCoder position
     * 
     */

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(m_canCoder.getAbsolutePosition());
    }

    /**
     * 
     * @return drive motor integrated encoder position
     * 
     */

    public double getDriveEncoder() {
        return m_driveMotor.getSelectedSensorPosition();
    }

    /**
     * 
     * @return SwerveModuleState state of the individual module
     * 
     */

    public SwerveModuleState getState() {

        double velocity = Conversions.falconToMPS(m_driveMotor.getSelectedSensorVelocity(), Constants.WHEEL_CIRCUMFERENCE, Constants.DRIVE_GEAR_RATIO);
        Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(m_azimuthMotor.getSelectedSensorPosition(), Constants.AZIMUTH_GEAR_RATIO));
        return new SwerveModuleState(velocity, angle);

    }

    /**
     * 
     * Sets the position of the azimuth integrated encoder to the zero position of the CANCoder
     * 
     */

    public void zeroModule() {
        m_azimuthMotor.setSelectedSensorPosition(Conversions.degreesToFalcon(m_offset, Constants.AZIMUTH_GEAR_RATIO));
    }


}
