package com.team3175.frc2022.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

public final class CTREConfigs {

    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;
    public static TalonFXConfiguration feederFXConfig;
    public static TalonFXConfiguration shooterFXConfig;
    public static TalonFXConfiguration climberFXConfig;
    public static TalonFXConfiguration intakeFXConfig;


    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();
        feederFXConfig = new TalonFXConfiguration();
        shooterFXConfig = new TalonFXConfiguration();
        climberFXConfig = new TalonFXConfiguration();
        intakeFXConfig = new TalonFXConfiguration();

        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.AZIMUTH_ENABLE_CURRENT_LIMIT, 
            Constants.AZIMUTH_CONTINUOUS_CURRENT_LIMIT, 
            Constants.AZIMUTH_PEAK_CURRENT_LIMIT, 
            Constants.AZIMUTH_PEAK_CURRENT_DURATION);

        swerveAngleFXConfig.slot0.kP = Constants.AZIMUTH_P;
        swerveAngleFXConfig.slot0.kI = Constants.AZIMUTH_I;
        swerveAngleFXConfig.slot0.kD = Constants.AZIMUTH_D;
        swerveAngleFXConfig.slot0.kF = Constants.AZIMUTH_F;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;
        swerveAngleFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;


        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.DRIVE_ENABLE_CURRENT_LIMIT, 
            Constants.DRIVE_CONTINUOUS_CURRENT_LIMIT, 
            Constants.DRIVE_PEAK_CURRENT_LIMIT, 
            Constants.DRIVE_PEAK_CURRENT_DURATION);

        swerveDriveFXConfig.slot0.kP = Constants.DRIVE_P;
        swerveDriveFXConfig.slot0.kI = Constants.DRIVE_I;
        swerveDriveFXConfig.slot0.kD = Constants.DRIVE_D;
        swerveDriveFXConfig.slot0.kF = Constants.DRIVE_F;        
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        swerveDriveFXConfig.openloopRamp = Constants.OPEN_LOOP_RAMP;
        swerveDriveFXConfig.closedloopRamp = Constants.CLOSED_LOOP_RAMP;

        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;

        /* Feeder Falcon Configuration */
        SupplyCurrentLimitConfiguration feederSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.FEEDER_ENABLE_CURRENT_LIMIT,
            Constants.FEEDER_CONTINUOUS_CURRENT_LIMIT,
            Constants.FEEDER_PEAK_CURRENT_LIMIT,
            Constants.FEEDER_PEAK_CURRENT_DURATION
        ); 

        feederFXConfig.slot0.kP = Constants.FEEDER_P;
        feederFXConfig.slot0.kI = Constants.FEEDER_I;
        feederFXConfig.slot0.kD = Constants.FEEDER_D;
        feederFXConfig.slot0.kF = Constants.FEEDER_F;
        feederFXConfig.supplyCurrLimit = feederSupplyLimit;
        feederFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        feederFXConfig.openloopRamp = Constants.OPEN_LOOP_RAMP;
        feederFXConfig.closedloopRamp = Constants.CLOSED_LOOP_RAMP;

        /* Shooter Falcon Configuration */
        SupplyCurrentLimitConfiguration shooterSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.SHOOTER_ENABLE_CURRENT_LIMIT,
            Constants.SHOOTER_CONTINUOUS_CURRENT_LIMIT,
            Constants.SHOOTER_PEAK_CURRENT_LIMIT,
            Constants.SHOOTER_PEAK_CURRENT_DURATION
        );

        shooterFXConfig.slot0.kP = Constants.SHOOTER_P;
        shooterFXConfig.slot0.kI = Constants.SHOOTER_I;
        shooterFXConfig.slot0.kD = Constants.SHOOTER_D;
        shooterFXConfig.slot0.kF = Constants.SHOOTER_F;
        shooterFXConfig.supplyCurrLimit = shooterSupplyLimit;
        shooterFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        shooterFXConfig.openloopRamp = Constants.OPEN_LOOP_RAMP;
        shooterFXConfig.closedloopRamp = Constants.CLOSED_LOOP_RAMP;

        /* Climber Falcon Configuration */
        SupplyCurrentLimitConfiguration climberSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.CLIMBER_ENABLE_CURRENT_LIMIT,
            Constants.CLIMBER_CONTINUOUS_CURRENT_LIMIT,
            Constants.CLIMBER_PEAK_CURRENT_LIMIT,
            Constants.CLIMBER_PEAK_CURRENT_DURATION
        );

        climberFXConfig.slot0.kP = Constants.CLIMBER_P;
        climberFXConfig.slot0.kI = Constants.CLIMBER_I;
        climberFXConfig.slot0.kD = Constants.CLIMBER_D;
        climberFXConfig.slot0.kF = Constants.CLIMBER_F;
        climberFXConfig.supplyCurrLimit = climberSupplyLimit;
        climberFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        climberFXConfig.openloopRamp = Constants.OPEN_LOOP_RAMP;
        climberFXConfig.closedloopRamp = Constants.CLOSED_LOOP_RAMP;

        /* Intake Falcon Configuration */
        SupplyCurrentLimitConfiguration intakeSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.INTAKE_ENABLE_CURRENT_LIMIT,
            Constants.INTAKE_CONTINUOUS_CURRENT_LIMIT,
            Constants.INTAKE_PEAK_CURRENT_LIMIT,
            Constants.INTAKE_PEAK_CURRENT_DURATION
        );

        /*intakeFXConfig.slot0.kP = 0.01;
        intakeFXConfig.slot0.kI = 0.0;
        intakeFXConfig.slot0.kD = 0.01;
        intakeFXConfig.slot0.kF = 100; */

        intakeFXConfig.supplyCurrLimit = intakeSupplyLimit;

        
        






        

    }

}
