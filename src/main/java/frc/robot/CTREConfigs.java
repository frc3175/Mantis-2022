package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

//TODO: make sure to config all motors and do all configs in subsystem manager file!!

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;
    public static TalonFXConfiguration hopperFXConfig;


    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();
        hopperFXConfig = new TalonFXConfiguration();

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

        /* Hopper Falcon Configuration */
        SupplyCurrentLimitConfiguration hopperSupplyLimit = new SupplyCurrentLimitConfiguration(
            Constants.HOPPER_ENABLE_CURRENT_LIMIT,
            Constants.HOPPER_CONTINUOUS_CURRENT_LIMIT,
            Constants.HOPPER_PEAK_CURRENT_LIMIT,
            Constants.HOPPER_PEAK_CURRENT_DURATION
        );

        hopperFXConfig.slot0.kP = Constants.HOPPER_P;
        hopperFXConfig.slot0.kI = Constants.HOPPER_I;
        hopperFXConfig.slot0.kD = Constants.HOPPER_D;
        hopperFXConfig.slot0.kF = Constants.HOPPER_F;
        hopperFXConfig.supplyCurrLimit = hopperSupplyLimit;
        hopperFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
        hopperFXConfig.openloopRamp = Constants.OPEN_LOOP_RAMP; //maybe make this own ramp to spin up faster???
        hopperFXConfig.closedloopRamp = Constants.CLOSED_LOOP_RAMP; //"                                     "



        

    }

}
