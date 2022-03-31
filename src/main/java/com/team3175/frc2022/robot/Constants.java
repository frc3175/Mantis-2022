package com.team3175.frc2022.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public final class Constants {

    /*============================
               Swerve 
    ==============================*/

    /* CAN IDs */
    public static final int BACK_LEFT_DRIVE = 5; //Josh
    public static final int BACK_LEFT_ENCODER = 12; //Gary 
    public static final int BACK_LEFT_AZIMUTH = 3; //Tracy

    public static final int BACK_RIGHT_DRIVE = 13; //Happy
    public static final int BACK_RIGHT_ENCODER = 4; //Bre
    public static final int BACK_RIGHT_AZIMUTH = 14; //Samuel

    public static final int FRONT_RIGHT_DRIVE = 11; //Keith
    public static final int FRONT_RIGHT_ENCODER = 7; //Freddy Mercury
    public static final int FRONT_RIGHT_AZIMUTH = 9; //Beth

    public static final int FRONT_LEFT_DRIVE = 8; //Chad
    public static final int FRONT_LEFT_ENCODER = 10; //Jonathan 
    public static final int FRONT_LEFT_AZIMUTH = 6; //Geraldine

    public static final int PIGEON = 21;

    /* CANCoder offsets */
    public static double FRONT_LEFT_OFFSET = 158.99;
    public static double FRONT_RIGHT_OFFSET = 69.43;
    public static double BACK_LEFT_OFFSET = 134.38;
    public static double BACK_RIGHT_OFFSET = 225.17;

    /* Azimuth reversed */
    public static boolean FRONT_LEFT_AZIMUTH_REVERSED = false;
    public static boolean FRONT_RIGHT_AZIMUTH_REVERSED = false;
    public static boolean BACK_LEFT_AZIMUTH_REVERSED = false;
    public static boolean BACK_RIGHT_AZIMUTH_REVERSED = false;

    /* Drive motors reversed */
    public static boolean FRONT_LEFT_DRIVE_REVERSED = true;
    public static boolean FRONT_RIGHT_DRIVE_REVERSED = true;
    public static boolean BACK_LEFT_DRIVE_REVERSED = true;
    public static boolean BACK_RIGHT_DRIVE_REVERSED = true;

    /* CANCoders reversed */
    public static boolean FRONT_LEFT_CANCODER_REVERSED = false;
    public static boolean FRONT_RIGHT_CANCODER_REVERSED = false;
    public static boolean BACK_LEFT_CANCODER_REVERSED = false;
    public static boolean BACK_RIGHT_CANCODER_REVERSED = true;

    /* Gyro reversed */
    public static final boolean INVERT_GYRO = false;

    /* Angle Motor PID Values */
    public static final double AZIMUTH_P = 0.2;
    public static final double AZIMUTH_I = 0.0;
    public static final double AZIMUTH_D = 0.1;
    public static final double AZIMUTH_F = 0.0;

    /* Drive Motor PID Values */
    public static final double DRIVE_P = 0.0; 
    public static final double DRIVE_I = 0.0;
    public static final double DRIVE_D = 0.0;
    public static final double DRIVE_F = 0.0;

    /* Drive Motor Characterization Values */
    public static final double DRIVE_S = (0.48665 / 12); //Values from SysId divided by 12 to convert to volts for CTRE
    public static final double DRIVE_V = (2.4132 / 12);
    public static final double DRIVE_A = (0.06921 / 12);

    /* Azimuth Current Limiting */
    public static final int AZIMUTH_CONTINUOUS_CURRENT_LIMIT = 25;
    public static final int AZIMUTH_PEAK_CURRENT_LIMIT = 40;
    public static final double AZIMUTH_PEAK_CURRENT_DURATION = 0.1;
    public static final boolean AZIMUTH_ENABLE_CURRENT_LIMIT = true;

    /* Drive Current Limiting */
    public static final int DRIVE_CONTINUOUS_CURRENT_LIMIT = 35;
    public static final int DRIVE_PEAK_CURRENT_LIMIT = 60;
    public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
    public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

    /* Neutral Modes */
    public static final NeutralMode AZIMUTH_NEUTRAL_MODE = NeutralMode.Coast;
    public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Brake;

    /* Swerve Gear Ratios */
    public static final double DRIVE_GEAR_RATIO = (6.86 / 1.0); //6.86:1 from SDS
    public static final double AZIMUTH_GEAR_RATIO = (12.8 / 1.0); //12.8:1 from SDS

    /* Swerve Profiling Values */
    public static final double MAX_SPEED = Units.feetToMeters(16.2); //meters per second (theoretical from SDS)
    public static final double MAX_ANGULAR_VELOCITY = Math.PI * 4.12; //radians per second (theoretical calculation)
    public static final double TURN_IN_PLACE_SPEED = 0.5;
    public static final double A_RATE_LIMITER = 2.0; //Slew Rate Limiter Constant

    /* Auto Swerve profiling */
    public static final double AUTO_MAX_SPEED = Units.feetToMeters(4.9);
    public static final double AUTO_MAX_ACCELERATION_MPS_SQUARED = 3;
    public static final double AUTO_P_X_CONTROLLER = 0.1; 
    public static final double AUTO_P_Y_CONTROLLER = 1.4884;
    public static final double AUTO_P_THETA_CONTROLLER = 2.8;
    public static final double FOUR_BALL_MAX_SPEED = Units.feetToMeters(16.2);
    public static final double FOUR_BALL_MAX_ACCELERATION = 7.6;
    
    /* Constraint for the motion profilied robot angle controller */
    public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS =
        new TrapezoidProfile.Constraints(
            Math.PI, (Math.PI * Math.PI));

    /*============================
               Shooter
    ==============================*/

    /* CAN IDs */
    public static final int LEFT_SHOOTER_FALCON = 2;
    public static final int RIGHT_SHOOTER_FALCON = 20;

    /* Shooter PIDF Values */
    public static final double SHOOTER_P = 0.5;
    public static final double SHOOTER_I = 0.0;
    public static final double SHOOTER_D = 0.0;
    public static final double SHOOTER_F = 0.06;

    /* CTRE Configs */
    public static final int SHOOTER_CONTINUOUS_CURRENT_LIMIT = 35;
    public static final int SHOOTER_PEAK_CURRENT_LIMIT = 60;
    public static final double SHOOTER_PEAK_CURRENT_DURATION = 0.1;
    public static final boolean SHOOTER_ENABLE_CURRENT_LIMIT = true;

    /* Shooter neutral mode */
    public static final NeutralMode SHOOTER_NEUTRAL_MODE = NeutralMode.Coast;

    /* Shooter inversions */
    public static final boolean INVERT_LEFT_SHOOTER = true;
    public static final boolean INVERT_RIGHT_SHOOTER = false;

    /* Shooter setpoint */
    public static final double SHOOTER_TARGET_RPM = 3050;
    public static final double SHOOTER_ERROR = 10; //allowable shooter error from setpoint to be "spun up"

    /*============================
               Climber
    ==============================*/

    /* CAN ID */
    public static final int CLIMBER_FALCON = 18;

    /* Climber PIDF Values */
    public static final double CLIMBER_P = 0.0;
    public static final double CLIMBER_I = 0.0;
    public static final double CLIMBER_D = 0.0;
    public static final double CLIMBER_F = 0.0;

    /* Climber CTRE Configs */
    public static final int CLIMBER_CONTINUOUS_CURRENT_LIMIT = 35;
     public static final int CLIMBER_PEAK_CURRENT_LIMIT = 50;
     public static final double CLIMBER_PEAK_CURRENT_DURATION = 0.1;
     public static final boolean CLIMBER_ENABLE_CURRENT_LIMIT = true;

    /* Climber neutral mode */
    public static final NeutralMode CLIMBER_NEUTRAL_MODE = NeutralMode.Brake;

    /* Invert climber */
    public static final boolean INVERT_CLIMBER = false;
    public static final boolean RE_INVERT_CLIMBER = INVERT_CLIMBER ? false : true;

    /* Climber speeds */
    public static final double CLIMBER_SPEED = 1.0;
    public static final double CLIMBER_UP_DISTANCE = 18;
    public static final double CLIMBER_DOWN_DISTANCE = CLIMBER_UP_DISTANCE - 0.25;
    public static final double CLIMBER_DOWN_SETPOINT = 1000;
    
    /* Climber mechanics */
    public static final double CLIMBER_PULLEY_CIRCUMFERENCE = 0.6 * Math.PI;
    public static final double CLIMBER_GEAR_RATIO = 9;

    /* Climber solenoid */
    public static final int CLIMBER_SOLENOID_LEFT = 4;
    public static final int CLIMBER_SOLENOID_RIGHT = 5;
    public static final DoubleSolenoid.Value CLIMBER_LOCK = Value.kForward;
    public static final DoubleSolenoid.Value CLIMBER_UNLOCK = Value.kReverse;

    /* Climber hooks */
    public static final int CLIMBER_HOOKS_LEFT = 6;
    public static final int CLIMBER_HOOKS_RIGHT = 7;

    /*============================
               Intake
    ==============================*/

    /* CAN ID */
    public static final int INTAKE_FALCON = 17;

    /* Intake CTRE Configs */
    public static final int INTAKE_CONTINUOUS_CURRENT_LIMIT = 15;
    public static final int INTAKE_PEAK_CURRENT_LIMIT = 20;
    public static final double INTAKE_PEAK_CURRENT_DURATION = 0.1;
    public static final boolean INTAKE_ENABLE_CURRENT_LIMIT = true;

    /* Intake inversions */
    public static final boolean INVERT_INTAKE = true;
    public static final boolean RE_INVERT_INTAKE = INVERT_INTAKE ? false : true;

    /* Intake neutral mode */
    public static final NeutralMode INTAKE_NEUTRAL_MODE = NeutralMode.Brake;

    /* Intake speeds */
    public static final double INTAKE_SPEED = 0.9;
    public static final double OUTTAKE_SPEED = -0.9;

    /*============================
               Feeder
    ==============================*/

    /* CAN ID */
    public static final int FEEDER_FALCON = 19;

    /* CTRE Configs */
    public static final int FEEDER_CONTINUOUS_CURRENT_LIMIT = 15;
    public static final int FEEDER_PEAK_CURRENT_LIMIT = 20;
    public static final double FEEDER_PEAK_CURRENT_DURATION = 0.1;
    public static final boolean FEEDER_ENABLE_CURRENT_LIMIT = true;

    /* Feeder neutral mode */
    public static final NeutralMode FEEDER_NEUTRAL_MODE = NeutralMode.Brake;

    /* Feeder PID */
    public static final double FEEDER_P = 0.01;
    public static final double FEEDER_I = 0.0;
    public static final double FEEDER_D = 0.4;
    public static final double FEEDER_F = 0.046;

    /* Inversions */
    public static final boolean INVERT_FEEDER = false;

    /* Feeder Constants */
    public static final double TARGET_FEEDER_RPM = 3000;
    public static final double FEEDER_PERCENT_OUTPUT = 0.9;

    /*============================
               Actuators
    ==============================*/

    /* CAN IDs */
    public static final int ACTUATORS_LEFT = 0;
    public static final int ACTUATORS_RIGHT = 1;

    /*============================
               Kinematics
    ==============================*/

    public static final double DRIVETRAIN_WIDTH = Units.inchesToMeters(21.5);
    public static final double DRIVETRAIN_LENGTH = Units.inchesToMeters(21);
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(DRIVETRAIN_LENGTH / 2.0, DRIVETRAIN_WIDTH / 2.0),
        new Translation2d(DRIVETRAIN_LENGTH / 2.0, -DRIVETRAIN_WIDTH / 2.0),
        new Translation2d(-DRIVETRAIN_LENGTH / 2.0, DRIVETRAIN_WIDTH / 2.0),
        new Translation2d(-DRIVETRAIN_LENGTH / 2.0, -DRIVETRAIN_WIDTH / 2.0));

    /*============================
                Misc.
    ==============================*/

    public static final double OPEN_LOOP_RAMP = 0.25;
    public static final double CLOSED_LOOP_RAMP = 0.0;

    /*============================
         Controller Constants
    ==============================*/

    /* Controller Constants */
    public static final double STICK_DEADBAND = 0.1;
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;
    public static final double OP_RUMBLE_PERCENT = 0.4;
    public static final double DRIVER_RUMBLE_PERCENT = 0.4;
    public static final RumbleType DRIVER_RUMBLE_LEFT = RumbleType.kLeftRumble;
    public static final RumbleType OP_RUMBLE_LEFT = RumbleType.kLeftRumble;
    public static final RumbleType DRIVER_RUMBLE_RIGHT = RumbleType.kRightRumble;
    public static final RumbleType OP_RUMBLE_RIGHT = RumbleType.kRightRumble;
    public static final double DRIVING_INTAKE_RUMBLE = 0.3;

    /*============================
            Auto Constants
    ==============================*/

    public static final double SHOOT_TICKS = 100000;
    public static final double TWO_BALL_INTAKE_TICKS = 100000;
    public static final double FEEDER_TICKS = 150000;
    public static final double FIVE_BALL_FEEDER_TICKS = 40000;
    public static final double SHOOT_DEFENSIVE_BALL_SPEED = 1500;

} 
