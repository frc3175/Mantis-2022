package com.team3175.frc2022.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public final class Constants {

    /*============================
               CAN IDs
    ==============================*/

     //CAN IDS
    public static final int BACK_LEFT_DRIVE = 5; //Josh
    public static final int BACK_LEFT_ENCODER = 12; //Gary 
    public static final int BACK_LEFT_AZIMUTH = 3; //Tracy

    public static final int BACK_RIGHT_DRIVE = 13; //Happy
    public static final int BACK_RIGHT_ENCODER = 4; //Bre
    public static final int BACK_RIGHT_AZIMUTH = 14; //Samuel

    public static final int FRONT_RIGHT_DRIVE = 8; //Keith
    public static final int FRONT_RIGHT_ENCODER = 10; //Freddy Mercury
    public static final int FRONT_RIGHT_AZIMUTH = 6; //Beth

    public static final int FRONT_LEFT_DRIVE = 11; //Chad
    public static final int FRONT_LEFT_ENCODER = 7; //Jonathan 
    public static final int FRONT_LEFT_AZIMUTH = 9; //Geraldine

    public static final int INTAKE_FALCON = 17;

    public static final int FEEDER_FALCON = 19;

    public static final int LEFT_SHOOTER_FALCON = 2;
    public static final int RIGHT_SHOOTER_FALCON = 20;

    public static final int CLIMBER_FALCON = 18;

    public static final int ACTUATORS_LEFT = 0;
    public static final int ACTUATORS_RIGHT = 1;

    /*============================
           Module Constants
    ==============================*/

    //this is where you put the angle offsets
    public static double FRONT_LEFT_OFFSET = 159.87; //temp value, normal is 224.25
    public static double FRONT_RIGHT_OFFSET = 67.67;
    public static double BACK_LEFT_OFFSET = 134.64; //temp value, normal is 223.50
    public static double BACK_RIGHT_OFFSET = 226.66;

    //Turning motors reversed
    public static boolean FRONT_LEFT_AZIMUTH_REVERSED = false;
    public static boolean FRONT_RIGHT_AZIMUTH_REVERSED = false;
    public static boolean BACK_LEFT_AZIMUTH_REVERSED = false;
    public static boolean BACK_RIGHT_AZIMUTH_REVERSED = false;

    //Drive motors reversed
    public static boolean FRONT_LEFT_DRIVE_REVERSED = true; //temp, normally true
    public static boolean FRONT_RIGHT_DRIVE_REVERSED = true;
    public static boolean BACK_LEFT_DRIVE_REVERSED = true; //temp, normally true
    public static boolean BACK_RIGHT_DRIVE_REVERSED = true; 

    //CanCoders Reversed
    public static boolean FRONT_LEFT_CANCODER_REVERSED = false;
    public static boolean FRONT_RIGHT_CANCODER_REVERSED = false;
    public static boolean BACK_LEFT_CANCODER_REVERSED = false;
    public static boolean BACK_RIGHT_CANCODER_REVERSED = true;

    /*============================
        PIDF & Characterization
    ==============================*/

    //TODO: Characterize drivetrain on new robot
    //TODO: Tune TeleOp PID on new robot for drivetrain, shooter, and climber

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
    public static final double DRIVE_S = (0.48665 / 12); //divide by 12 to convert from volts to percent output for CTRE
    public static final double DRIVE_V = (2.4132 / 12);
    public static final double DRIVE_A = (0.06921 / 12);

    /* Shooter PIDF Values */
    public static final double SHOOTER_P = 0.1;
    public static final double SHOOTER_I = 0.001;
    public static final double SHOOTER_D = 5.0;
    public static final double SHOOTER_F = 750.0;

    /* Climber PIDF Values */
    public static final double CLIMBER_P = 0.0;
    public static final double CLIMBER_I = 0.0;
    public static final double CLIMBER_D = 0.0;
    public static final double CLIMBER_F = 0.0;

    /*============================
               Kinematics
    ==============================*/

    //TODO: Adjust motor ramps, potentially create new motor ramps for shooter and other subs

    public static final double DRIVETRAIN_WIDTH = Units.inchesToMeters(21.5);
    public static final double DRIVETRAIN_LENGTH = Units.inchesToMeters(21);
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;

    public static final double OPEN_LOOP_RAMP = 0.25; //tune this as well??
    public static final double CLOSED_LOOP_RAMP = 0.0;

    public static final double DRIVE_GEAR_RATIO = (6.86 / 1.0); //6.86:1
    public static final double AZIMUTH_GEAR_RATIO = (12.8 / 1.0); //12.8:1

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(DRIVETRAIN_LENGTH / 2.0, DRIVETRAIN_WIDTH / 2.0),
        new Translation2d(DRIVETRAIN_LENGTH / 2.0, -DRIVETRAIN_WIDTH / 2.0),
        new Translation2d(-DRIVETRAIN_LENGTH / 2.0, DRIVETRAIN_WIDTH / 2.0),
        new Translation2d(-DRIVETRAIN_LENGTH / 2.0, -DRIVETRAIN_WIDTH / 2.0));

    /*============================
              CTRE Configs
    ==============================*/

    //TODO: Adjust ALL current limits to what coopers calculations find reasonable
    //TODO: Double check all neutral modes to make sure they are what they should be

     /* Current Limiting */
     public static final int AZIMUTH_CONTINUOUS_CURRENT_LIMIT = 25;
     public static final int AZIMUTH_PEAK_CURRENT_LIMIT = 40;
     public static final double AZIMUTH_PEAK_CURRENT_DURATION = 0.1;
     public static final boolean AZIMUTH_ENABLE_CURRENT_LIMIT = true;

     public static final int DRIVE_CONTINUOUS_CURRENT_LIMIT = 35;
     public static final int DRIVE_PEAK_CURRENT_LIMIT = 60;
     public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
     public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

     public static final int FEEDER_CONTINUOUS_CURRENT_LIMIT = 35; //absolutely no clue what these should be, fix!!
     public static final int FEEDER_PEAK_CURRENT_LIMIT = 60;
     public static final double FEEDER_PEAK_CURRENT_DURATION = 0.1;
     public static final boolean FEEDER_ENABLE_CURRENT_LIMIT = true;

     public static final int SHOOTER_CONTINUOUS_CURRENT_LIMIT = 35;
     public static final int SHOOTER_PEAK_CURRENT_LIMIT = 60;
     public static final double SHOOTER_PEAK_CURRENT_DURATION = 0.1;
     public static final boolean SHOOTER_ENABLE_CURRENT_LIMIT = true;

     public static final int CLIMBER_CONTINUOUS_CURRENT_LIMIT = 35;
     public static final int CLIMBER_PEAK_CURRENT_LIMIT = 60;
     public static final double CLIMBER_PEAK_CURRENT_DURATION = 0.1;
     public static final boolean CLIMBER_ENABLE_CURRENT_LIMIT = true;

     public static final int INTAKE_CONTINUOUS_CURRENT_LIMIT = 35;
     public static final int INTAKE_PEAK_CURRENT_LIMIT = 60;
     public static final double INTAKE_PEAK_CURRENT_DURATION = 0.1;
     public static final boolean INTAKE_ENABLE_CURRENT_LIMIT = true;

     /* Neutral Modes */
     public static final NeutralMode AZIMUTH_NEUTRAL_MODE = NeutralMode.Coast;
     public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Brake;
     public static final NeutralMode FEEDER_NEUTRAL_MODE = NeutralMode.Brake; //???
     public static final NeutralMode SHOOTER_NEUTRAL_MODE = NeutralMode.Coast; //this is coast in case i need to switch to bang bang
     public static final NeutralMode CLIMBER_NEUTRAL_MODE = NeutralMode.Brake; //????
     public static final NeutralMode INTAKE_NEUTRAL_MODE = NeutralMode.Brake;

    /*============================
            TeleOp Constants
    ==============================*/

    //TODO: Adjust controller constants to preferences
    //TODO: Fix intake inversion
    //TODO: Change all max speeds based on testing and tuning

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

    /* Inversions */
    public static final boolean INVERT_GYRO = true; // Always ensure Gyro is CCW+ CW- !!!!!
    public static final boolean INVERT_INTAKE = true; //opposite of intake_not_inverted
    public static final boolean RE_INVERT_INTAKE = false; //opposite of invert_intake
    public static final boolean INVERT_FEEDER = false;
    public static final boolean INVERT_LEFT_SHOOTER = true;
    public static final boolean INVERT_RIGHT_SHOOTER = false;
    public static final boolean INVERT_CLIMBER = false;

    /* Intake Constants */
    public static final double INTAKE_SPEED = 0.9;
    public static final double OUTTAKE_SPEED = 0.9;

    /* Feeder Constants */
    public static final double TARGET_FEEDER_RPM = 1000; //random

    /* Shooter Constants */
    public static final double SHOOTER_TARGET_RPM = 50; //random
    public static final double SHOOTER_ERROR = 100; //allowable shooter error from setpoint to be "spun up"

    /* Climber Constants */
    public static final double CLIMBER_SPEED = 0.5;
    public static final double CLIMBER_UP_SETPOINT = 5000; 
    public static final double CLIMBER_DOWN_SETPOINT = 100;

    /* Swerve Profiling Values */
    public static final double MAX_SPEED = Units.feetToMeters(8); //meters per second //TODO: make 16
    public static final double MAX_ANGULAR_VELOCITY = Math.PI * 5.2 * 0.625; //actually 5.2
    public static final double TURN_IN_PLACE_SPEED = 0.5;

    /*============================
            Auto Constants
    ==============================*/

    //TODO: Adjust all auto P controllers on new robot

    //RPS IS RADIANS PER SECOND
    //MPS IS METERS PER SECOND
    public static final double AUTO_MAX_SPEED = 4.9;
    public static final double AUTO_MAX_ACCELERATION_MPS_SQUARED = 3;
    
    public static final double AUTO_P_X_CONTROLLER = 0; 
    public static final double AUTO_P_Y_CONTROLLER = 1.4884;
    public static final double AUTO_P_THETA_CONTROLLER = 0.01;
    
    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS =
        new TrapezoidProfile.Constraints(
            Math.PI, (Math.PI * Math.PI));
    }
