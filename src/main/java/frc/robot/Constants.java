package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

    /*============================
               CAN IDs
    ==============================*/

     //CAN IDS
    public static final int BACK_LEFT_DRIVE = 13; //Josh
    public static final int BACK_LEFT_ENCODER = 4; //Gary 
    public static final int BACK_LEFT_AZIMUTH = 14; //Tracy

    public static final int BACK_RIGHT_DRIVE = 8; //Happy
    public static final int BACK_RIGHT_ENCODER = 10; //Bre
    public static final int BACK_RIGHT_AZIMUTH = 6; //Samuel

    public static final int FRONT_RIGHT_DRIVE = 11; //Keith
    public static final int FRONT_RIGHT_ENCODER = 7; //Freddy Mercury
    public static final int FRONT_RIGHT_AZIMUTH = 9; //Beth

    public static final int FRONT_LEFT_DRIVE = 5; //Chad
    public static final int FRONT_LEFT_ENCODER = 12; //Jonathan 
    public static final int FRONT_LEFT_AZIMUTH = 3; //Geraldine

    public static final int INTAKE_FALCON = 0; //tune

    /*============================
           Module Constants
    ==============================*/

    //this is where you put the angle offsets
    public static double FRONT_LEFT_OFFSET = 44.25; //temp value, normal is 224.25
    public static double FRONT_RIGHT_OFFSET = 250.84;
    public static double BACK_LEFT_OFFSET = 43.5; //temp value, normal is 223.50
    public static double BACK_RIGHT_OFFSET = 202.43;

    //Turning motors reversed
    public static boolean FRONT_LEFT_AZIMUTH_REVERSED = false;
    public static boolean FRONT_RIGHT_AZIMUTH_REVERSED = false;
    public static boolean BACK_LEFT_AZIMUTH_REVERSED = false;
    public static boolean BACK_RIGHT_AZIMUTH_REVERSED = false;

    //Drive motors reversed
    public static boolean FRONT_LEFT_DRIVE_REVERSED = false; //temp, normally true
    public static boolean FRONT_RIGHT_DRIVE_REVERSED = true;
    public static boolean BACK_LEFT_DRIVE_REVERSED = false; //temp, normally true
    public static boolean BACK_RIGHT_DRIVE_REVERSED = true; 

    //CanCoders Reversed
    public static boolean FRONT_LEFT_CANCODER_REVERSED = false;
    public static boolean FRONT_RIGHT_CANCODER_REVERSED = false;
    public static boolean BACK_LEFT_CANCODER_REVERSED = false;
    public static boolean BACK_RIGHT_CANCODER_REVERSED = true;

    /*============================
        PIDF & Characterization
    ==============================*/

    /* Angle Motor PID Values */
    public static final double AZIMUTH_P = 0.2; //FIXME: Tune all of these (drive pidf too) at some point too
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

    /*============================
               Kinematics
    ==============================*/

    public static final double DRIVETRAIN_WIDTH = Units.inchesToMeters(18);
    public static final double DRIVETRAIN_LENGTH = Units.inchesToMeters(18);
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

     /* Swerve Current Limiting */
     public static final int AZIMUTH_CONTINUOUS_CURRENT_LIMIT = 25; //FIXME: Adjust these to stop brownouts
     public static final int AZIMUTH_PEAK_CURRENT_LIMIT = 40;
     public static final double AZIMUTH_PEAK_CURRENT_DURATION = 0.1;
     public static final boolean AZIMUTH_ENABLE_CURRENT_LIMIT = true;

     public static final int DRIVE_CONTINUOUS_CURRENT_LIMIT = 35;
     public static final int DRIVE_PEAK_CURRENT_LIMIT = 60;
     public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
     public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

     /* Neutral Modes */
     public static final NeutralMode AZIMUTH_NEUTRAL_MODE = NeutralMode.Coast;
     public static final NeutralMode DRIVE_NEUTRAL_MODE = NeutralMode.Brake;

    /*============================
            TeleOp Constants
    ==============================*/

    /* Controller Constants */
    public static final double STICK_DEADBAND = 0.1;
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;

    /* Inversions */
    public static final boolean INVERT_GYRO = true; // Always ensure Gyro is CCW+ CW- !!!!!
    public static final boolean INVERT_INTAKE = true; //opposite of intake_not_inverted
    public static final boolean RE_INVERT_INTAKE = false; //opposite of invert_intake

    /* Swerve Profiling Values */
    public static final double MAX_SPEED = Units.feetToMeters(16.2); //meters per second
    public static final double MAX_ANGULAR_VELOCITY = Math.PI * 5.2 * 0.625; //actually 5.2
    public static final double TURN_IN_PLACE_SPEED = 0.5;

    /*============================
            Auto Constants
    ==============================*/

    //RPS IS RADIANS PER SECOND
    //MPS IS METERS PER SECOND
    public static final double AUTO_MAX_SPEED_MPS = 4.9; //FIXME: Adjust this to whatever speed we want for auto
    public static final double AUTO_MAX_ACCELERATION_MPS_SQUARED = 3;
    
    public static final double AUTO_P_X_CONTROLLER = 1; 
    public static final double AUTO_P_Y_CONTROLLER = 1.4884;
    public static final double AUTO_P_THETA_CONTROLLER = 1;
    
    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS =
        new TrapezoidProfile.Constraints(
            MAX_ANGULAR_VELOCITY, (MAX_ANGULAR_VELOCITY * MAX_ANGULAR_VELOCITY));
    }

