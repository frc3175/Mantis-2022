package com.team3175.frc2022.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.kauailabs.navx.frc.AHRS;
import com.team3175.frc2022.robot.Constants;

import edu.wpi.first.wpilibj.SPI;

public class SwerveDrivetrain extends SubsystemBase {

    public SwerveDriveOdometry m_swerveOdometry;
    public SwerveModule[] m_swerveModules;
    public AHRS m_gyro;

    /**
     * 
     * Constructor for the entire Swerve Drivetrain
     * 
     * Creates all 4 module instances in addition to gyro and odometry configuration
     * 
     */

    public SwerveDrivetrain() {
        m_gyro = new AHRS(SPI.Port.kMXP);
        m_gyro.reset();
        
        m_swerveOdometry = new SwerveDriveOdometry(Constants.swerveKinematics, getYaw());

        m_swerveModules = new SwerveModule[] {
            new SwerveModule(0, 
                             Constants.FRONT_LEFT_OFFSET, 
                             Constants.FRONT_LEFT_AZIMUTH, 
                             Constants.FRONT_LEFT_DRIVE, 
                             Constants.FRONT_LEFT_ENCODER, 
                             Constants.FRONT_LEFT_AZIMUTH_REVERSED,
                             Constants.FRONT_LEFT_DRIVE_REVERSED,
                             Constants.FRONT_LEFT_CANCODER_REVERSED),
            new SwerveModule(1,
                             Constants.FRONT_RIGHT_OFFSET,
                             Constants.FRONT_RIGHT_AZIMUTH,
                             Constants.FRONT_RIGHT_DRIVE,
                             Constants.FRONT_RIGHT_ENCODER,
                             Constants.FRONT_RIGHT_AZIMUTH_REVERSED,
                             Constants.FRONT_RIGHT_DRIVE_REVERSED,
                             Constants.FRONT_RIGHT_CANCODER_REVERSED),
            new SwerveModule(2,
                             Constants.BACK_LEFT_OFFSET,
                             Constants.BACK_LEFT_AZIMUTH,
                             Constants.BACK_LEFT_DRIVE,
                             Constants.BACK_LEFT_ENCODER,
                             Constants.BACK_LEFT_AZIMUTH_REVERSED,
                             Constants.BACK_LEFT_DRIVE_REVERSED,
                             Constants.BACK_LEFT_CANCODER_REVERSED),
            new SwerveModule(3,
                             Constants.BACK_RIGHT_OFFSET,
                             Constants.BACK_RIGHT_AZIMUTH,
                             Constants.BACK_RIGHT_DRIVE,
                             Constants.BACK_RIGHT_ENCODER,
                             Constants.BACK_RIGHT_AZIMUTH_REVERSED,
                             Constants.BACK_RIGHT_DRIVE_REVERSED,
                             Constants.BACK_RIGHT_CANCODER_REVERSED)
        };

    }

    /**
     * 
     * The drive method for the drivetrain
     * 
     * @param translation Translation2d object containing a vector which represents the distance to be traveled in x and y axes
     * @param rotation The holonomic rotation value from the rotation joystick axis
     * @param fieldRelative If the robot is driving field relative or not, should only be false in the case of a brownout
     * @param isOpenLoop Whether or not the robot is driving using open loop control, almost always false
     * 
     */

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {

        SwerveModuleState[] swerveModuleStates =
            Constants.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYaw())
                              : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.MAX_SPEED);

        for(SwerveModule module : m_swerveModules){
            module.setDesiredState(swerveModuleStates[module.m_moduleNumber], isOpenLoop);
        }


    }

    /**
     * 
     * Sets all 4 modules to the desired states using an array of SwerveModuleStates based on calculations
     * 
     * @param desiredStates An array of all 4 desired states
     * 
     */

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.MAX_SPEED);
        
        for(SwerveModule mod : m_swerveModules){
            mod.setDesiredState(desiredStates[mod.m_moduleNumber], false);
        }
    }

    /**
     * 
     * Sets all 4 modules to the desired states using a ChassisSpeeds object
     * 
     * @param targetSpeeds The target speeds for all modules in ChassisSpeeds form
     * 
     */
    
    public void setChassisSpeeds(ChassisSpeeds targetSpeeds) {
        setModuleStates(Constants.swerveKinematics.toSwerveModuleStates(targetSpeeds));
    }

    /**
     * 
     * @return Pose of the robot in meters 
     * 
     */

    public Pose2d getPose() {
        return m_swerveOdometry.getPoseMeters();
    }

    /**
     * 
     * Sets the robot odometry to the current known pose
     * 
     * @param pose Current pose of the robot
     * 
     */

    public void resetOdometry(Pose2d pose) {
        m_swerveOdometry.resetPosition(pose, getYaw());
    }

    /**
     * 
     * Gets the theta angle of the robot
     * 
     * @return Current gyro Yaw value in degrees -180-180
     * 
     */

    public double getAngle() {
        double angle = m_gyro.getYaw();
        return angle;
    }

    /**
     * 
     * @return Array of SwerveModuleStates containing the states of all 4 robots
     * 
     */

    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : m_swerveModules){
            states[mod.m_moduleNumber] = mod.getState();
        }
        return states;
    }

    /**
     * 
     * Returns the theta angle of the robot as a Rotation2d object
     * 
     * @return Gyro angle as Rotation2d
     * 
     */

    public Rotation2d getYaw() {
        return (Constants.INVERT_GYRO) ? Rotation2d.fromDegrees(360 - m_gyro.getYaw()) 
                                             : Rotation2d.fromDegrees(m_gyro.getYaw());
    }

    /**
     * 
     * Sets the current gyro angle to 0 no matter the robot orientation
     * 
     */

    public void resetGyro() {
        m_gyro.reset();
    }

    /**
     * 
     * Sets all module positions to 0 no matter their orientation
     * 
     */

    public void zeroModules() {

        for(SwerveModule mod: m_swerveModules) {
            mod.zeroModule();
        }

    }

    /**
     * 
     * Optimizes the holonomic rotation of the robot
     * 
     * @param currentAngle current theta orientation of the robot
     * @param desiredAngle desired theta orientation of the robot
     * @return which direction the robot should turn: true = left, false = right
     * 
     * Note: WILL ALWAYS RETURN A VALUE, EVEN IF CURRENT = DESIRED SO MAKE SURE TO ONLY RUN THIS METHOD WHEN CURRENT != DESIRED
     * 
     */

    public boolean optimizeTurning(double currentAngle, double desiredAngle) {

        boolean isDesiredPositive = desiredAngle > 0;
        boolean isCurrentPositive = currentAngle > 0;

        double m_desiredAngle = Math.abs(desiredAngle);
        double m_currentAngle = Math.abs(currentAngle);

        double absTotal = Math.abs(currentAngle) + Math.abs(desiredAngle);

        if(isDesiredPositive && isCurrentPositive) {
            if(m_desiredAngle > m_currentAngle) {
                return true;
            } else {
                return false;
            }
        } else if(!isDesiredPositive && !isCurrentPositive) {
            if(m_desiredAngle > m_currentAngle) {
                return true;
            } else {
                return false;
            }
        } else {
            if(absTotal > 180) {
                return false;
            } else {
                return true;
            }
        }
        

    }

    /**
     * 
     * Updates odometry with current theta angle and module states
     * 
     * Pushes module cancoder and integrated encoder values, module velocities, and gyro angle to SmartDashboard
     * 
     */

    @Override
    public void periodic(){
        m_swerveOdometry.update(getYaw(), getStates());  

        for(SwerveModule mod : m_swerveModules){
            SmartDashboard.putNumber("Mod " + mod.m_moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.m_moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
            SmartDashboard.putNumber("Mod " + mod.m_moduleNumber + " Drive Encoder", mod.getDriveEncoder());    
            SmartDashboard.putNumber("Mod " + mod.m_moduleNumber + " Azimuth angle", mod.getState().angle.getDegrees());
        }

        SmartDashboard.putNumber("Gyro Yaw: ", m_gyro.getYaw());

    }

    
}
