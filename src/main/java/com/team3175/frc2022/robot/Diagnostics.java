package com.team3175.frc2022.robot;

import com.team3175.frc2022.lib.math.Conversions;
import com.team3175.frc2022.robot.subsystems.Actuators;
import com.team3175.frc2022.robot.subsystems.Climber;
import com.team3175.frc2022.robot.subsystems.Feeder;
import com.team3175.frc2022.robot.subsystems.Intake;
import com.team3175.frc2022.robot.subsystems.Shooter;
import com.team3175.frc2022.robot.subsystems.SwerveDrivetrain;
import com.team3175.frc2022.robot.subsystems.SwerveModule;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings("unused")
public class Diagnostics extends SubsystemBase {

    private static NetworkTableInstance inst;
    private static NetworkTable diagnosticTable;

    private SwerveDrivetrain m_drivetrain;
    private Climber m_climber;
    private Intake m_intake;
    private Feeder m_feeder;
    private Shooter m_shooter;
    private Actuators m_actuators;

    private ShuffleboardTab driveTab;
    private ComplexWidget gyroEntry;
    private NetworkTableEntry intakeActuationEntry;
    private NetworkTableEntry climberLockEntry;

    /* Creates a diagnostic table */
    public Diagnostics(SwerveDrivetrain drivetrain, Climber climber, Intake intake, Feeder feeder, Shooter shooter, Actuators actuators){

        m_drivetrain = drivetrain;
        m_climber = climber;
        m_intake = intake;
        m_feeder = feeder;
        m_shooter = shooter;
        m_actuators = actuators;

        UsbCamera m_camera = CameraServer.startAutomaticCapture();
        m_camera.setResolution(400, 400);

        inst = NetworkTableInstance.getDefault();
        diagnosticTable = inst.getTable("datatable");
        inst.setUpdateRate(0.01);

        driveTab = Shuffleboard.getTab("Match Dashboard");
        
        gyroEntry = driveTab.add("Gyro", m_drivetrain.m_gyro);
    
        intakeActuationEntry = driveTab.add("intake down", m_intake.isIntakeRunning())
                                       .getEntry();

        climberLockEntry = driveTab.add("climber lock", m_climber.isClimberLocked()).getEntry();

    }

    /**
     * 
     * Push match diagnostics to the match dashboard
     * 
     */

    public void pushMatchDashboardDiagnostics() {

        boolean isIntakeDown = m_intake.isIntakeRunning();
        intakeActuationEntry.setBoolean(isIntakeDown);

    }

    /**
     * 
     * Push each module position and gyro position to dashboard
     * 
     */

    public void pushDrivetrainDiagnostics() {

        SwerveModule[] m_swerveModules = m_drivetrain.getModules();

        for(SwerveModule mod : m_swerveModules){
            String cancoder = "Mod " + mod.m_moduleNumber + " Cancoder";
            String velocity = "Mod " + mod.m_moduleNumber + " Velocity";
            String driveEncoder = "Mod " + mod.m_moduleNumber + " Drive Encoder";  
            String azimuth = "Mod " + mod.m_moduleNumber + " Azimuth angle";

            pushDouble(cancoder, mod.getCanCoder().getDegrees());
            pushDouble(velocity, mod.getState().speedMetersPerSecond);
            pushDouble(driveEncoder, mod.getDriveEncoder());
            pushDouble(azimuth, mod.getState().angle.getDegrees());
        }

        SmartDashboard.putNumber("Gyro Yaw: ", m_drivetrain.getYaw().getDegrees());

        pushDouble("raw gyro", m_drivetrain.getYaw().getDegrees());

        diagnosticTable.getEntry("pose x").setDouble(m_drivetrain.getPose().getX());
        diagnosticTable.getEntry("pose y").setDouble(m_drivetrain.getPose().getY());
        diagnosticTable.getEntry("pose rot").setDouble((m_drivetrain.getPose().getRotation().getDegrees()));

    }

    /**
     * 
     * Push climber encoder, temp, current draw, and rpm to dashboard
     * 
     */

    public void pushClimberDiagnostics() {

        pushDouble("climber encoder", m_climber.getClimberEncoder());
        pushDouble("climber temp", m_climber.getTemp());
        pushDouble("climber current", m_climber.getCurrent());
        pushBoolean("climber alive", m_climber.isAlive());
        pushDouble("climber velocity rpm", Conversions.falconToRPM(m_climber.getVelocity(), 1.0));
        pushBoolean("climber locked", m_climber.isClimberLocked());

    }

    /**
     * 
     * Push intake encoder, temp, current draw, and rpm to dashboard
     * 
     */

    public void pushIntakeDiagnostics() {

        pushDouble("intake encoder", m_intake.getEncoders());
        pushDouble("intake temp", m_intake.getTemp());
        pushDouble("intake current", m_intake.getCurrent());
        pushBoolean("intake alive", m_intake.isAlive());
        pushDouble("intake velocity rpm", Conversions.falconToRPM(m_intake.getVelocity(), 1.0));

    }

    /**
     * 
     * Push feeder encoder, temp, current draw, and rpm to dashboard
     * 
     */

    public void pushFeederDiagnostics() {

        pushDouble("feeder encoder", m_feeder.getEncoder());
        pushDouble("feeder temp", m_feeder.getTemp());
        pushDouble("feeder current", m_feeder.getCurrent());
        pushBoolean("feeder alive", m_feeder.isAlive());
        pushDouble("feeder velocity rpm", Conversions.falconToRPM(m_feeder.getVelocity(), 1.0));

    }

    /**
     * 
     * Push left and right shooter encoders, temps, current draws, and rpms to dashboard
     * 
     */

    public void pushShooterDiagnostics() {

        pushDouble("shooter left encoder", m_shooter.getLeftEncoder());
        pushDouble("shooter left temp", m_shooter.getLeftTemp());
        pushDouble("shooter left current", m_shooter.getLeftCurrent());
        pushBoolean("shooter left alive", m_shooter.isLeftAlive());
        pushDouble("shooter left velocity rpm", m_shooter.getLeftVelocityRPM());
        pushDouble("shooter right encoder", m_shooter.getRightEncoder());
        pushDouble("shooter right temp", m_shooter.getRightTemp());
        pushDouble("shooter right current", m_shooter.getRightCurrent());
        pushBoolean("shooter right alive", m_shooter.isRightAlive());
        pushDouble("shooter right velocity rpm", m_shooter.getRightVelocityRPM());

    }

    /**
     * 
     * Creates a network table entry with a specified name and double value
     * 
     * @param name name to push to network tables
     * @param value double value to publish to network tables
     * 
     */

    public static void pushDouble(String name, double value){
        diagnosticTable.getEntry(name).setDouble(value);
    }

    /**
     * 
     * Creates a network table entry with a specified name and boolean value
     * 
     * @param name name to push to network tables
     * @param value boolean value to publish to network tables
     * 
     */

    public static void pushBoolean(String name, boolean value) {
        diagnosticTable.getEntry(name).setBoolean(value);
    }

    @Override
    public void periodic() {

        pushDrivetrainDiagnostics();
        pushFeederDiagnostics();
        pushIntakeDiagnostics();
        pushShooterDiagnostics();
        pushClimberDiagnostics();
        pushMatchDashboardDiagnostics();

    }

}