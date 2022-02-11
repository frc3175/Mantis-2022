package com.team3175.frc2022.robot;

import com.team3175.frc2022.lib.math.Conversions;
import com.team3175.frc2022.robot.subsystems.Climber;
import com.team3175.frc2022.robot.subsystems.Feeder;
import com.team3175.frc2022.robot.subsystems.Intake;
import com.team3175.frc2022.robot.subsystems.Shooter;
import com.team3175.frc2022.robot.subsystems.SwerveDrivetrain;
import com.team3175.frc2022.robot.subsystems.SwerveModule;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@SuppressWarnings("unused")
public class Diagnostics {

    private static NetworkTableInstance inst;
    private static NetworkTable diagnosticTable;

    private SwerveDrivetrain m_drivetrain = new SwerveDrivetrain();
    private Climber m_climber = new Climber();
    private Intake m_intake = new Intake();
    private Feeder m_feeder = new Feeder();
    private Shooter m_shooter = new Shooter();
    //private Actuators m_actuators = new Actuators();

    private ShuffleboardTab driveTab;
    private ComplexWidget gyroEntry;
    private NetworkTableEntry intakeActuationEntry;

    // creates a diagnostic table
    public Diagnostics(){

        UsbCamera m_camera = CameraServer.startAutomaticCapture();
        m_camera.setResolution(400, 400);

        inst = NetworkTableInstance.getDefault();
        diagnosticTable = inst.getTable("datatable");
        inst.setUpdateRate(0.01);

        driveTab = Shuffleboard.getTab("Match Dashboard");
        
        gyroEntry = driveTab.add("Gyro", m_drivetrain.m_gyro);
    
        intakeActuationEntry = driveTab.add("intake down", m_intake.isIntakeRunning())
                                       .getEntry();
    }

    public void pushMatchDashboardDiagnostics() {

        //double gyro = m_drivetrain.getAngle();
        //gyroEntry.setDouble(gyro);

        boolean isIntakeDown = m_intake.isIntakeRunning();
        intakeActuationEntry.setBoolean(isIntakeDown);

    }

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

        SmartDashboard.putNumber("Gyro Yaw: ", m_drivetrain.getAngle());

    }

    public void pushClimberDiagnostics() {

        pushDouble("climber encoder", m_climber.getClimberEncoder());
        pushDouble("climber temp", m_climber.getTemp());
        pushDouble("climber current", m_climber.getCurrent());
        pushBoolean("climber alive", m_climber.isAlive());
        pushDouble("climber velocity rpm", Conversions.falconToRPM(m_climber.getVelocity(), 1.0));

    }

    public void pushIntakeDiagnostics() {

        pushDouble("intake encoder", m_intake.getEncoders());
        pushDouble("intake temp", m_intake.getTemp());
        pushDouble("intake current", m_intake.getCurrent());
        pushBoolean("intake alive", m_intake.isAlive());
        pushDouble("intake velocity rpm", Conversions.falconToRPM(m_intake.getVelocity(), 1.0));

    }

    public void pushFeederDiagnostics() {

        pushDouble("feeder encoder", m_feeder.getEncoder());
        pushDouble("feeder temp", m_feeder.getTemp());
        pushDouble("feeder current", m_feeder.getCurrent());
        pushBoolean("feeder alive", m_feeder.isAlive());
        pushDouble("feeder velocity rpm", Conversions.falconToRPM(m_feeder.getVelocity(), 1.0));

    }

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

    public static void pushDouble(String name, double value){
        diagnosticTable.getEntry(name).setDouble(value);
    }

    public static void pushBoolean(String name, boolean value) {
        diagnosticTable.getEntry(name).setBoolean(value);
    }

}