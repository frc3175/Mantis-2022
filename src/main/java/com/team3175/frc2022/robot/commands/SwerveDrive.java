package com.team3175.frc2022.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.team3175.frc2022.robot.Constants;
import com.team3175.frc2022.robot.subsystems.SwerveDrivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class SwerveDrive extends CommandBase {

    private double m_rotation;
    private Translation2d m_translation;
    private boolean m_fieldRelative;
    private boolean m_openLoop;
    
    private SwerveDrivetrain m_swerveDrivetrain;
    private XboxController m_driverController;
    private int m_driveAxis;
    private int m_strafeAxis;
    private int m_rotationAxis;

    private SlewRateLimiter m_xAxisARateLimiter;
    private SlewRateLimiter m_yAxisARateLimiter;

    /**
     * 
     * Driver control
     * 
     */
    public SwerveDrive(SwerveDrivetrain swerveDrivetrain, XboxController driverController, int driveAxis, int strafeAxis, int rotationAxis, boolean fieldRelative, boolean openLoop) {
        m_swerveDrivetrain = swerveDrivetrain;
        addRequirements(m_swerveDrivetrain);

        m_driverController = driverController;
        m_driveAxis = XboxController.Axis.kLeftY.value;
        m_strafeAxis = strafeAxis;
        m_rotationAxis = rotationAxis;
        m_fieldRelative = fieldRelative;
        m_openLoop = openLoop;

        m_xAxisARateLimiter = new SlewRateLimiter(Constants.A_RATE_LIMITER);
        m_yAxisARateLimiter = new SlewRateLimiter(Constants.A_RATE_LIMITER);

    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        double yAxis = -m_driverController.getRawAxis(m_driveAxis);
        double xAxis = -m_driverController.getRawAxis(m_strafeAxis);
        double rAxis = -m_driverController.getRawAxis(m_rotationAxis);
        
        /* Deadbands */
        yAxis = (Math.abs(yAxis) < Constants.STICK_DEADBAND) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < Constants.STICK_DEADBAND) ? 0 : xAxis;
        rAxis = (Math.abs(rAxis) < Constants.STICK_DEADBAND) ? 0 : rAxis;

        double rAxisSquared = rAxis > 0 ? rAxis * rAxis : rAxis * rAxis * -1;
        double xAxisSquared = xAxis > 0 ? xAxis * xAxis : xAxis * xAxis * -1;
        double yAxisSquared = yAxis > 0 ? yAxis * yAxis : yAxis * yAxis * -1;

        double yAxisFiltered = m_yAxisARateLimiter.calculate(yAxisSquared);
        double xAxisFiltered = m_xAxisARateLimiter.calculate(xAxisSquared);

        m_translation = new Translation2d(yAxisFiltered, xAxisFiltered).times(Constants.MAX_SPEED);
        m_rotation = rAxisSquared * Constants.MAX_ANGULAR_VELOCITY * 0.5;
        m_swerveDrivetrain.drive(m_translation, m_rotation, m_fieldRelative, m_openLoop);

        SmartDashboard.putNumber("yAxis", yAxis);
        SmartDashboard.putNumber("xAxis", xAxis);
        SmartDashboard.putNumber("rAxis", rAxis);

    }
}

