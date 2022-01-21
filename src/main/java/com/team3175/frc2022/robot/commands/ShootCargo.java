package com.team3175.frc2022.robot.commands;

import com.team3175.frc2022.robot.Constants;
import com.team3175.frc2022.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ShootCargo extends CommandBase {

    private final Shooter m_shooter;
    private final double m_rpm;
    private final XboxController m_opController;
    private final XboxController m_driveController;

    public ShootCargo(Shooter shooter, double rpm, XboxController opController, XboxController driveController) {

        m_shooter = shooter;
        m_rpm = rpm;
        m_opController = opController;
        m_driveController = driveController;

        addRequirements(m_shooter);

    }

    @Override
    public void initialize() {

        m_shooter.resetEncoders();

    }

    @Override
    public void execute() {

        m_shooter.shoot(m_rpm);

        if(m_shooter.rightFalconAtSetpoint(m_rpm) && m_shooter.leftFalconAtSetpoint(m_rpm)) {
            m_driveController.setRumble(Constants.DRIVER_RUMBLE, Constants.DRIVER_RUMBLE_PERCENT);
            m_opController.setRumble(Constants.OP_RUMBLE, Constants.OP_RUMBLE_PERCENT);
        }

    }


    
}
