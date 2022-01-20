package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

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

        boolean rightFalconAtSetpoint = (m_rpm - m_shooter.getRightVelocity()) < Constants.SHOOTER_ERROR ? true : false; 
        boolean leftFalconAtSetpoint = (m_rpm - m_shooter.getLeftVelocity()) < Constants.SHOOTER_ERROR ? true : false;

        if(rightFalconAtSetpoint && leftFalconAtSetpoint) {
            m_driveController.setRumble(Constants.DRIVER_RUMBLE, Constants.DRIVER_RUMBLE_PERCENT);
            m_opController.setRumble(Constants.OP_RUMBLE, Constants.OP_RUMBLE_PERCENT);
        }

    }


    
}
