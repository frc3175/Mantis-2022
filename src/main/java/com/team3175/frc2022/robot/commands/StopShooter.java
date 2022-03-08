package com.team3175.frc2022.robot.commands;

import com.team3175.frc2022.robot.subsystems.Shooter;
import com.team3175.frc2022.robot.Constants;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class StopShooter extends CommandBase {
    
    private final Shooter m_shooter;
    private final XboxController m_driverController;
    private final XboxController m_opController;

    /**
     * 
     * Sets the shooter power to 0 and rumble to 0
     * 
     * @param shooter shooter instance
     * @param driverController driver controller object
     * @param opController operator controller object
     * 
     */

    public StopShooter(Shooter shooter, XboxController driverController, XboxController opController) {

        m_shooter = shooter;
        m_driverController = driverController;
        m_opController = opController;

        addRequirements(m_shooter);

    }

    @Override
    public void initialize() {

        m_shooter.resetEncoders();

    }

    @Override
    public void execute() {

        m_shooter.stopShooter();

        m_driverController.setRumble(Constants.DRIVER_RUMBLE_LEFT, 0);
        m_opController.setRumble(Constants.OP_RUMBLE_LEFT, 0);
        m_driverController.setRumble(Constants.DRIVER_RUMBLE_RIGHT, 0);
        m_opController.setRumble(Constants.OP_RUMBLE_RIGHT, 0);

    }

}
