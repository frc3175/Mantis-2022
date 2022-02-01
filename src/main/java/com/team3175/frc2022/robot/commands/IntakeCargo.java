package com.team3175.frc2022.robot.commands;

import com.team3175.frc2022.robot.Constants;
import com.team3175.frc2022.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

@SuppressWarnings("unused")
public class IntakeCargo extends CommandBase {

    private final Intake m_intake;
    private final double m_speed;
    private final XboxController m_driveController;
    
    public IntakeCargo(Intake intake, double speed, XboxController driveController) {

        m_intake = intake;
        m_speed = speed;
        m_driveController = driveController;

        addRequirements(m_intake);

    }

    @Override
    public void execute() {
       
        m_intake.intakeCargo(m_speed);

    }

}
