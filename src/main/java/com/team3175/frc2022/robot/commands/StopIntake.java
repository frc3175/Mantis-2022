package com.team3175.frc2022.robot.commands;

import com.team3175.frc2022.robot.Constants;
import com.team3175.frc2022.robot.subsystems.Actuators;
import com.team3175.frc2022.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class StopIntake extends ParallelCommandGroup {

    private Intake m_intake;
    private Actuators m_actuators;
    private XboxController m_driverController;

    public StopIntake(Intake intake, Actuators actuators, XboxController driveController) {

        m_intake = intake;
        m_actuators = actuators;
        m_driverController = driveController;

        //addRequirements(m_intake, m_actuators);
        
    }

    @Override
    public void execute() {
        addCommands(new ActuateIntake(m_actuators),
                    new IntakeCargo(m_intake, Constants.INTAKE_SPEED, m_driverController)
                   );  
    }
    
    
}
