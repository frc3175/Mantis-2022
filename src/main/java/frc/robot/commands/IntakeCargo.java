package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeCargo extends CommandBase {

    private final Intake m_intake;
    private final double m_speed;
    
    public IntakeCargo(Intake intake, double speed) {

        m_intake = intake;
        m_speed = speed;

        addRequirements(m_intake);

    }

    @Override
    public void execute() {
       
        m_intake.intakeCargo(m_speed);

    }

}
