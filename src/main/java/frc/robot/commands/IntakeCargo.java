package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeCargo extends CommandBase {

    private final Intake m_intake;
    private final XboxController m_opController;
    private final int m_intakeAxis;
    
    public IntakeCargo(Intake intake, XboxController opController, int intakeAxis) {

        m_intake = intake;
        m_opController = opController;
        m_intakeAxis = intakeAxis;

        addRequirements(m_intake);

    }

    @Override
    public void execute() {
       
        m_intake.intakeCargo((m_opController.getRawAxis(m_intakeAxis)) * -1);

    }

}
