package com.team3175.frc2022.robot.commands;

import com.team3175.frc2022.robot.Constants;
import com.team3175.frc2022.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

@SuppressWarnings("unused")
public class IntakeCargo extends CommandBase {

    private final Intake m_intake;
    private final double m_speed;
    private final XboxController m_driveController;
    private boolean intakeRunning;
    
    /**
     * 
     * Runs the intake and actuates the pistons down when a button is held
     * 
     * @param intake intake instance
     * @param speed speed to set the intake motor to
     * @param driveController driver controller object
     * 
     */

    public IntakeCargo(Intake intake, double speed, XboxController driveController) {

        m_intake = intake;
        m_speed = speed;
        m_driveController = driveController;

        addRequirements(m_intake);

    }

    @Override
    public void execute() {

        intakeRunning = true;
       
        m_intake.intakeCargo(m_speed);

    }

    @Override
    public void end(boolean isFinished) {

        intakeRunning = false;

    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
