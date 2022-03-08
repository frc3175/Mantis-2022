package com.team3175.frc2022.robot.commands;

import com.team3175.frc2022.robot.Constants;
import com.team3175.frc2022.robot.subsystems.SwerveDrivetrain;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class StopSwerve extends CommandBase {

    private Translation2d m_translation;
    
    private SwerveDrivetrain m_swerveDrivetrain;

    /**
     * 
     * Stops the drivetrain movement, called at the end of every auto path
     * 
     */

    public StopSwerve(SwerveDrivetrain swerveDrivetrain) {

        m_swerveDrivetrain = swerveDrivetrain;
        addRequirements(m_swerveDrivetrain);

    }

    @Override
    public void execute() {

        m_translation = new Translation2d(0, 0).times(Constants.MAX_SPEED);
        m_swerveDrivetrain.drive(m_translation, 0, true, true);

    }

    @Override
    public boolean isFinished() {
        return true;
    }

}

