package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;

public class TurnToTheta extends CommandBase {

    private double m_desiredAngle;
    private SwerveDrivetrain m_drivetrain;
    private Translation2d m_translation; 

    public TurnToTheta(SwerveDrivetrain drivetrain, double angle) {
        m_desiredAngle = angle;
        m_drivetrain = drivetrain;

        addRequirements(m_drivetrain);
    }

    @Override 
    public void execute() {

        double currentAngle = m_drivetrain.getAngle();

        boolean atAngle = Math.abs(currentAngle - m_desiredAngle) < 1;

        boolean positive = m_drivetrain.optimizeTurning(currentAngle, m_desiredAngle);

        m_translation = new Translation2d(0, 0);

        if(!atAngle) {

            if(positive) {
                m_drivetrain.drive(m_translation, (Constants.TURN_IN_PLACE_SPEED * Constants.MAX_ANGULAR_VELOCITY), true, true); 
            } else {
                m_drivetrain.drive(m_translation, -(Constants.TURN_IN_PLACE_SPEED * Constants.MAX_ANGULAR_VELOCITY), true, true);
            }

        }

    }

}
