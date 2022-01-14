package frc.robot.autos;

import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrivetrain;
import edu.wpi.first.math.geometry.Translation2d;

public class DriveForwardAuto extends SequentialCommandGroup {

    public DriveForwardAuto(SwerveDrivetrain m_swerveDrivetrain) {

        //configure trajectory with maximum speed and acceleration
        TrajectoryConfig m_config = new TrajectoryConfig(Constants.MAX_SPEED, 
                                                         Constants.AUTO_MAX_ACCELERATION_MPS_SQUARED)
                                                         .setKinematics(Constants.swerveKinematics);
                                                         //TODO: figure out how to plot points and create a line of best fit to calculate max acceleration
                                                         //TODO: ask how to calculate theoretical max acceleration because thats good enough for now

        //Start point
        var startPoint = new Pose2d(0, 0, Rotation2d.fromDegrees(180));

        //Waypoints
        //TODO: consider switching this to quintic splines so that it stops spinning???
        var interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(new Translation2d(Units.feetToMeters(-2), Units.feetToMeters(0)));

        //End point
        var endPoint = new Pose2d(Units.feetToMeters(-4), Units.feetToMeters(0), Rotation2d.fromDegrees(180));

        //creates a trajectory
        var m_trajectory = TrajectoryGenerator.generateTrajectory(startPoint,
                                                                  interiorWaypoints,
                                                                  endPoint,
                                                                  m_config);

        //Creates pid controllers for translation, strafe, and theta
        var m_translationController = new PIDController(Constants.AUTO_P_X_CONTROLLER, 0, 0);
        var m_strafeController = new PIDController(Constants.AUTO_P_Y_CONTROLLER, 0, 0);
        var m_thetaController = new ProfiledPIDController(Constants.AUTO_P_THETA_CONTROLLER, 0, 0, 
                                                        Constants.THETA_CONTROLLER_CONSTRAINTS);
        //enable continuous output because we use falcons and thats what you do apparantly
        m_thetaController.enableContinuousInput(-Constants.MAX_ANGULAR_VELOCITY, Constants.MAX_ANGULAR_VELOCITY);

        //creates the swerve command
        SwerveControllerCommand swerveCommand = 
        new SwerveControllerCommand(m_trajectory, 
                                    m_swerveDrivetrain::getPose, 
                                    Constants.swerveKinematics, 
                                    m_translationController, 
                                    m_strafeController, 
                                    m_thetaController, 
                                    m_swerveDrivetrain::setModuleStates, 
                                    m_swerveDrivetrain);

        //commands to sequentially run in autonomous
        addCommands(
            new InstantCommand(() -> m_swerveDrivetrain.resetOdometry(m_trajectory.getInitialPose())), 
            swerveCommand
        );

    }   

}

