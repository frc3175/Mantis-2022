// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.automodes.PathweaverTest;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  /* Controllers */
  private final XboxController m_driverController = new XboxController(Constants.DRIVER_PORT);
  private final XboxController m_opController = new XboxController(Constants.OPERATOR_PORT);

  /* Drive Axes */
  private final int m_translationAxis = XboxController.Axis.kLeftY.value;
  private final int m_strafeAxis = XboxController.Axis.kLeftX.value;
  private final int m_rotationAxis = XboxController.Axis.kRightX.value;

  /* Operator Axes */
  private final int m_intakeAxis = XboxController.Axis.kLeftTrigger.value;
  private final int m_outtakeAxis = XboxController.Axis.kRightTrigger.value;

  /* Driver Buttons */
  private final JoystickButton m_zeroGyro = new JoystickButton(m_driverController, XboxController.Button.kY.value);
  private final JoystickButton m_turnTo90 = new JoystickButton(m_driverController, XboxController.Button.kB.value);

  /* Operator Buttons */
  private final JoystickButton m_intakeCargo = new JoystickButton(m_opController, XboxController.Axis.kLeftTrigger.value);
  private final JoystickButton m_outtakeCargo = new JoystickButton(m_opController, XboxController.Axis.kRightTrigger.value);

  /* Subsystems */
  private final SwerveDrivetrain m_swerveDrivetrain = new SwerveDrivetrain();
  private final Intake m_intake = new Intake();

  /* Autos */
  //private final Command m_auto = new FigureEightAuto(m_swerveDrivetrain);
  private final Command m_pathweaverAuto = new PathweaverTest(m_swerveDrivetrain);

  /* Trajectories */
  String pathJSON = "paths/IntakeTrenchLineUp.wpilib.json";
  public static Trajectory m_trajectory = new Trajectory();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    boolean fieldRelative = true;
    boolean openLoop = true;
    m_swerveDrivetrain.setDefaultCommand(new SwerveDrive(m_swerveDrivetrain, m_driverController, m_translationAxis, m_strafeAxis, m_rotationAxis, fieldRelative, openLoop));

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(pathJSON);
      m_trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + pathJSON, ex.getStackTrace());
    }
    
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    /* Driver Buttons */
    m_zeroGyro.whenPressed(new InstantCommand(() -> m_swerveDrivetrain.resetGyro()));
    m_turnTo90.whenPressed(new TurnToTheta(m_swerveDrivetrain, -90))
              .whenReleased(new SwerveDrive(m_swerveDrivetrain, m_driverController, m_translationAxis, m_strafeAxis, m_rotationAxis, true, true));
    
    /* Operator Buttons */  
    m_intakeCargo.whenActive(new IntakeCargo(m_intake, m_opController, m_intakeAxis));
    m_outtakeCargo.whenActive(new IntakeCargo(m_intake, m_opController, m_outtakeAxis));


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_pathweaverAuto;
  }

  public static Trajectory getTrajectory() {
    return m_trajectory;
  }
  
}
