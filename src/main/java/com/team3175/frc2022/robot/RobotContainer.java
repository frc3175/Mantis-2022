// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3175.frc2022.robot;

import com.team3175.frc2022.robot.autos.automodes.PathweaverTest;
import com.team3175.frc2022.robot.commands.*;
import com.team3175.frc2022.robot.subsystems.*;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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

  /* Driver Buttons */
  private final JoystickButton m_zeroGyro = new JoystickButton(m_driverController, XboxController.Button.kY.value);
  private final JoystickButton m_turnTo90 = new JoystickButton(m_driverController, XboxController.Button.kB.value);
  private final JoystickButton m_feedShooter = new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value);

  /* Operator Buttons */
  private final JoystickButton m_intakeCargo = new JoystickButton(m_opController, XboxController.Axis.kLeftTrigger.value);
  private final JoystickButton m_outtakeCargo = new JoystickButton(m_opController, XboxController.Axis.kRightTrigger.value);
  private final JoystickButton m_shootCargo = new JoystickButton(m_opController, XboxController.Button.kA.value);
  private final JoystickButton m_actuaterIntake = new JoystickButton(m_opController, XboxController.Button.kB.value);

  /* Subsystems */
  private final SwerveDrivetrain m_swerveDrivetrain = new SwerveDrivetrain();
  private final Intake m_intake = new Intake();
  private final Feeder m_feeder = new Feeder();
  private final Shooter m_shooter = new Shooter();
  private final Actuaters m_actuater = new Actuaters();

  /* Autos */
  //private final Command m_auto = new FigureEightAuto(m_swerveDrivetrain);
  private final Command m_pathweaverAuto = new PathweaverTest(m_swerveDrivetrain);

  /* Trajectories */
  //String pathJSON = "paths/IntakeTrenchLineUp.wpilib.json";
  //public static Trajectory m_trajectory = new Trajectory();


  // The container for the robot. Contains subsystems, OI devices, and commands. 
 
  public RobotContainer(){
    boolean fieldRelative = true;
    boolean openLoop = true;
    m_swerveDrivetrain.setDefaultCommand(new SwerveDrive(m_swerveDrivetrain, m_driverController, m_translationAxis, m_strafeAxis, m_rotationAxis, fieldRelative, openLoop));
    
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
    m_feedShooter.whenPressed(new FeedShooter(m_feeder, Constants.TARGET_FEEDER_RPM));
    
    /* Operator Buttons */  
    m_intakeCargo.whenActive(new IntakeCargo(m_intake, Constants.INTAKE_SPEED));
    m_outtakeCargo.whenActive(new IntakeCargo(m_intake, Constants.OUTTAKE_SPEED));
    m_shootCargo.whenPressed(new ShootCargo(m_shooter, Constants.SHOOTER_TARGET_RPM, m_driverController, m_opController));
    m_actuaterIntake.whenPressed(new ActuateIntake(m_actuater));


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
  
}
