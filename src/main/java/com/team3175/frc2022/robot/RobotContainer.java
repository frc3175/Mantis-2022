// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team3175.frc2022.robot;

import java.util.Map;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.team3175.frc2022.robot.autos.autocommands.PathplannerCommand;
import com.team3175.frc2022.robot.autos.automodes.FigureEightAuto;
import com.team3175.frc2022.robot.autos.automodes.OneBallAuto;
import com.team3175.frc2022.robot.autos.automodes.PPSwerveControllerAuto;
import com.team3175.frc2022.robot.autos.automodes.PathweaverTest;
import com.team3175.frc2022.robot.autos.automodes.TwoBallTerminalCenter;
//import com.team3175.frc2022.robot.autos.automodes.PathplannerTesting;
import com.team3175.frc2022.robot.commands.*;
import com.team3175.frc2022.robot.subsystems.*;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

@SuppressWarnings("unused")
public class RobotContainer {

  private final PathPlannerTrajectory m_trajectory;

  /* Controllers */
  private final XboxController m_driverController = new XboxController(Constants.DRIVER_PORT);
  private final XboxController m_opController = new XboxController(Constants.OPERATOR_PORT);

  /* Drive Axes */
  private final int m_translationAxis = XboxController.Axis.kLeftY.value;
  private final int m_strafeAxis = XboxController.Axis.kLeftX.value;
  private final int m_rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton m_zeroGyro = new JoystickButton(m_driverController, XboxController.Button.kX.value);
  private final JoystickButton m_feedShooter = new JoystickButton(m_driverController, XboxController.Button.kA.value);

  /* Operator Buttons */
  private final JoystickButton m_intakeCargo = new JoystickButton(m_opController, XboxController.Button.kY.value);
  private final JoystickButton m_outtakeCargo = new JoystickButton(m_opController, XboxController.Button.kX.value);
  private final JoystickButton m_shootCargo = new JoystickButton(m_opController, XboxController.Button.kLeftBumper.value);

  /* Subsystems */
  private final SwerveDrivetrain m_swerveDrivetrain = new SwerveDrivetrain();
  private final Intake m_intake = new Intake();
  private final Feeder m_feeder = new Feeder();
  private final Shooter m_shooter = new Shooter();
  private final Actuators m_actuator = new Actuators();

  /* Autos */
  private final Command m_auto = new FigureEightAuto(m_swerveDrivetrain); //Uses manual trajectory generation, no theta updates
  private final Command m_pathweaverAuto = new PathweaverTest(m_swerveDrivetrain); //Uses pathweaver, no theta updates
  //private final Command m_pathplannerAuto = new PathplannerTesting(m_swerveDrivetrain); //Uses pathplanner, no theta updates, commented bc I don't have a command right now that will take my 3 args instead of a controller arg
  private final Command m_pathplannerCommand; //handwritten command, doesn't quite work
  private final Command m_PPSwerveControllerTest = new PPSwerveControllerAuto(m_swerveDrivetrain); //uses PPSwerveControllerCommand, not yet tested
  private final Command m_oneBall = new OneBallAuto(m_shooter, m_feeder, m_swerveDrivetrain);

  /* Trajectories */
  //String pathJSON = "paths/IntakeTrenchLineUp.wpilib.json";
  //public static Trajectory m_trajectory = new Trajectory();

  public RobotContainer(){
    boolean fieldRelative = true;
    boolean openLoop = true;
    m_swerveDrivetrain.setDefaultCommand(new SwerveDrive(m_swerveDrivetrain, m_driverController, m_translationAxis, m_strafeAxis, m_rotationAxis, fieldRelative, openLoop));
    
    m_trajectory = PathPlanner.loadPath("New Path", 8, 5);
    m_pathplannerCommand = new PathplannerCommand(m_swerveDrivetrain, m_trajectory);

    UsbCamera m_camera = CameraServer.startAutomaticCapture();
    m_camera.setResolution(400, 400);

    Shuffleboard.getTab("Drive").add("gyro", m_swerveDrivetrain.getAngle());

    /* Shuffleboard.getTab("Drive")
                .add("intake running", m_intake.isIntakeRunning())
                .withWidget(BuiltInWidgets.kBooleanBox)
                .withProperties(Map.of("colorWhenTrue", "red", "colorWhenFalse", "black"))
                .getEntry(); */

    Shuffleboard.getTab("Drive").add("shooter speed", m_shooter.getLeftVelocityRPM());
    
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
    m_feedShooter.whenPressed(new InstantCommand(() -> m_feeder.feederRunPercentOutput(Constants.FEEDER_PERCENT_OUTPUT)))
                  .whenReleased(new InstantCommand(() -> m_feeder.feederRunPercentOutput(0)));
                  
    /* Operator Buttons */  
    m_intakeCargo.whenPressed(new IntakeCargo(m_intake, Constants.INTAKE_SPEED, m_opController))
                 .whenReleased(new IntakeCargo(m_intake, 0, m_opController));
    m_outtakeCargo.whenPressed(new IntakeCargo(m_intake, Constants.OUTTAKE_SPEED, m_opController))
                  .whenReleased(new IntakeCargo(m_intake, 0, m_opController));
    m_shootCargo.whenPressed(new ShootCargo(m_shooter, Constants.SHOOTER_TARGET_RPM, m_driverController, m_opController))
                .whenReleased(new StopShooter(m_shooter));
    m_intakeCargo.whenPressed(new ActuateIntake(m_actuator))
                 .whenReleased(new ActuateBack(m_actuator));
    //m_intakeCargo.whenPressed(new DeployIntake(m_intake, m_actuator, m_driverController))
                 //.whenReleased(new StopIntake(m_intake, m_actuator, m_driverController));


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous

    //TODO: Create a sendable chooser to select command
    //For now just type out the name of the command to be run

      return m_oneBall;

  }
  
}
