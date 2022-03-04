

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.FeedBall;
import frc.robot.commands.FeedBallForShooter;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeOut;
import frc.robot.commands.ReverseBall;
import frc.robot.commands.ReverseIntake;
import frc.robot.commands.ReverseShoot;
import frc.robot.commands.ShootBall;
import frc.robot.commands.Group.Group.Climber.ClimbReverse;
import frc.robot.commands.Group.Group.Climber.ClimbRobot;
import frc.robot.commands.Group.Group.Climber.ClimberFullExtend;
import frc.robot.commands.Group.Group.Climber.ClimberGoHome;
import frc.robot.commands.Group.Group.Climber.ClimberIn;
import frc.robot.commands.Group.Group.Climber.ClimberOut;
import frc.robot.commands.Group.Group.Climber.CloseHook;
import frc.robot.commands.Group.Group.Climber.OpenHook;
import frc.robot.commands.Group.Group.Climber.ReverseClimb;
import frc.robot.commands.Group.Group.Climber.SenseClimber;
import frc.robot.commands.Group.Group.Feeder.IndexFirstBall;
import frc.robot.commands.Group.Group.Intake.FullIntake;
import frc.robot.commands.Group.Group.Intake.IntakeMaster;
import frc.robot.commands.Group.Group.Shooter.ShootBalls;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.drivetrain.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  XboxController driverController = new XboxController(0);
  XboxController operatorController = new XboxController(1); 

  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();
  private final Feeder m_feeder = new Feeder();
  private final Shooter m_shooter = new Shooter();
  private final Intake m_intake = new Intake();
  private final Climber m_climber = new Climber();
 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    
    drivetrain.setDefaultCommand(
      new RunCommand(
        () -> drivetrain.drive( //removed negative
          Drivetrain.deadZone(-3.6*driverController.getLeftY()),
          Drivetrain.deadZone(-3.6*driverController.getLeftX()),
          Drivetrain.deadZone(-3.6*driverController.getRightX()),
          true), 
        drivetrain)
    );

    configureButtonBindings();

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    final JoystickButton Abutton = new JoystickButton(operatorController,1);
    final JoystickButton Bbutton = new JoystickButton(operatorController,2);
    final JoystickButton Xbutton = new JoystickButton(operatorController,3);
    final JoystickButton Ybutton = new JoystickButton(operatorController,4);
    final JoystickButton LeftBumper = new JoystickButton(operatorController,5);
    final JoystickButton RightBumper = new JoystickButton(operatorController,6);  
    final JoystickButton leftStartButton = new JoystickButton(operatorController,7);
    final JoystickButton leftButton = new JoystickButton(operatorController,9);
    final JoystickButton rightButton = new JoystickButton(operatorController,10);

    Abutton.whileHeld(new IntakeMaster(m_intake,m_feeder));
    Bbutton.whileHeld(new ShootBall(m_shooter));
    RightBumper.whileHeld(new FeedBallForShooter(m_feeder));
    Xbutton.whileHeld(new ClimberGoHome(m_climber));
    Ybutton.whileHeld(new ClimberFullExtend(m_climber));

    
    final JoystickButton buttonA = new JoystickButton(driverController,1);
    final JoystickButton buttonB = new JoystickButton(driverController,2);
    final JoystickButton buttonX = new JoystickButton(driverController,3);
    final JoystickButton buttonY = new JoystickButton(driverController,4);
    final JoystickButton BumperLeft = new JoystickButton(driverController,5);
    final JoystickButton BumperRight = new JoystickButton(driverController,6);
    final JoystickButton leftStartButtonDriver = new JoystickButton(driverController,7);
    
    buttonY.whenPressed(() -> drivetrain.resetGyro(),drivetrain); 
    buttonB.whileHeld(new ClimberOut(m_climber));
    //buttonX.whenPressed(new ClimberIn(m_climber));
    BumperLeft.whileHeld(new OpenHook(m_climber));
    //BumperRight.whileHeld(new CloseHook(m_climber));
    buttonA.whileHeld(new ClimbRobot(m_climber));
    leftStartButtonDriver.whileHeld(new ReverseClimb(m_climber));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

}
