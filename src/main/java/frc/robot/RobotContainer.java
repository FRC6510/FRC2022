// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.commands.ClimbReverse;
import frc.robot.commands.ClimbRobot;
import frc.robot.commands.ClimberIn;
import frc.robot.commands.ClimberOut;
import frc.robot.commands.CloseHook;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.FeedBall;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.OpenHook;
import frc.robot.commands.ReverseBall;
import frc.robot.commands.ReverseClimb;
import frc.robot.commands.ReverseIntake;
import frc.robot.commands.ReverseShoot;
import frc.robot.commands.ShootBall;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
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
  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain;

  private final Feeder m_feeder = new Feeder();
  private final Shooter m_shooter = new Shooter();
  private final Intake m_intake = new Intake();
  private final Climber m_climber = new Climber();

  XboxController driverController = new XboxController(0);
  XboxController operatorController = new XboxController(1); 



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drivetrain = new Drivetrain();
    // Configure the button bindings
    configureButtonBindings();


    drivetrain.setDefaultCommand(
      new RunCommand(
        () -> drivetrain.drive( //removed negative
          Drivetrain.deadZone(-driverController.getLeftY()),
          Drivetrain.deadZone(-driverController.getLeftX()),
          Drivetrain.deadZone(-driverController.getRightX()),
          true), 
        drivetrain)
    );

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    //final JoystickButton RightBumper = new JoystickButton(operatorController,6);
    //RightBumper.whileHeld(new ReverseBall(m_feeder));

    //final JoystickButton Abutton = new JoystickButton(operatorController,1);
    //Abutton.whileHeld(new ClimbRobot(m_climber));

    final JoystickButton Bbutton = new JoystickButton(operatorController,2);
    Bbutton.whileHeld(new ShootBall(m_shooter));
    //final JoystickButton Xbutton = new JoystickButton(operatorController,3);
    //Xbutton.whileHeld(new ReverseClimb(m_climber));

    final JoystickButton Ybutton = new JoystickButton(operatorController,1);
    Ybutton.whileHeld(new ReverseShoot(m_shooter));
    
    final JoystickButton RightBumper = new JoystickButton(operatorController,5);
    RightBumper.whileHeld(new FeedBall(m_feeder));

    final JoystickButton LeftBumper = new JoystickButton(operatorController,3);
    LeftBumper.whileHeld(new IntakeBall(m_intake));
    
    final JoystickButton startButton = new JoystickButton(driverController,XboxController.Button.kY.value);
    startButton.whenPressed(() -> drivetrain.resetGyro(),drivetrain);

    //final JoystickButton leftTriggButton = new JoystickButton(operatorController, 0);
    //leftTriggButton.whileHeld(new ReverseIntake(m_intake)); 

    final JoystickButton rightTriggButton = new JoystickButton(operatorController,6);
    rightTriggButton.whileHeld(new ReverseBall(m_feeder));
    
    //final JoystickButton buttonX = new JoystickButton(driverController,3);
    //buttonX.whileHeld(new ClimberIn(m_climber));

    //final JoystickButton buttonB = new JoystickButton(driverController,2);
    //buttonX.whileHeld(new ClimberOut(m_climber));

    //final JoystickButton leftBumper = new JoystickButton(driverController,5);
    //buttonX.whileHeld(new OpenHook(m_climber));

    //final JoystickButton rightBumper = new JoystickButton(driverController,6);
    //buttonX.whileHeld(new CloseHook(m_climber));

    final JoystickButton leftButton = new JoystickButton(operatorController,9);
    leftButton.whileHeld(new ClimbRobot(m_climber));

    final JoystickButton rightButton = new JoystickButton(operatorController,10);
    rightButton.whileHeld(new ClimbReverse(m_climber));
    
;
  // new JoystickButton(driverController, Button.kA.value)
  // .whenPressed(() -> drivetrain.turnLFmodule(5)).whenReleased(() -> drivetrain.turnLFmodule(0));
  // new JoystickButt    - on(driverController, Button.kB.value)
  // .whenPressed(() -> drivetrain.turnLFmodule(30)).whenReleased(() -> drivetrain.turnLFmodule(0));
  // new JoystickButton(driverController, Button.kX.value)
  // .whenPressed(() -> drivetrain.turnLFmodule(90)).whenReleased(() -> drivetrain.turnLFmodule(0));
  // new JoystickButton(driverController, Button.kY.value)
  // .whenPressed(() -> drivetrain.turnLFmodule(-120)).whenReleased(() -> drivetrain.turnLFmodule(0));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

}
