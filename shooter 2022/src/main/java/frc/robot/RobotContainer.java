/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.sql.Driver;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Shooter.ShootBalls;
import frc.robot.commands.Shooter.ShooterStop;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.commands.FeedBalls;
import frc.robot.commands.StopBalls;
import frc.robot.commands.Drivetrain.ArcadeDrive;
import frc.robot.commands.Drivetrain.Highgear;
import frc.robot.commands.Drivetrain.Lowgear;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;



/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final Shooter m_shooter = new Shooter();
  private final Feeder m_feeder = new Feeder();
  private final Drivetrain m_drivetrain = new Drivetrain();

  private final Joystick m_joystick = new Joystick(0);
  private final Joystick m_driver_joystick = new Joystick(1);
  
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    m_drivetrain.setDefaultCommand(new ArcadeDrive(() -> m_driver_joystick.getRawAxis(1), 
    () -> -m_driver_joystick.getRawAxis(4), m_drivetrain));

    SmartDashboard.putNumber(null, 125890);


    // once it swapped - couldn't find reason. worked after code was built again
    // Configure the button bindings

   // m_drivetrain.setDefaultCommand(new ArcadeDrive(() -> m_driver_joystick.getRawAxis(1),() -> -m_driver_joystick.getRawAxis(4), m_driver_joystick));



    configureButtonBindings();
  }   

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    //Operator
    
    final JoystickButton one = new JoystickButton(m_joystick,1);
    final JoystickButton two = new JoystickButton(m_joystick,2);
    final JoystickButton three = new JoystickButton(m_joystick,3);
    final JoystickButton seven = new JoystickButton(m_joystick,7);
    final JoystickButton ten = new JoystickButton(m_joystick,10);
    final JoystickButton eleven = new JoystickButton(m_joystick,11);
    final JoystickButton twelve = new JoystickButton(m_joystick,12);
    final JoystickButton nine = new JoystickButton(m_joystick,9);
    final JoystickButton six = new JoystickButton(m_joystick,6);
    final JoystickButton eight = new JoystickButton(m_joystick,8);
    final JoystickButton five = new JoystickButton(m_joystick,5);

    //Driver
    final JoystickButton lButton = new JoystickButton(m_driver_joystick,5);
    final JoystickButton rButton = new JoystickButton(m_driver_joystick,6);
    final JoystickButton aButton = new JoystickButton(m_driver_joystick,1);
    final JoystickButton bButton = new JoystickButton(m_driver_joystick,2);

  //Operator
   
    one.whenPressed(new ShootBalls(m_shooter));// ShooterEighty(m_shooter));
    one.whenReleased(new ShooterStop(m_shooter));
    two.whenPressed(new FeedBalls(m_feeder));// ShooterEighty(m_shooter));
    two.whenReleased(new StopBalls(m_feeder));

  //Driver
    lButton.whenPressed(new Lowgear(m_drivetrain));
    rButton.whenPressed(new Highgear(m_drivetrain));
    //lButton.whenPressed(new ToggleShifting(m_drivetrain));
    


  }


}
