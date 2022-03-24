

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Profiled2dMovement;
import frc.robot.commands.Group.Autonomous.Autonomous1.A1M1;
import frc.robot.commands.Group.Autonomous.Autonomous2.A2M1;
import frc.robot.commands.Group.Autonomous.Autonomous3.A3M1;
import frc.robot.commands.Group.Autonomous.Autonomous4.A4M1;
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
import frc.robot.commands.Group.Group.Feeder.FeedBallForShooter;
import frc.robot.commands.Group.Group.Feeder.IndexFirstBall;
import frc.robot.commands.Group.Group.Feeder.ReverseBall;
import frc.robot.commands.Group.Group.Intake.FullIntake;
import frc.robot.commands.Group.Group.Intake.IntakeBall;
import frc.robot.commands.Group.Group.Intake.IntakeIn;
import frc.robot.commands.Group.Group.Intake.IntakeMaster;
import frc.robot.commands.Group.Group.Intake.IntakeMasterTwoBalls;
import frc.robot.commands.Group.Group.Intake.IntakeOut;
import frc.robot.commands.Group.Group.Intake.ReverseIntake;
import frc.robot.commands.Group.Group.Shooter.ReverseShoot;
import frc.robot.commands.Group.Group.Shooter.ShootBall;
import frc.robot.commands.Group.Group.Shooter.ShootBalls;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Limelight.Limelight;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

  //changing the code yay!! just adding comments  

public class RobotContainer {

  XboxController driverController = new XboxController(0);
  XboxController operatorController = new XboxController(1);
  XboxController climberController = new XboxController(2); 

  // The robot's subsystems and commands are defined here...
  private final Drivetrain drivetrain = new Drivetrain();
  private final Feeder m_feeder = new Feeder();
  private final Shooter m_shooter = new Shooter();
  private final Intake m_intake = new Intake();
  private final Climber m_climber = new Climber();
  private final Limelight m_Limelight = new Limelight();

  public static boolean feeder_sensor = false;
  public static boolean climber_sensor = false;

  public static double  FrontShooterTargetVelocity_Slow = 9000; 
  public static double  BackShooter1TargetVelocity_Slow = 7000;

  public static double  FrontShooterTargetVelocity_Fast = 16500;//16500 
  public static double  BackShooter1TargetVelocity_Fast = 10000;//12000//10000 good//11000

  public static double  FrontShooterTargetVelocity_Auto3a = 9000*1.2; 
  public static double  BackShooter1TargetVelocity_Auto3a = 7000*1.2; 

  public static double  FrontShooterTargetVelocity_Auto3b = 9000*1.4;
  public static double  BackShooter1TargetVelocity_Auto3b = 7000*1.4;

  public static double  AmelieTestFront = 9000*1.68; //15120
  public static double  AmelieTestBack = 7000*1.68; //11760

  public static double shooterVelocity;

  public static final double
		K = 0.38, //constant for speed / distance
		FConstant = 9000,
		BConstant = 7000;

  public static double D, SF, SB ;

  private final Command Red2balls =  new A1M1(m_intake, m_feeder, drivetrain, m_shooter);
  private final Command Red1ball  =  new A2M1(drivetrain, m_feeder, m_shooter);
  private final Command Red3Ball  =  new A3M1( drivetrain, m_intake, m_feeder , m_shooter);
  private final Command Red4ball = new A4M1 (drivetrain, m_intake, m_feeder , m_shooter);

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    
    drivetrain.setDefaultCommand(
      new RunCommand(
        () -> drivetrain.drive( //removed negative
           -DrivetrainConstants.DEADZONEMULTIPLIER*Drivetrain.deadZone(driverController.getLeftY()), //*3.6 all
           -DrivetrainConstants.DEADZONEMULTIPLIER*Drivetrain.deadZone(driverController.getLeftX()),
           -DrivetrainConstants.DEADZONEMULTIPLIER*Drivetrain.deadZone(driverController.getRightX()),
          true), 
        drivetrain)
        );

    m_feeder.setDefaultCommand(new RunCommand(() -> feeder_sensor = m_feeder.sense_ball(), m_feeder));

    m_climber.setDefaultCommand(new RunCommand(() -> m_climber.climblog() , m_climber));  

    m_Limelight.setDefaultCommand(new RunCommand(() -> shooterVelocity = Limelight.UpdateShooterVelocity(), m_Limelight));

    configureButtonBindings();

    m_chooser.setDefaultOption("2 ball auto", Red2balls);
    m_chooser.addOption("1 ball auto", Red1ball);
    m_chooser.addOption("3 ball auto", Red3Ball);
    m_chooser.addOption("4 ball auto", Red4ball);

    SmartDashboard.putData(m_chooser);
    SmartDashboard.putNumber("distance from limelight to goal",frc.robot.subsystems.Limelight.Limelight.distanceFromLimelightToGoalInches);

   
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

    Abutton.whileHeld(new IntakeMasterTwoBalls(m_intake,m_feeder));
    Bbutton.whileHeld(new ShootBall(m_shooter, Limelight.FrontSpeed,14000 ));
    RightBumper.whileHeld(new FeedBallForShooter(m_feeder));
    LeftBumper.whileHeld(new ReverseBall(m_feeder));
    Xbutton.whileHeld(new ReverseIntake(m_intake));
    //Ybutton.toggleWhenPressed()
    //LeftBumper.whenPressed( new A3M1( drivetrain, m_intake, m_feeder , m_shooter));
    //LeftBumper.whenReleased(new A2M1 (drivetrain, m_feeder, m_shooter));
    //LeftBumper.whenReleased ( new A1M1(m_intake, m_feeder, drivetrain, m_shooter));
    //LeftBumper.whenReleased(new Profiled2dMovement(drivetrain, DrivetrainConstants.movementParameters, 
    //new Pose2d(1, 0, Rotation2d.fromDegrees(0))));
    //leftButton.whileHeld((new Profiled2dMovement(drivetrain, DrivetrainConstants.movementParameters, 
    //new Pose2d(0, 0, Rotation2d.fromDegrees(0)))));
    //RightBumper.whileHeld(new ClimberIn(m_climber));
    //LeftBumper.whileHeld(new ClimberOut(m_climber));

    final JoystickButton buttonA = new JoystickButton(driverController,1);
    final JoystickButton buttonB = new JoystickButton(driverController,2);
    final JoystickButton buttonX = new JoystickButton(driverController,3);
    final JoystickButton buttonY = new JoystickButton(driverController,4);
    final JoystickButton BumperLeft = new JoystickButton(driverController,5);
    final JoystickButton BumperRight = new JoystickButton(driverController,6);
    final JoystickButton leftStartButtonDriver = new JoystickButton(driverController,7);
    final JoystickButton joystickButtonRight = new JoystickButton(driverController,10);


    buttonY.whenPressed(() -> drivetrain.resetGyro(),drivetrain); 
    buttonB.whileHeld(new RunCommand(
      () -> drivetrain.drive( //removed negative
         -DrivetrainConstants.SLOWDEADZONEMULTIPLIER*Drivetrain.deadZone(driverController.getLeftY()), //*3.6 all
         -DrivetrainConstants.SLOWDEADZONEMULTIPLIER*Drivetrain.deadZone(driverController.getLeftX()),
         -1*Drivetrain.deadZone(driverController.getRightX()),
        true), 
      drivetrain)
      ); //-1*Drivetrain.deadZone(driverController.getRightX())
    buttonA.whileHeld(new RunCommand(
      () -> drivetrain.drive( //removed negative
        -DrivetrainConstants.SLOWDEADZONEMULTIPLIER*Drivetrain.deadZone(driverController.getLeftY()), //*3.6 all
        -DrivetrainConstants.SLOWDEADZONEMULTIPLIER*Drivetrain.deadZone(driverController.getLeftX()),
        -Limelight.Angle*0.05*1.2,//REMOVE1.5
        true), 
      drivetrain)
      ); //-1*Drivetrain.deadZone(driverController.getRightX())

      joystickButtonRight.whileHeld(new RunCommand(
      () -> drivetrain.drive( //removed negative
        -DrivetrainConstants.SLOWDEADZONEMULTIPLIER*Drivetrain.deadZone(driverController.getLeftY()), //*3.6 all
        -DrivetrainConstants.SLOWDEADZONEMULTIPLIER*Drivetrain.deadZone(driverController.getLeftX()),
        -1*Drivetrain.deadZone(driverController.getRightX()),//REMOVE1.5
        false), 
      drivetrain)
      ); //-1*Drivetrain.deadZone(driverController.getRightX())

      BumperLeft.whileHeld(new ClimberFullExtend(m_climber));
      BumperRight.whileHeld(new ClimberGoHome(m_climber));
        
        final JoystickButton CbuttonA = new JoystickButton(climberController,1);
        final JoystickButton CbuttonB = new JoystickButton(climberController,2);
        final JoystickButton CbuttonX = new JoystickButton(climberController,3);
        final JoystickButton CbuttonY = new JoystickButton(climberController,4);
        final JoystickButton CBumperLeft = new JoystickButton(climberController,5);
        final JoystickButton CBumperRight = new JoystickButton(climberController,6);
        final JoystickButton CleftStartButtonDriver = new JoystickButton(climberController,7);


        CBumperLeft.whenReleased(new OpenHook(m_climber));
        CBumperRight.whenReleased(new CloseHook(m_climber));
        CbuttonB.whenReleased(new ClimberIn(m_climber));
        CbuttonY.whenReleased(new ClimberOut(m_climber));
        
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

   public Command getAutonomousCommand(){
    return m_chooser.getSelected();
   }
}
