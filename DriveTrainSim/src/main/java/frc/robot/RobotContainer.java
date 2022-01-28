package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilsim.RobotControls;

/**
 * Wraps all the subsystems and commands for the robot.
 */
public class RobotContainer {
  // Note: The robot container is a functional description of the robot.
  // It should not describe how to something but what the robot does
  // when a particular inputs is received.

  ////////////////
  // Subsystems //
  ////////////////
  // Add subsystems
  private final DriveTrain driveTrain;
  private RobotControls controls = new RobotControls(false);
  private Button shifterButton = new Button(controls::getShifterButton);

  private enum DriveType {
    TANK, ARCADE 
  }

  private SendableChooser<DriveType> driveTypeChooser
      = new SendableChooser<>();

  /**
   * Default constructor for RobotContainer.
   */
  public RobotContainer() {
    driveTrain = DriveTrain.getInstance();

    driveTypeChooser.setDefaultOption("Tank", DriveType.TANK);
    SmartDashboard.putData(driveTypeChooser);
    
    configureButtonBindings();
    configureDefaultCommands();
  }

  private void configureButtonBindings() {
    
 
  }

  /**
   * Configure the default commands for each subsystem. The default commands
   * should be the commands run for during tellop.
   */
  private void configureDefaultCommands() {
    
    // Configure the default command to drive based off of what drive system
    // the user currently has selected.
    // Note that we only want to use one drive type in competition for performance.
    driveTrain.setDefaultCommand(new ArcadeDrive(controls::getLeftDriverX,
                                                   controls::getRightDriverX,   // Use RightDriverY for jobstick control, here is for the convenience of keyboard
                                                   driveTrain));
  }

  /**
   * Get the command to run for auto.
   *
   * @return The command to be run for auto.
   */
  public Command getAutonomousCommand() {
    return new PrintCommand("Auto would run now.");
  }
}
