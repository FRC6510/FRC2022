// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Group.Group.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter.Shooter;

/** An example command that uses an example subsystem. */
public class ShootBallAuto extends CommandBase {
  private final Shooter m_shooter;
  public static double  m_FrontVel;
  public static double  m_BackVel;




  public ShootBallAuto(Shooter shooter, double FrontVel, double BackVel) {
    m_shooter = shooter;
    m_FrontVel = FrontVel;
    m_BackVel = BackVel;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.spin_shooter(m_FrontVel, m_BackVel);
  }
    
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //m_shooter.stop_shooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_shooter.TargetReached()){
      return true;
    }
    else{
      return false;
    }
  }
}
