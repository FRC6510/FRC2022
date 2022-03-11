// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Group.Group.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight.Limelight;
import frc.robot.subsystems.Shooter.Shooter;

/** An example command that uses an example subsystem. */
public class ShootBall extends CommandBase {
  private final Shooter m_shooter;
  private final double m_FrontVel;
  private final double m_BackVel;

  public ShootBall(Shooter shooter, double FrontVel, double BackVel) {
    m_shooter = shooter;
    m_FrontVel= FrontVel;
    m_BackVel= BackVel;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if((Limelight.BackSpeed > 0) && (Limelight.FrontSpeed > 0))
      m_shooter.spin_shooter(Limelight.FrontSpeed,Limelight.BackSpeed);
    else
      m_shooter.spin_shooter(9000,7000);
    
    //SmartDashboard.putNumber("vel1", m_FrontVel);

  }
    
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop_shooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
