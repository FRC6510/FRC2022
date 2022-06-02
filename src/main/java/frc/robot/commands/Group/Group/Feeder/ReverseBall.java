// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Group.Group.Feeder;

import frc.robot.subsystems.Feeder;

import java.util.concurrent.TimeoutException;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ReverseBall extends CommandBase {
  private final Feeder m_feeder;

  public ReverseBall(Feeder feeder) {
    m_feeder = feeder;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_feeder.reverse_feeder();
    //SmartDashboard.putString("reverBall", "isRunning");

  }
    
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_feeder.stop_feeder();
    //SmartDashboard.putString("reverBall", "isNotRunning");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_feeder.sense_ball() == true){

      return false;
     } else {
       return true;
     }
 
  }
}
