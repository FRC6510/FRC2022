// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class IntakeBall extends CommandBase {
  private final Intake m_intake;
  private final Feeder m_feeder;

  public IntakeBall(Intake intake, Feeder feeder) {
    m_intake = intake;
    m_feeder = feeder;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake, m_feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {



  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_intake.spin_intake();

  }
    
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stop_intake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_intake.sense_intake() && m_feeder.sense_ball()){
      return false;
    } else{
      return true;
    }
    
  }
}
