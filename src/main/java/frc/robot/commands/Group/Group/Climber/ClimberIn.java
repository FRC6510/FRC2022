// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Group.Group.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber.Climber;

/** An example command that uses an example subsystem. */
public class ClimberIn extends CommandBase {
  private final Climber m_climber;

  public ClimberIn(Climber climber) {
    m_climber = climber;
    //  addRequirements() here to declare subsystem dependencies.
    addRequirements(m_climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climber.climber_in();
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    

  }
    
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
