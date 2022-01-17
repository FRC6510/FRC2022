Feeder.java// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Feeder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Spindexer;

public class FeederUp extends CommandBase {
  private final Feeder m_feeder;

  /** Creates a new FeederUp. */
  public FeederUp(Feeder m_feeder2) {
    m_feeder = m_feeder2;
    addRequirements(m_feeder);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_feeder.spin_feeder_lift();
    m_feeder.spin_feeder_main();
    m_feeder.flip_out();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
