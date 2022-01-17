// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class ArcadeDrive extends CommandBase {
  private final Drivetrain m_drivetrain;
  private final DoubleSupplier m_left;
  private final DoubleSupplier m_right;


 
  /**
   * 
   * @param speed
   * @param rotation
   * @param drivetrain
   */

  /** Creates a new ArcadeDrive. */
  public ArcadeDrive(DoubleSupplier speed, DoubleSupplier rotation, Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    m_left = speed;
    m_right = rotation;
    addRequirements(m_drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrain.drive(m_left.getAsDouble(), m_right.getAsDouble());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
