// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Group.Autonomous;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class setRobotPosition extends CommandBase {

    static WPI_Pigeon2 imu = new WPI_Pigeon2(00, "canivore");
    private final Drivetrain m_drivetrain;
    private Pose2d m_startPos;
    private Rotation2d m_startR;

  /** Creates a new setRobotPosition. */
  public setRobotPosition(Drivetrain drivetrain, Pose2d startPos, Rotation2d startR) {
    //SmartDashboard.putNumber("xPos", Drivetrain.driveOdometry.getPoseMeters().getTranslation().getX());
    //SmartDashboard.putNumber("yPos", driveOdometry.getPoseMeters().getTranslation().getY());
    m_drivetrain = drivetrain;
    m_startPos = startPos;
    //m_pose = pose;
    m_startR = startR;
  
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  /*public void setRobotPosition(Pose2d pose2d, Rotation2d gyroAngle) {
    imu.setYaw(-gyroAngle.getDegrees());
    driveOdometry.resetPosition(pose2d, gyroAngle);
}*/

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.setRobotPosition(m_startPos, m_startR);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
