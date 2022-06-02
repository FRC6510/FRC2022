// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Group.Autonomous.Autonomous2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.Group.Autonomous.setRobotPosition;
import frc.robot.commands.Group.Autonomous.Autonomous1.ShootAndFeed;
import frc.robot.commands.Group.Group.Drivetrain.Profiled2dMovement;
import frc.robot.commands.Group.Group.Drivetrain.SimpleDrive;
import frc.robot.commands.Group.Group.Feeder.IndexFirstBall;
import frc.robot.commands.Group.Group.Intake.IntakeBall;
import frc.robot.commands.Group.Group.Intake.IntakeMaster;
import frc.robot.commands.Group.Group.Intake.IntakeOut;
import frc.robot.commands.Group.Group.Shooter.ShootBall;
import frc.robot.commands.ResetGyro;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class A2M1 extends SequentialCommandGroup {

  private final Drivetrain drivetrain = new Drivetrain();


  public A2M1(Drivetrain drivetrain, Feeder feeder, Shooter shooter) {
  
    addCommands(
    //new SimpleDrive(drivetrain, 0, 0, 0, 8),
    new Profiled2dMovement(drivetrain, DrivetrainConstants.movementParameters, new Pose2d(-0.7, 0, Rotation2d.fromDegrees(0))),
    new ShootAndFeed (shooter, feeder, RobotContainer.FrontShooterTargetVelocity_Slow, RobotContainer.BackShooter1TargetVelocity_Slow),
    new Profiled2dMovement(drivetrain, DrivetrainConstants.movementParameters, new Pose2d(-2, 0, Rotation2d.fromDegrees(0))),
    new ResetGyro()

    );

  }
}
