// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Group.Autonomous.Autonomous1;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.Group.Autonomous.setRobotPosition;
import frc.robot.commands.Group.Group.Drivetrain.Profiled2dMovement;
import frc.robot.commands.Group.Group.Drivetrain.SimpleDrive;
import frc.robot.commands.Group.Group.Feeder.IndexFirstBall;
import frc.robot.commands.Group.Group.Intake.IntakeBall;
import frc.robot.commands.Group.Group.Intake.IntakeMaster;
import frc.robot.commands.Group.Group.Intake.IntakeOut;
import frc.robot.commands.Group.Group.Shooter.ShootBall;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class A1M1 extends SequentialCommandGroup {


  //private final Drivetrain drivetrain = new Drivetrain();
  



  public A1M1(Intake intake,Feeder feeder, Drivetrain drivetrain, Shooter shooter) {
  
    addCommands(
    new IntakeDrive1a(drivetrain, intake, feeder),
    new Profiled2dMovement(drivetrain, DrivetrainConstants.movementParameters, new Pose2d(0.98, 0.75, Rotation2d.fromDegrees(-167))), //84, 0.6, -167 between -150 
    new ShootAndFeed (shooter, feeder, RobotContainer.FrontShooterTargetVelocity_Mid, RobotContainer.BackShooter1TargetVelocity_Mid),
    new Profiled2dMovement(drivetrain, DrivetrainConstants.movementParameters, new Pose2d(1.5, 0.6, Rotation2d.fromDegrees(-130))), //-167
    //new Profiled2dMovement(drivetrain, DrivetrainConstants.movementParameters, new Pose2d(2, 0.6, Rotation2d.fromDegrees(-90))), 
    //new Profiled2dMovement(drivetrain, DrivetrainConstants.movementParameters, new Pose2d(2, 0.9, Rotation2d.fromDegrees(-130))),
    new SimpleDrive(drivetrain, 0, 0, 0, 1),
    new ResetGyro()

    //blue alliance intakeDrive1a x101.5 y85                            
                                                                                                                                                                  
    );
 
  }
}
 