// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Group.Autonomous.Autonomous2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.IntakeOut;
import frc.robot.commands.Profiled2dMovement;
import frc.robot.commands.ShootBall;
import frc.robot.commands.Group.Autonomous.setRobotPosition;
import frc.robot.commands.Group.Autonomous.Autonomous1.IntakeDrive;
import frc.robot.commands.Group.Autonomous.Autonomous1.ShootAndFeed;
import frc.robot.commands.Group.Group.Feeder.IndexFirstBall;
import frc.robot.commands.Group.Group.Intake.IntakeMaster;
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
 
    new Profiled2dMovement(drivetrain, DrivetrainConstants.movementParameters, new Pose2d(-0.7, 0, Rotation2d.fromDegrees(0))),
    new ShootAndFeed2 (shooter, feeder),
    new Profiled2dMovement(drivetrain, DrivetrainConstants.movementParameters, new Pose2d(-2, 0, Rotation2d.fromDegrees(0)))


    );

  }
}
