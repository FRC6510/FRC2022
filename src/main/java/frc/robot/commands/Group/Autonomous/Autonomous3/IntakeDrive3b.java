// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Group.Autonomous.Autonomous3;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.commands.Profiled2dMovement;
import frc.robot.commands.Group.Group.Feeder.IndexFirstBall;
import frc.robot.commands.Group.Group.Intake.IntakeBall;
import frc.robot.commands.Group.Group.Intake.IntakeMaster;
import frc.robot.commands.Group.Group.Intake.IntakeOut;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainConstants;
import frc.robot.subsystems.Feeder;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeDrive3b extends ParallelCommandGroup {

  private final Drivetrain drivetrain = new Drivetrain();

  public IntakeDrive3b (Drivetrain drivetrain, Intake intake, Feeder feeder) {

    addCommands(
      new Profiled2dMovement(drivetrain, DrivetrainConstants.movementParameters, new Pose2d(0.01, -2.57, Rotation2d.fromDegrees(-140))),
      new IntakeMaster(intake, feeder).withTimeout(3) //used to be 5 but might take too long

         
      );

  }
}
