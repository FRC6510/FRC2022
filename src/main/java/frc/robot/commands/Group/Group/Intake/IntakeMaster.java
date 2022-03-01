// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Group.Group.Intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.commands.IntakeBall;
import frc.robot.commands.IntakeOut;
import frc.robot.commands.Group.Group.Feeder.IndexFirstBall;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Feeder;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeMaster extends ParallelCommandGroup {
  /** Creates a new Intake. */
  public IntakeMaster(Intake intake,Feeder feeder) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //addCommands(new ScheduleCommand(intake, feeder), new IndexFirstBall(feeder));
    addCommands(
      new IntakeBall(intake,feeder), new ScheduleCommand( new IndexFirstBall(feeder)));

  }
}
