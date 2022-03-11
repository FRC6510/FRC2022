// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Group.Group.Feeder;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.Group.Group.Intake.ReverseIntake;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ReverseIntakeFeeder extends ParallelCommandGroup {
  /** Creates a new ReverseIntakeFeeder. */
  public ReverseIntakeFeeder(Feeder m_feeder,Intake m_intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
   
    addCommands(
      new ReverseWrongBall(m_feeder),
      new ReverseIntake(m_intake)
         
      );
  }
}
