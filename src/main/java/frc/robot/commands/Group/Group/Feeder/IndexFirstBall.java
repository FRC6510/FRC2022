// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Group.Group.Feeder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FeedBall;
import frc.robot.commands.stopFeeder;
import frc.robot.subsystems.Feeder;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IndexFirstBall extends SequentialCommandGroup {


  /** Creates a new IndexFirstBall. */
  public IndexFirstBall(Feeder feeder) {



    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

    new ReverseBall(feeder).withTimeout(0.3), new FeedBall(feeder), new stopFeeder(feeder).withTimeout(1), new ReverseBall(feeder)//.withTimeout()
    

    );
  }
}
