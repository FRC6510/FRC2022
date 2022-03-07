// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Group.Autonomous.Autonomous1;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FeedBall;
import frc.robot.commands.stopFeeder;
import frc.robot.commands.Group.Group.Feeder.ReverseBall;
import frc.robot.commands.Group.Group.Shooter.ShootBallAuto;
import frc.robot.commands.Group.Group.Shooter.ShootBalls;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootAndFeed extends SequentialCommandGroup {


  /** Creates a new IndexFirstBall. */
  public ShootAndFeed (Shooter shooter, Feeder feeder) {


    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

    new ShootBallAuto(shooter), new ShootBalls(shooter, feeder).withTimeout(3)
    
    );
  }
}
