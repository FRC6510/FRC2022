// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Group.Group.Shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.FeedBall;
import frc.robot.commands.FeedBallForShooter;
import frc.robot.commands.ShootBall;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootBalls extends ParallelCommandGroup {
  /** Creates a new ShootBalls. */
  public ShootBalls(Shooter shooter, Feeder feeder) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

    new ShootBall(shooter), new FeedBallForShooter(feeder)


    );
  }
}
