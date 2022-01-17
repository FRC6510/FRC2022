// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Spindexer;
import frc.robot.commands.Intake.IntakeStop;
import frc.robot.commands.Intake.IntakeSpin;
import frc.robot.commands.Intake.IntakeIn;
import frc.robot.commands.Spindexer.DetectBalls;
import frc.robot.commands.Spindexer.SpindexerSpin;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeBallsStop extends SequentialCommandGroup {
  /** Creates a new IntakeBallsStop. */
  public IntakeBallsStop(Intake intake, Spindexer spindexer) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands( new IntakeIn(intake),new IntakeStop(intake), new DetectBalls(spindexer).withTimeout(1)    );
  }
}

