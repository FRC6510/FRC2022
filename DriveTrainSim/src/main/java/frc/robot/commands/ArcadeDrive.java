/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class ArcadeDrive extends CommandBase {
  private DriveTrain drive;
  private Supplier<Double> throttle;
  private Supplier<Double> steer;

  /**
   * Constructor for DriveArcade Command.
   *
   * @param throttle The supplier for the command to read the throttle value.
   * @param steer The supplier for the command to read the steering value.
   * @param drive The instance of drive to use.
   */
  public ArcadeDrive(Supplier<Double> throttle,
                            Supplier<Double> steer,
                            DriveTrain drive) {
    this.throttle = throttle;
    this.steer = steer;

    this.drive = drive;
    addRequirements(this.drive);
  }

  @Override
  public void execute() {
    double throttleVal = Math.abs(throttle.get()) > 0.1 ? throttle.get() : 0;
    double steerVal = Math.abs(steer.get()) > 0.1 ? steer.get() : 0; 
    drive.driveOpenLoop(throttleVal + steerVal,
                             throttleVal - steerVal);
    //System.out.print("Throttle =" + throttleVal + ", steer = " + steerVal);
  }

  @Override
  public void end(boolean interrupted) {
    drive.driveOpenLoop(0.0, 0.0);
  }
}
