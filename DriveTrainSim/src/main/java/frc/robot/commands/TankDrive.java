package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import java.util.function.Supplier;

/**
 * This command drives the robot using the tank control
 * layout.
 */
public class TankDrive extends CommandBase {
  private Supplier<Double> left;
  private Supplier<Double> right;
  private DriveTrain drive;

  /**
   * Constructor for DriveTankCommand.
   *
   * @param left The supplier to get left drive power from
   * @param right THe supplier to get right drive power from
   * @param drive The drive subsystem instance to use.
   */
  public TankDrive(Supplier<Double> left,
                          Supplier<Double> right,
                          DriveTrain drive) {
    this.left = left;
    this.right = right;

    this.drive = drive;
    addRequirements(this.drive);
  }

  @Override
  public void execute() {
    drive.driveOpenLoop(left.get(), right.get());
  }

  @Override
  public void end(boolean interrupted) {
    drive.driveOpenLoop(0.0, 0.0);
  }
}
