package frc.robot.utilsim;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.geometry.Rotation2d;

public class SimGyro {

  private SimDouble heading; 

  /**
   * Construct a sim device.
   *
   * @param name Name of the gyro. Must be different from any other SimGyro
   */
  public SimGyro(String name) {
    SimDevice device = SimDevice.create("Gyro[" + name + "]");
    heading = device.createDouble("Heading", SimDevice.Direction.kOutput, 0);
  }
  
  /**
   * Set the SimGyro's heading.
   *
   * @param heading Rotation2d representing the heading.
   */
  public void setHeading(Rotation2d heading) {
    this.heading.set(heading.getRadians());
  }

  /**
   * Get the heading of the SimGyro.
   *
   * @return Current heading of the SimGyro as a Rotation2d
   */
  public Rotation2d getHeading() {
    return new Rotation2d(heading.get());
  }
}
