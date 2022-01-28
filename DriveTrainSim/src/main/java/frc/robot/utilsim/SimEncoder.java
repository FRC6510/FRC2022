package frc.robot.utilsim;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;

public class SimEncoder {

  private SimDouble distance;
  private SimDouble speed;

  /**
   * Construct a encoder given a CAN ID.
   *
   * @param name Name of the encoder must be a unique name.
   */
  public SimEncoder(String name) {
    SimDevice device = SimDevice.create("Encoder[" + name + "]");
    distance = device.createDouble("Distance", SimDevice.Direction.kOutput, 0);
    speed = device.createDouble("Speed", SimDevice.Direction.kOutput, 0);
  }
  
  /**
   * Get the speed of the encoder.
   *
   * @return The speed of the encoder in whatever units the user used when
   *     calling setSpeed.
   */
  public double getSpeed() {
    return distance.get();
  }

  /**
   * Get the distance of the encoder.
   *
   * @return The distance of the encoder in whatever units the user used when
   *     calling setDistance.
   */
  public double getDistance() {
    return speed.get();
  }

  /**
   * Set the speed of the encoder.
   *
   * @param speed Speed of the encoder in unit's of the users choice.
   */
  public void setSpeed(double speed) {
    this.speed.set(speed);
  }

  /**
   * Set the distance of the encoder.
   *
   * @param distance Distance of the encoder in unit's of the users choice.
   */
  public void setDistance(double distance) {
    this.distance.set(distance);
  }
}
