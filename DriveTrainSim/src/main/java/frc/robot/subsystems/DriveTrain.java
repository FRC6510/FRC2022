/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.utilsim.SimEncoder;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class DriveTrain extends SubsystemBase {

  // static variable single_instance of type Singleton
   private static DriveTrain driveTrain = null;

  // CONSTANTS //

  private static final int LEFT_MASTER_ID = 1;
  private static final int LEFT_SLAVE_ID = 2;
  private static final int RIGHT_MASTER_ID = 3;
  private static final int RIGHT_SLAVE_ID = 4;
  private static final int PIGEON_ID = 0;

  /*
   * These numbers are an example of a fairly light robot.
   * Note you can utilize results from robot characterization instead of theoretical numbers.
   * https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-characterization/introduction.html#introduction-to-robot-characterization
   */
  final int kCountsPerRev = 2048;     //Encoder counts per revolution of the motor shaft.
  final double kSensorGearRatio = 1;  //Gear ratio is the ratio between the *encoder* and the wheels.  On the AndyMark drivetrain, encoders mount 1:1 with the gearbox shaft.
  final double kGearRatio = 10.71;    //TODO: Measure. Switch kSensorGearRatio to this gear ratio if encoder is on the motor instead of on the gearbox.
  final double kWheelRadiusMetres = 0.0762;     //TODO: Measure.
  private static final double kEffectiveTrackWidthMetres = 0.654; // meters
  final int k100msPerSecond = 10;
  private static final double KV_LINEAR = 0.2395; // TODO: Find out
  private static final double KA_LINEAR = 0.03624; // TODO: Find out
  private static final double KV_ANGULAR = KV_LINEAR; // TODO: Find out
  private static final double KA_ANGULAR = KA_LINEAR; // TODO: Find out

  private static final DCMotor GEAR_BOX = DCMotor.getFalcon500(2);  //2 Falcon 500s on each side of the drivetrain.
  

  private static final LinearSystem<N2, N2, N2> DRIVE_PLANT 
      = LinearSystemId.identifyDrivetrainSystem(KV_LINEAR,
                                                KA_LINEAR,
                                                KV_ANGULAR,
                                                KA_ANGULAR);
  
  private static final edu.wpi.first.math.kinematics.DifferentialDriveKinematics KINEMATICS
      = new DifferentialDriveKinematics(kEffectiveTrackWidthMetres);

  // END CONSTANTS //

  private WPI_TalonFX leftMaster = new WPI_TalonFX(LEFT_MASTER_ID);
  private WPI_TalonFX leftSlave = new WPI_TalonFX(LEFT_SLAVE_ID);
  private WPI_TalonFX rightMaster = new WPI_TalonFX(RIGHT_MASTER_ID);
  private WPI_TalonFX rightSlave = new WPI_TalonFX(RIGHT_SLAVE_ID);

  /* Object for simulated inputs into Talon. */
  TalonFXSimCollection m_leftDriveSim = leftMaster.getSimCollection();
  TalonFXSimCollection m_rightDriveSim = rightMaster.getSimCollection();

  WPI_Pigeon2 m_pigeon = new WPI_Pigeon2(PIGEON_ID);
  /* Object for simulated inputs into Pigeon. */
  BasePigeonSimCollection m_pigeonSim = m_pigeon.getSimCollection();

   // These are set in constuctor because they depend on weather we are in
  // simulation or not.
  //private PigeonIMU gyro;
  private DifferentialDriveOdometry odometry;

  //private SimGyro gyroSim;
  private DifferentialDrivetrainSim sim;
  private Field2d field;

  /**
   * Creates a new DriveTrain.
   */
  private DriveTrain() {
    robotInit();
  }

  // static method to create instance of Singleton class
  public static DriveTrain getInstance() {
    if (driveTrain == null)
    driveTrain = new DriveTrain();

    return driveTrain;
  }

  @Override
  public void periodic() {
    /* if (RobotBase.isSimulation()) {
      odometry.update(gyroSim.getHeading(),
          leftEncoderSim.getDistance(), rightEncoderSim.getDistance());
    } else {
      odometry.update(m_pigeon.getRotation2d(), nativeUnitsToDistanceMeters(leftMaster.getSelectedSensorPosition()),
      nativeUnitsToDistanceMeters(rightMaster.getSelectedSensorPosition()) );
    } */

    odometry.update(m_pigeon.getRotation2d(), nativeUnitsToDistanceMeters(leftMaster.getSelectedSensorPosition()),
      nativeUnitsToDistanceMeters(rightMaster.getSelectedSensorPosition()) );
  }

  @Override
  public void simulationPeriodic() {

    /* Pass the robot battery voltage to the simulated Talon FXs */
    m_leftDriveSim.setBusVoltage(RobotController.getBatteryVoltage());
    m_rightDriveSim.setBusVoltage(RobotController.getBatteryVoltage());

    // To update our simulation, we set motor voltage inputs, update the simulation,
    // and write the simulated positions and velocities to our simulated encoder and gyro.
    // We negate the right side so that positive voltages make the right side
    // move forward.
    //sim.setInputs(leftMaster.get() * RobotController.getBatteryVoltage(),
    //      rightMaster.get() * RobotController.getBatteryVoltage());
    sim.setInputs(m_leftDriveSim.getMotorOutputLeadVoltage(), m_rightDriveSim.getMotorOutputLeadVoltage());
    sim.update(Robot.LOOP_TIME);

    /*
     * Update all of our sensors.
     *
     * Since WPILib's simulation class is assuming +V is forward,
     * but -V is forward for the right motor, we need to negate the
     * position reported by the simulation class. Basically, we
     * negated the input, so we need to negate the output.
     */
    //leftEncoderSim.setDistance(sim.getLeftPositionMeters());
    //leftEncoderSim.setSpeed(sim.getLeftVelocityMetersPerSecond());
    //rightEncoderSim.setDistance(sim.getRightPositionMeters());
    //rightEncoderSim.setSpeed(sim.getRightVelocityMetersPerSecond());
    //gyroSim.setHeading(sim.getHeading());
    
    m_leftDriveSim.setIntegratedSensorRawPosition(
                    distanceToNativeUnits(
                        sim.getLeftPositionMeters()
                    ));
    m_leftDriveSim.setIntegratedSensorVelocity(
                    velocityToNativeUnits(
                        sim.getLeftVelocityMetersPerSecond()
                    ));
    m_rightDriveSim.setIntegratedSensorRawPosition(
                    distanceToNativeUnits(
                        -sim.getRightPositionMeters()
                    ));
    m_rightDriveSim.setIntegratedSensorVelocity(
                    velocityToNativeUnits(
                        -sim.getRightVelocityMetersPerSecond()
                    ));
    m_pigeonSim.setRawHeading(sim.getHeading().getDegrees());

    field.setRobotPose(getPose());
  }

  /**
   * Drive the drivetrain in open loop (No feedback).
   *
   * @param left The power [-1.0, 1.0] to supply to the right side.
   * @param right The power [-1.0, 1.0] to supply to the right side.
   */
  public void driveOpenLoop(double left, double right) {    // left - throttle, right - turn
    //leftMaster.set(left);
    //rightMaster.set(right);
    /* Basic Arcade Drive.  This can optionally be replaced with WPILib's DifferentialDrive class. */
    leftMaster.set(ControlMode.PercentOutput, left, DemandType.ArbitraryFeedForward, right);
    rightMaster.set(ControlMode.PercentOutput, left, DemandType.ArbitraryFeedForward, -right);

    if (RobotBase.isSimulation()) {
      leftSlave.set(leftMaster.get());
      rightSlave.set(rightMaster.get());
    }
  }

  /**
   * Set the robot to a new pose. Note that this will only 
   * set the x and y of the robot.
   *
   * @param newPose The pose to set the robot (x, y) to.
   */
  public void setOdometry(Pose2d newPose) {
    odometry.resetPosition(newPose, m_pigeon.getRotation2d());
  }

  /**
   * Set's the gyro to a new heading.
   *
   * @param newHeading The Rotation2d to set the gyro to.
   */
  public void setGryo(Rotation2d newHeading) {
    if (RobotBase.isSimulation()) {
      //gyroSim.setHeading(newHeading);
      m_pigeonSim.setRawHeading(newHeading.getDegrees());
    } else {
      //gyro.reset();
      //gyro.setAngleAdjustment(newHeading);
      //gyro.setFusedHeading(newHeading.getDegrees());
      m_pigeon.setYaw(newHeading.getDegrees());
    }
  }

  /**
   * Reset gryo and odometry to 0.
   */
  public void resetSensors() {
    setGryo(new Rotation2d());
    setOdometry(new Pose2d());
  }

  /**
   * Get the pose of the robot.
   *
   * @return Pose2d representing the robots position. (x and y are in meters)
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public void robotInit(){

    rightMaster.configFactoryDefault();
    rightSlave.configFactoryDefault();
    rightSlave.follow(rightMaster);
    rightSlave.setInverted(InvertType.FollowMaster);

    leftMaster.configFactoryDefault();
    leftSlave.configFactoryDefault();
    leftSlave.follow(leftMaster);
    leftSlave.setInverted(InvertType.FollowMaster);

    leftMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    rightMaster.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

    // On the robot, the left side is positive forward, so don't change it.
    leftMaster.setInverted(TalonFXInvertType.CounterClockwise);

    // On the robot, the right side output needs to be inverted so that positive is forward.
    rightMaster.setInverted(TalonFXInvertType.Clockwise);
    
    leftMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1,
        (int) (Robot.LOOP_TIME * 1000));
    rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1,
        (int) (Robot.LOOP_TIME * 1000));

    if (RobotBase.isSimulation()) {
      new SimEncoder("Left Drive");
      new SimEncoder("Right Drive");
      //gyroSim = new SimGyro("PigeonIMU");
      odometry = new DifferentialDriveOdometry(m_pigeon.getRotation2d());

      sim = new DifferentialDrivetrainSim(GEAR_BOX,
                                          kGearRatio,     // Gearing reduction.
                                          2.1,            //MOI of 2.1 kg m^2 (from CAD model).
                                          28,             //Mass of the robot is 28 kg.
                                          kWheelRadiusMetres,
                                          kEffectiveTrackWidthMetres, //Distance between wheels is _ meters.
                                          // The standard deviations for measurement:
                                          // x and y:   0.001m
                                          // heading:   0.001 rad
                                          // l and r velocity: 0.1 m/s
                                          // l and r position: 0.005m
                                          VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));
      
      field = new Field2d();
      SmartDashboard.putData("Field", field);
    } else {
      //gyro = new PigeonIMU(PIGEON_IMU_ID);
      odometry = new DifferentialDriveOdometry(m_pigeon.getRotation2d());
    }
  
  }


  private int distanceToNativeUnits(double positionMeters){
    double wheelRotations = positionMeters/(2 * Math.PI * kWheelRadiusMetres);
    double motorRotations = wheelRotations * kSensorGearRatio;
    int sensorCounts = (int)(motorRotations * kCountsPerRev);
    return sensorCounts;
  }

  private int velocityToNativeUnits(double velocityMetersPerSecond){
    double wheelRotationsPerSecond = velocityMetersPerSecond/(2 * Math.PI * kWheelRadiusMetres);
    double motorRotationsPerSecond = wheelRotationsPerSecond * kSensorGearRatio;
    double motorRotationsPer100ms = motorRotationsPerSecond / k100msPerSecond;
    int sensorCountsPer100ms = (int)(motorRotationsPer100ms * kCountsPerRev);
    return sensorCountsPer100ms;
  }

  private double nativeUnitsToDistanceMeters(double sensorCounts){
    double motorRotations = (double)sensorCounts / kCountsPerRev;
    double wheelRotations = motorRotations / kSensorGearRatio;
    double positionMeters = wheelRotations * (2 * Math.PI * kWheelRadiusMetres);
    return positionMeters;
  }

}
