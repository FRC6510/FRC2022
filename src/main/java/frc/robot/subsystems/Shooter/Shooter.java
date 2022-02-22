// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private static TalonFX ShooterLeft = new TalonFX(7);
  private static TalonFX ShooterRight = new TalonFX(8);
  private static TalonFX ShooterBack = new TalonFX(35);

  double targetVelocity_UnitsPer100ms = 12204; //15255 for 75% speed


  private static final double velocitykp = 0, velocityki = 0, velocitykd = 0;

  public Shooter(){
    initShooter();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void spin_shooter(){

    ShooterLeft.set(TalonFXControlMode.Velocity, targetVelocity_UnitsPer100ms);
		ShooterRight.set(TalonFXControlMode.Velocity, targetVelocity_UnitsPer100ms);

    //ShooterLeft.set(ControlMode.PercentOutput,0.6); //speed
    //ShooterRight.set(ControlMode.PercentOutput,0.6);
    //ShooterBack.set(ControlMode.PercentOutput,0.4);

  }

  public void reverse_shooter(){
    ShooterLeft.set(ControlMode.PercentOutput,-0.5); //speed
    ShooterRight.set(ControlMode.PercentOutput,-0.5);
    ShooterBack.set(ControlMode.PercentOutput,0.5);
  }

  public void stop_shooter(){
    ShooterLeft.set(ControlMode.PercentOutput,0);
    ShooterRight.set(ControlMode.PercentOutput,0);
    ShooterBack.set(ControlMode.PercentOutput,0);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void initShooter(){
    /* Factory Default all hardware to prevent unexpected behaviour */
		ShooterRight.configFactoryDefault();
		ShooterLeft.configFactoryDefault();

		ShooterLeft.setInverted(true); //direction
		ShooterRight.setInverted(false);
		ShooterLeft.follow(ShooterRight); //slave
		ShooterLeft.setNeutralMode(NeutralMode.Coast); //stop mode
		ShooterRight.setNeutralMode(NeutralMode.Coast);
		
		/* Config neutral deadband to be the smallest possible */
		ShooterRight.configNeutralDeadband(0.001);
		ShooterLeft.configNeutralDeadband(0.001);

		/* Config sensor used for Primary PID [Velocity] */
        ShooterRight.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                                            Constants.kPIDLoopIdx, 
											Constants.kTimeoutMs);
		ShooterLeft.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                                            Constants.kPIDLoopIdx, 
											Constants.kTimeoutMs);	

		/* Config the peak and nominal outputs */
		ShooterRight.configNominalOutputForward(0, Constants.kTimeoutMs);
		ShooterRight.configNominalOutputReverse(0, Constants.kTimeoutMs);
		ShooterRight.configPeakOutputForward(1, Constants.kTimeoutMs);
		ShooterRight.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		ShooterLeft.configNominalOutputForward(0, Constants.kTimeoutMs);
		ShooterLeft.configNominalOutputReverse(0, Constants.kTimeoutMs);
		ShooterLeft.configPeakOutputForward(1, Constants.kTimeoutMs);
		ShooterLeft.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		/* Config the Velocity closed loop gains in slot0 */
		ShooterRight.config_kF(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kF, Constants.kTimeoutMs);
		ShooterRight.config_kP(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kP, Constants.kTimeoutMs);
		ShooterRight.config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kI, Constants.kTimeoutMs);
		ShooterRight.config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kD, Constants.kTimeoutMs);

		ShooterLeft.config_kF(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kF, Constants.kTimeoutMs);
		ShooterLeft.config_kP(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kP, Constants.kTimeoutMs);
		ShooterLeft.config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kI, Constants.kTimeoutMs);
		ShooterLeft.config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kD, Constants.kTimeoutMs);
		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
		 */
        // _talon.setSensorPhase(true);
  }


}
