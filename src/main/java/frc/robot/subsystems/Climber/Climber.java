// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import org.opencv.features2d.FlannBasedMatcher;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private static TalonFX climber;
  private static Solenoid climber_pneu;
  private static Solenoid hook_pneu;
  DigitalInput climberSensor = new DigitalInput(1);
  double goHome = -6000;
  double fullExtend = 201731;//202331



  //private static final double velocitykp = 0, velocityki = 0, velocitykd = 0;

  public Climber(){

    climber = new TalonFX(44, "canivore");
    climber_pneu = new Solenoid(PneumaticsModuleType.CTREPCM, 7);
    hook_pneu = new Solenoid(PneumaticsModuleType.CTREPCM, 6);
    
    climber.setInverted(true);
    climber.setNeutralMode(NeutralMode.Brake); //stop mode
    climber.configOpenloopRamp(0.3); //ramp acceleration
    //Intake.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor); <-- What's this?
    //climber.configStatorCurrentLimit(true);
    //climber.configPeakCurrentLimit(30); // don't activate current limit until current
    //climber.configPeakCurrentDuration(100); // ... for at least 100 ms
    //climber.configContinuousCurrentLimit(20); // once current-limiting is actived, hold at
    climber_pneu.set(false);

    climber.setSelectedSensorPosition(0); //MAX POSITION -199070
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void spin_climber(){

    if (!sense_climber() || climber.getSelectedSensorPosition()<0){
      stop_climber();
    }
    else{
      climber.set(ControlMode.PercentOutput,-0.4); //speed
    }
   
  }

  public void reverse_climber(){

    if ( climber.getSelectedSensorPosition()<(140000)){ //202331 for max, 190000 for usual number
      climber.set(ControlMode.PercentOutput,0.7); //speed
    }
    else{
      stop_climber();
    }
  }

  public void fullextend_climber(){
			climber.set(TalonFXControlMode.MotionMagic, fullExtend);
  }

  public void goHome_climber(){
			climber.set(TalonFXControlMode.MotionMagic, goHome);
  }

  public void climber_in(){

    climber_pneu.set(false); //speed
  }

  public void climber_out(){
    climber_pneu.set(true);//speed
  }

  public void hook_close(){
    hook_pneu.set(false); //speed
  }

  public void hook_open(){
    hook_pneu.set(true);//speed
  }

  public void stop_climber(){
    climber.set(ControlMode.PercentOutput,0);
  }

  public boolean sense_climber(){
    return climberSensor.get();
  }

  public void climblog(){
    SmartDashboard.putBoolean("climberSensor", climberSensor.get());
    SmartDashboard.putNumber("Climber", climber.getSelectedSensorPosition());
  }

  public void init_climber(){
        /* Factory Default all hardware to prevent unexpected behaviour */
        climber.configFactoryDefault();

		climber.setInverted(true); //direction
		climber.setNeutralMode(NeutralMode.Brake); //stop mode
		
		/* Config neutral deadband to be the smallest possible */
		climber.configNeutralDeadband(0.001);


		/* Config sensor used for Primary PID [Velocity] */
    climber.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                                            CConstants.kPIDLoopIdx, CConstants.kTimeoutMs);

    climber.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, CConstants.kTimeoutMs);
		climber.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, CConstants.kTimeoutMs);


		/* Config the peak and nominal outputs */
		climber.configNominalOutputForward(0, CConstants.kTimeoutMs);
		climber.configNominalOutputReverse(0, CConstants.kTimeoutMs);
		climber.configPeakOutputForward(1, CConstants.kTimeoutMs);
		climber.configPeakOutputReverse(-1, CConstants.kTimeoutMs);

		/* Config the Velocity closed loop gains in slot0 */
		climber.config_kF(CConstants.kPIDLoopIdx, CConstants.kGains_Velocit.kF, CConstants.kTimeoutMs);
		climber.config_kP(CConstants.kPIDLoopIdx, CConstants.kGains_Velocit.kP, CConstants.kTimeoutMs);
		climber.config_kI(CConstants.kPIDLoopIdx, CConstants.kGains_Velocit.kI, CConstants.kTimeoutMs);
		climber.config_kD(CConstants.kPIDLoopIdx, CConstants.kGains_Velocit.kD, CConstants.kTimeoutMs);

    climber.configMotionCruiseVelocity(19337.5, CConstants.kTimeoutMs);
		climber.configMotionAcceleration(40000, CConstants.kTimeoutMs);

    climber.setSelectedSensorPosition(0, CConstants.kPIDLoopIdx, CConstants.kTimeoutMs);

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  
}
