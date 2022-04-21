// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
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
  private static TalonFX climbMotor;
  private static Solenoid climber_pneu;
  private static Solenoid hook_pneu;
  DigitalInput climberSensor = new DigitalInput(1);
  double goHome = -8000; //6000
  double fullExtend = 201731;//202331



  //private static final double velocitykp = 0, velocityki = 0, velocitykd = 0;

  public Climber(){


    climbMotor = new TalonFX(44, "canivore");
    climber_pneu = new Solenoid(PneumaticsModuleType.CTREPCM, 6);
    hook_pneu = new Solenoid(PneumaticsModuleType.CTREPCM, 7);
    
    init_climber();

    climbMotor.setInverted(true);
    climbMotor.setNeutralMode(NeutralMode.Brake); //stop mode
    //climbMotor.configOpenloopRamp(0.3); //ramp acceleration
    //Intake.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor); <-- What's this?
    //climber.configStatorCurrentLimit(true);
    //climbMotor.configPeakCurrentLimit(20); // don't activate current limit until current
    //climbMotor.configPeakCurrentDuration(100); // ... for at least 100 ms
    //climber.configContinuousCurrentLimit(20); // once current-limiting is actived, hold at
  
    //enabled | Limit(amp) | Trigger Threshold(amp) | Trigger Threshold Time(s)  
    //climbMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,20,25,1.0)); 
    //climbMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,20,100,20));
    //climbMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,10,15,0.5));

    climber_pneu.set(false);

    climbMotor.setSelectedSensorPosition(0); //MAX POSITION -199070
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println("Climber Position" + climbMotor.getSelectedSensorPosition());
    System.out.println("Climber Error" + climbMotor.getClosedLoopError());
    SmartDashboard.putNumber("clim pos", climbMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("clim error", climbMotor.getClosedLoopError());
  }

  public void spin_climber(){

    if (!sense_climber() || climbMotor.getSelectedSensorPosition()<0){
      stop_climber();
    }
    else{
      climbMotor.set(ControlMode.PercentOutput,-0.4); //speed
    }
   
  }

  public void reverse_climber(){

    if ( climbMotor.getSelectedSensorPosition()<(140000)){ //202331 for max, 190000 for usual number
      climbMotor.set(ControlMode.PercentOutput,0.7); //speed
    }
    else{
      stop_climber();
    }
  }

  public void fullextend_climber(){
			climbMotor.set(TalonFXControlMode.MotionMagic, fullExtend);
  }

  public void goHome_climber(){
			climbMotor.set(TalonFXControlMode.MotionMagic, goHome);
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
    climbMotor.set(ControlMode.PercentOutput,0);
  }

  public boolean sense_climber(){
    return climberSensor.get();
  }

  public void climblog(){
    //SmartDashboard.putBoolean("climberSensor", climberSensor.get());
    //SmartDashboard.putNumber("Climber", climber.getSelectedSensorPosition());
  }

  public void init_climber(){
        /* Factory Default all hardware to prevent unexpected behaviour */
        climbMotor.configFactoryDefault();

		climbMotor.setInverted(true); //direction
		climbMotor.setNeutralMode(NeutralMode.Brake); //stop mode
		
		/* Config neutral deadband to be the smallest possible */
		climbMotor.configNeutralDeadband(0.001);


		/* Config sensor used for Primary PID [Velocity] */
    climbMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                                            CConstants.kPIDLoopIdx, CConstants.kTimeoutMs);

    climbMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, CConstants.kTimeoutMs);
		climbMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, CConstants.kTimeoutMs);


		/* Config the peak and nominal outputs */
		climbMotor.configNominalOutputForward(0, CConstants.kTimeoutMs);
		climbMotor.configNominalOutputReverse(0, CConstants.kTimeoutMs);
		climbMotor.configPeakOutputForward(1, CConstants.kTimeoutMs);
		climbMotor.configPeakOutputReverse(-1, CConstants.kTimeoutMs);

		/* Config the Velocity closed loop gains in slot0 */
		climbMotor.config_kF(CConstants.kPIDLoopIdx, CConstants.kGains_Velocit.kF, CConstants.kTimeoutMs);
		climbMotor.config_kP(CConstants.kPIDLoopIdx, CConstants.kGains_Velocit.kP, CConstants.kTimeoutMs);
		climbMotor.config_kI(CConstants.kPIDLoopIdx, CConstants.kGains_Velocit.kI, CConstants.kTimeoutMs);
		climbMotor.config_kD(CConstants.kPIDLoopIdx, CConstants.kGains_Velocit.kD, CConstants.kTimeoutMs);

    climbMotor.configMotionCruiseVelocity(19337.5, CConstants.kTimeoutMs);
		climbMotor.configMotionAcceleration(40000, CConstants.kTimeoutMs);

    climbMotor.setSelectedSensorPosition(0, CConstants.kPIDLoopIdx, CConstants.kTimeoutMs);

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  
}
