/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.Shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
 
  WPI_TalonFX _shooterL = new WPI_TalonFX(1);
  WPI_TalonFX _shooterR = new WPI_TalonFX(2);
  
  public Shooter() {
    robotInit();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void shoot40(){
  _shooterL.config_kF(Constants.kPIDLoopIdx, Constants.kGains_Velocit40.kF, Constants.kTimeoutMs);
  _shooterL.config_kP(Constants.kPIDLoopIdx, Constants.kGains_Velocit40.kP, Constants.kTimeoutMs);
  _shooterL.config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocit40.kI, Constants.kTimeoutMs);
  _shooterL.config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocit40.kD, Constants.kTimeoutMs);

  double targetVelocity_UnitsPer100ms = 2552 * 2048 / 600;
      
  _shooterL.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
  }
/** 
  public void shoot50(){
    _shooterL.config_kF(Constants.kPIDLoopIdx, Constants.kGains_Velocit50.kF, Constants.kTimeoutMs);
    _shooterL.config_kP(Constants.kPIDLoopIdx, Constants.kGains_Velocit50.kP, Constants.kTimeoutMs);
    _shooterL.config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocit50.kI, Constants.kTimeoutMs);
    _shooterL.config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocit50.kD, Constants.kTimeoutMs);
  
    double targetVelocity_UnitsPer100ms = 3188 * 2048 / 600;
        
    _shooterL.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
  }
  public void shoot60(){
    _shooterL.config_kF(Constants.kPIDLoopIdx, Constants.kGains_Velocit60.kF, Constants.kTimeoutMs);
    _shooterL.config_kP(Constants.kPIDLoopIdx, Constants.kGains_Velocit60.kP, Constants.kTimeoutMs);
    _shooterL.config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocit60.kI, Constants.kTimeoutMs);
    _shooterL.config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocit60.kD, Constants.kTimeoutMs);
  
    double targetVelocity_UnitsPer100ms = 3825 * 2048 / 600;
        
    _shooterL.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
    }
  public void shoot80(){
    _shooterL.config_kF(Constants.kPIDLoopIdx, Constants.kGains_Velocit80.kF, Constants.kTimeoutMs);
    _shooterL.config_kP(Constants.kPIDLoopIdx, Constants.kGains_Velocit80.kP, Constants.kTimeoutMs);
    _shooterL.config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocit80.kI, Constants.kTimeoutMs);
    _shooterL.config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocit80.kD, Constants.kTimeoutMs);

    _shooterR.config_kF(Constants.kPIDLoopIdx, Constants.kGains_Velocit80.kF, Constants.kTimeoutMs);
    _shooterR.config_kP(Constants.kPIDLoopIdx, Constants.kGains_Velocit80.kP, Constants.kTimeoutMs);
    _shooterR.config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocit80.kI, Constants.kTimeoutMs);
    _shooterR.config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocit80.kD, Constants.kTimeoutMs);
  
    double targetVelocity_UnitsPer100ms = 5100 * 2048 / 600;
        
    _shooterL.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
    _shooterR.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);


  }
  
*/


  public void stopShooter(){
    _shooterL.set(ControlMode.PercentOutput,0.0);
    _shooterR.set(ControlMode.PercentOutput,0.0);
  }
  public void ShootBalls(){
    _shooterL.set(ControlMode.PercentOutput,1);
  }

  public void robotInit(){
    _shooterL.configFactoryDefault();
    _shooterR.configFactoryDefault();

    //_shooterR.follow(_shooterL);

    
    _shooterR.setInverted(true);
    _shooterL.setInverted(false);
    
     
    _shooterL.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,
                                          Constants.kPIDLoopIdx, 
                                          Constants.kTimeoutMs);
    
     _shooterR.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,
                                          Constants.kPIDLoopIdx, 
                                          Constants.kTimeoutMs);

        
    _shooterL.setSensorPhase(true);
    _shooterR.setSensorPhase(true);

		_shooterL.configNominalOutputForward(0, Constants.kTimeoutMs);
		_shooterL.configNominalOutputReverse(0, Constants.kTimeoutMs);
		_shooterL.configPeakOutputForward(1, Constants.kTimeoutMs);
    _shooterL.configPeakOutputReverse(-1, Constants.kTimeoutMs);
    _shooterR.configNominalOutputForward(0, Constants.kTimeoutMs);
		_shooterR.configNominalOutputReverse(0, Constants.kTimeoutMs);
		_shooterR.configPeakOutputForward(1, Constants.kTimeoutMs);
    _shooterR.configPeakOutputReverse(-1, Constants.kTimeoutMs);
    
    


  }
}
