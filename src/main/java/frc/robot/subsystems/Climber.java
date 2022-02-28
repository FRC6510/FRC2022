// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import org.opencv.features2d.FlannBasedMatcher;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private static TalonFX climber;
  private static Solenoid climber_pneu;
  private static Solenoid hook_pneu;


  //private static final double velocitykp = 0, velocityki = 0, velocitykd = 0;

  public Climber(){

    climber = new TalonFX(44);
    climber_pneu = new Solenoid(PneumaticsModuleType.CTREPCM, 2);
    hook_pneu = new Solenoid(PneumaticsModuleType.CTREPCM, 7);

    climber.setInverted(false);
    climber.setNeutralMode(NeutralMode.Brake); //stop mode
    climber.configOpenloopRamp(0.3); //ramp acceleration
    //Intake.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor); <-- What's this?
    //climber.configStatorCurrentLimit(true);
    //climber.configPeakCurrentLimit(30); // don't activate current limit until current
    //climber.configPeakCurrentDuration(100); // ... for at least 100 ms
    //climber.configContinuousCurrentLimit(20); // once current-limiting is actived, hold at
    climber_pneu.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void spin_climber(){
    climber.set(ControlMode.PercentOutput,0.4); //speed
  }

  public void reverse_climber(){
    climber.set(ControlMode.PercentOutput,-0.4); //speed
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

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
