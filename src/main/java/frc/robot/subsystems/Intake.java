// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private static TalonFX intake;
 
  //Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

  private static Solenoid intake_pneu  = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

  //private static final double velocitykp = 0, velocityki = 0, velocitykd = 0;

  public Intake(){

    intake = new TalonFX(55);
   //intake_pneu = 
    
    intake.setInverted(false);
    intake.setNeutralMode(NeutralMode.Brake); //stop mode
    intake.configOpenloopRamp(1); //ramp acceleration
    //Intake.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor); <-- What's this?
    intake_pneu.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void spin_intake(){
    intake.set(ControlMode.PercentOutput,0.8); //speed
    intake_pneu.set(true);
  }

  public void reverse_intake(){
   intake.set(ControlMode.PercentOutput,-0.8); //speed
  }

  public void stop_intake(){
    intake.set(ControlMode.PercentOutput,0);
    intake_in();
  }

  public void intake_out(){
    intake_pneu.set(true);
  }

  public void intake_in(){
    intake_pneu.set(false);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
