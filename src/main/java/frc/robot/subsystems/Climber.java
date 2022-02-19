// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private static TalonFX climber = new TalonFX(44);
  private static Solenoid climber_pneu = new Solenoid(PneumaticsModuleType.CTREPCM, 5);


  //private static final double velocitykp = 0, velocityki = 0, velocitykd = 0;

  public Climber(){
    climber.setInverted(false);
    climber.setNeutralMode(NeutralMode.Brake); //stop mode
    climber.configOpenloopRamp(1); //ramp acceleration
    //Intake.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor); <-- What's this?
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void spin_climber(){
    climber.set(ControlMode.PercentOutput,0.5); //speed
  }

  public void reverse_climber(){
    climber.set(ControlMode.PercentOutput,-0.5); //speed
  }

  public void climber_in(){
    climber_pneu.set(true); //speed
  }

  public void climber_out(){
    climber_pneu.set(false);//speed
  }

  public void stop_climber(){
    climber.set(ControlMode.PercentOutput,0);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
