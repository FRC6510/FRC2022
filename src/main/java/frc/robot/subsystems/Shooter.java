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

public class Shooter extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private static TalonFX ShooterLeft = new TalonFX(7);
  private static TalonFX ShooterRight = new TalonFX(8);
  private static TalonFX ShooterBack = new TalonFX(35);


  private static final double velocitykp = 0, velocityki = 0, velocitykd = 0;

  public Shooter(){
    ShooterLeft.setInverted(true); //direction
    ShooterRight.setInverted(false);
    ShooterLeft.follow(ShooterRight); //slave
    ShooterLeft.setNeutralMode(NeutralMode.Coast); //stop mode
    ShooterRight.setNeutralMode(NeutralMode.Coast);
    ShooterBack.setNeutralMode(NeutralMode.Coast);
    ShooterLeft.configOpenloopRamp(0.5); //ramp acceleration
    ShooterRight.configOpenloopRamp(0.5);
    ShooterBack.configOpenloopRamp(0.5);
    ShooterRight.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    ShooterBack.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void spin_shooter(){
    ShooterLeft.set(ControlMode.PercentOutput,0.6); //speed
    ShooterRight.set(ControlMode.PercentOutput,0.6);
    ShooterBack.set(ControlMode.PercentOutput,0.4);

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
}
