// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Feeder extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private static VictorSPX leftFeeder = new VictorSPX(5);
  private static VictorSPX frontFeeder = new VictorSPX(6);
  public DigitalInput feederSensor = new DigitalInput(0);

  public Feeder(){
    leftFeeder.setInverted(false);
    frontFeeder.setInverted(true);
    leftFeeder.setNeutralMode(NeutralMode.Brake); //stop mode
    frontFeeder.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void spin_feeder(double speed){
    leftFeeder.set(ControlMode.PercentOutput,speed);
    frontFeeder.set(ControlMode.PercentOutput,speed);
  }

  public void reverse_feeder(){
    leftFeeder.set(ControlMode.PercentOutput,-0.35);
    //frontFeeder.set(ControlMode.PercentOutput,-0.35);
  }

  public void stop_feeder(){
    leftFeeder.set(ControlMode.PercentOutput,0);
    frontFeeder.set(ControlMode.PercentOutput,0);
  }

  public boolean sense_ball(){
    return feederSensor.get();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void log(){
    SmartDashboard.getBoolean("feederSensor", feederSensor.get());

  }
}
