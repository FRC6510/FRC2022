// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Feeder extends SubsystemBase {

  WPI_VictorSPX feeder_lift = new WPI_VictorSPX(11);
  WPI_VictorSPX feeder_main = new WPI_VictorSPX(8);
  DoubleSolenoid feeder_pneu = new DoubleSolenoid(4,5);



  /** Creates a new Feeder. */
  public Feeder() {
    feeder_main.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void spin_feeder_lift(){
    feeder_lift.set(.5);
  }

  public void stop_feeder_lift(){
    feeder_lift.set(0);
  }

  public void spin_feeder_main(){
    feeder_main.set(1);
  }

  public void stop_feeder_main(){
    feeder_main.set(0);
  }
  public void flip_out(){
    feeder_pneu.set(Value.kReverse);
  }

  public void flip_in(){
      feeder_pneu.set(Value.kForward);
  }





}
