// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  WPI_TalonFX intake_motor = new WPI_TalonFX(9);
  DoubleSolenoid intake_pneu = new DoubleSolenoid(2,3);


  /** Creates a new Intake. */
  public Intake() {
    intake_motor.setInverted(true);
  }

  @Override
  public void periodic() {

  }
    // This method will be called once per scheduler run
    
    public void reverse_intake(){
    intake_motor.set(-0.5);    }
  
    public void stop_intake(){
      intake_motor.set(0);
    }
    public void spin_intake(){
      intake_motor.set(1);
    }
    public void retract_intake(){
      intake_pneu.set(Value.kReverse);
      intake_motor.set(0);

    }
    public void intake_out(){
      intake_pneu.set(Value.kForward);
      intake_motor.set(1);
    }

    



  }