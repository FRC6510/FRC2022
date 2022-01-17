// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;



public class Drivetrain extends SubsystemBase {
  WPI_TalonFX LeftDrive1 = new WPI_TalonFX(4);
  WPI_TalonFX LeftDrive2 = new WPI_TalonFX(5);
  WPI_TalonFX RightDrive1 = new WPI_TalonFX(3);
  WPI_TalonFX RightDrive2 = new WPI_TalonFX(0);
  Timer timer = new Timer();

  //DoubleSolenoid ShiftGear = new DoubleSolenoid(0,1);

  TalonFXInvertType LeftInvert = TalonFXInvertType.CounterClockwise;
  TalonFXInvertType RightInvert = TalonFXInvertType.CounterClockwise;



  DifferentialDrive m_safety_drive = new DifferentialDrive(LeftDrive1,RightDrive1);




  /** Creates a new Drivetrain. */
  public Drivetrain() {
    robotInit();
    

  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void robotInit(){
  //ShiftGear.set(Value.kForward);

    LeftDrive1.configFactoryDefault();
    LeftDrive2.configFactoryDefault();
    RightDrive1.configFactoryDefault();
    RightDrive2.configFactoryDefault();

    LeftDrive1.setNeutralMode(NeutralMode.Coast);
    LeftDrive2.setNeutralMode(NeutralMode.Coast);
    RightDrive1.setNeutralMode(NeutralMode.Coast);
    RightDrive2.setNeutralMode(NeutralMode.Coast);

    LeftDrive2.follow(LeftDrive1);
    RightDrive2.follow(RightDrive1);


    //LeftDrive1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    //RightDrive1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    LeftDrive2.setInverted(InvertType.FollowMaster);
    RightDrive2.setInverted(InvertType.FollowMaster);
    

  }




  public void drive(double speed, double rotation) {
    m_safety_drive.arcadeDrive(0.8 * speed, 0.7 * rotation);

  }


/*
  public void setLowGear () {
    ShiftGear.set(Value.kReverse);
  }

  
  public void setHighGear () {
    ShiftGear.set(Value.kForward);
  }
*/


}
