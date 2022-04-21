// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Limelight;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Limelight extends SubsystemBase {
  public static double distanceFromLimelightToGoalInches;
  public static double FrontSpeed,BackSpeed;
  public static double Angle, ControllerValue;
  public static double x;

  /** Creates a new ExampleSubsystem. */
  private boolean m_LimelightHasValidTarget = false;

  public Limelight(){
 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

   // SmartDashboard.putNumber("distance from limelight to goal",distanceFromLimelightToGoalInches);
  
   NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
   NetworkTableEntry tx = table.getEntry("tx");
   NetworkTableEntry ty = table.getEntry("ty");
   NetworkTableEntry ta = table.getEntry("ta");

   double targetOffsetAngle_Vertical = ty.getDouble(0.0);

   double limelightMountAngleDegrees = 16.5; //67.08, idk maybe subtract 16.5 from 90
   double limelightLensHeightInches = 35.7;
   double goalHeightInches = 104;

   double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
   double angleToGoalRadians = angleToGoalDegrees * (3.14159/180.0);

  distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches)/Math.tan(angleToGoalRadians);

  
       ////Update_Limelight_Tracking();
       //read values periodically
       double x = tx.getDouble(0.0);
       double y = ty.getDouble(0.0);
       double area = ta.getDouble(0.0);

       
  RobotContainer.D = distanceFromLimelightToGoalInches;
  RobotContainer.SF = RobotContainer.K*RobotContainer.D*RobotContainer.FConstant;
  RobotContainer.SB = RobotContainer.K*RobotContainer.D*RobotContainer.BConstant;

  /*if(distanceFromLimelightToGoalInches <= 400){
    FrontSpeed = 0.1*distanceFromLimelightToGoalInches*9000*1.8*0.0254;
    BackSpeed =  0.23*distanceFromLimelightToGoalInches*7000*1.8*0.0254;
  } else if(400 > distanceFromLimelightToGoalInches && distanceFromLimelightToGoalInches > 300){
    FrontSpeed = 0.1*distanceFromLimelightToGoalInches*9000*2*0.0254;
    BackSpeed =  0.23*distanceFromLimelightToGoalInches*7000*2*0.0254;
  } else if(300 > distanceFromLimelightToGoalInches && distanceFromLimelightToGoalInches > 160){
    FrontSpeed = 0.25*distanceFromLimelightToGoalInches*9000*1.4*0.0254; //0.38, 0.3
    BackSpeed =  0.25*distanceFromLimelightToGoalInches*7000*1.4*0.0254;  
  } else{
    FrontSpeed = 
    9000;
    BackSpeed = 7000;
  } */

  //improve mid-back distance
  /*if(distanceFromLimelightToGoalInches >= 380){
    FrontSpeed = 0.7*0.08*distanceFromLimelightToGoalInches*9000*2.1*0.0254; //0.09 multiplier, 2.1 multiplier -> 0.08
    BackSpeed = 1*0.28*distanceFromLimelightToGoalInches*7000*2.1*0.0254; //0.26 multiplier, 2.1 multiplier) ->0.9?
  } else if(380 > distanceFromLimelightToGoalInches && distanceFromLimelightToGoalInches >= 270){
    FrontSpeed = 0.13*distanceFromLimelightToGoalInches*9000*1.3*0.0254; //0.15 , 1.8 multiplier
    BackSpeed =  0.25*distanceFromLimelightToGoalInches*7000*1.3*0.0254; //0.24, 1.8 multiplier
  } else if(270 > distanceFromLimelightToGoalInches && distanceFromLimelightToGoalInches >= 130){
    FrontSpeed = 0.22 *distanceFromLimelightToGoalInches*9000*0.0254; //0.38, 0.3
    BackSpeed =  0.22*distanceFromLimelightToGoalInches*7000*0.0254;  //both used to be
  } else{
    FrontSpeed = 
    9000;
    BackSpeed = 7000;
  }  */

  if(distanceFromLimelightToGoalInches >= 350){
    FrontSpeed = 0.7*0.08*distanceFromLimelightToGoalInches*9000*2.1*0.0254; //0.09 multiplier, 2.1 multiplier -> 0.08
    BackSpeed = 1*0.28*distanceFromLimelightToGoalInches*7000*2.1*0.0254; //0.26 multiplier, 2.1 multiplier) ->0.9?
  } else{
    FrontSpeed = 
    9000;
    BackSpeed = 7000;
  } 


  Angle = x;
  //FrontSpeed = 0.25*distanceFromLimelightToGoalInches*9000*0.025; //0.38, 0.3
  //BackSpeed =  0.25*distanceFromLimelightToGoalInches*7000*0.025;

//post to smart dashboard periodically
//SmartDashboard.putNumber("LimelightX", x);
//SmartDashboard.putNumber("LimelightY", y);
//SmartDashboard.putNumber("LimelightArea", area);
//SmartDashboard.putNumber("distance from limelight to goal",distanceFromLimelightToGoalInches);
//SmartDashboard.putBoolean("LMhasTarget",m_LimelightHasValidTarget);
//SmartDashboard.putNumber("D", RobotContainer.D);
//SmartDashboard.putNumber("SF", FrontSpeed);
//SmartDashboard.putNumber("SB", BackSpeed);
  }

  public static double UpdateShooterVelocity(){
    return FrontSpeed;
  }

  public void turnLimelightOn(){
    

  }
  public void Update_Limelight_Tracking()
  {
        // These numbers must be tuned for your Robot!  Be careful!
        final double STEER_K = 0.03;                    // how hard to turn toward the target
        final double DRIVE_K = 0.26;                    // how hard to drive fwd toward the target
        final double DESIRED_TARGET_AREA = 13.0;        // Area of the target when the robot reaches the wall
        final double MAX_DRIVE = 0.7;                   // Simple speed limit so we don't drive too fast

        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

        if (tv < 1.0)
        {
          
          return;
        }

        m_LimelightHasValidTarget = true;

        // Start with proportional steering

        // try to drive forward until the target area reaches our desired area

        // don't let the robot drive too fast into the goal
        
  }
  


}
