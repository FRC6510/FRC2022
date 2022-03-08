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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Limelight extends SubsystemBase {
  public static double distanceFromLimelightToGoalInches;
  public static double FrontSpeed,BackSpeed;
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

   double limelightMountAngleDegrees = 22.92;//67.08;
   double limelightLensHeightInches = 37.4;
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

  FrontSpeed = 0.38*distanceFromLimelightToGoalInches*9000*0.025;
  BackSpeed =  0.38*distanceFromLimelightToGoalInches*7000*0.025;

//post to smart dashboard periodically
SmartDashboard.putNumber("LimelightX", x);
SmartDashboard.putNumber("LimelightY", y);
SmartDashboard.putNumber("LimelightArea", area);
SmartDashboard.putNumber("distance from limelight to goal",distanceFromLimelightToGoalInches);
SmartDashboard.putBoolean("LMhasTarget",m_LimelightHasValidTarget);
SmartDashboard.putNumber("D", RobotContainer.D);
SmartDashboard.putNumber("SF", FrontSpeed);
SmartDashboard.putNumber("SB", BackSpeed);
  }

  public static double UpdateShooterVelocity(){
    return FrontSpeed;
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
