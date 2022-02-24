// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Add your docs here. */
public class CTREModuleStates {

    public static SwerveModuleState optimise(SwerveModuleState desiredState, Rotation2d currentAngle){
         double targetAngle = adjustForContinuousRange(currentAngle.getDegrees(), desiredState.angle.getDegrees());
         double targetSpeed = desiredState.speedMetersPerSecond;
         double delta =  - currentAngle.getDegrees();
        //  if(Math.abs(delta) > 90){
        //     targetSpeed = -targetSpeed;
        //     if(delta > 90){
        //         targetAngle -= 180;
        //     } else{
        //         targetAngle += 180;
        //     }
        //  }
         return new SwerveModuleState(targetSpeed,Rotation2d.fromDegrees(targetAngle));
    }

    public static double adjustForContinuousRange(double currentAngle, double targetAngle){
        double lowerBound;
        double upperBound;
        double lowerOffset = currentAngle % 360;
        if(lowerOffset >= 0){
            lowerBound = currentAngle - lowerOffset;
            upperBound = currentAngle + (360 - lowerOffset);
        } else{
            upperBound = currentAngle - lowerOffset;
            lowerBound = currentAngle - (360 + lowerOffset);
        }
        while(targetAngle < lowerBound){
            targetAngle += 360;
        }
        while(targetAngle > upperBound){
            targetAngle -= 360;
        }
        if(targetAngle - currentAngle > 180){
            targetAngle -= 360;
        }else if(targetAngle - currentAngle < -180){
            targetAngle += 360;
        }
        return targetAngle;
    }

}
