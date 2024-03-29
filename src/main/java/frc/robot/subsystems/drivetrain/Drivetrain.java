package frc.robot.subsystems.drivetrain;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.*;

import javax.imageio.spi.ImageOutputStreamSpi;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Drivetrain extends SubsystemBase{
    SwerveModule[] swerveModules;
    SwerveDriveKinematics driveKinematics;
    SwerveDriveOdometry driveOdometry;
    static  WPI_Pigeon2  imu = new WPI_Pigeon2(0, "canivore");
    

   // SwerveModule[] swerveModules; //0 is LF, 1 is LB, 2 is RF etc

    public Drivetrain(){ //constructor method
        swerveModules= new SwerveModule[]{
            new SwerveModule(LF_DRIVE_MOTOR_PORT, LF_STEER_MOTOR_PORT , LF_ANGLE_ENCODER_PORT, LF_OFFSET, true),
            new SwerveModule(LB_DRIVE_MOTOR_PORT, LB_STEER_MOTOR_PORT , LB_ANGLE_ENCODER_PORT, LB_OFFSET, true),
            new SwerveModule(RF_DRIVE_MOTOR_PORT, RF_STEER_MOTOR_PORT , RF_ANGLE_ENCODER_PORT, RF_OFFSET, false),
            new SwerveModule(RB_DRIVE_MOTOR_PORT, RB_STEER_MOTOR_PORT , RB_ANGLE_ENCODER_PORT, RB_OFFSET, false) //TODO simplify later
        };
        
   

        driveKinematics = new SwerveDriveKinematics(moduleOffset);
        // driveOdometry.
        driveOdometry = new SwerveDriveOdometry(driveKinematics, imu.getRotation2d());
      // imu.reset(); //////////

    }
    @Override 
    public void periodic(){
        SwerveModuleState[] measuredModuleStates = new SwerveModuleState[4];
        for (int x = 0; x < measuredModuleStates.length; x++){
            measuredModuleStates[x] = swerveModules[x].getModuleState();
            // System.out.println("Wheel Angle "+ x +  ": " + swerveModules[x].getModuleState().angle.getDegrees());

        }
     
        driveOdometry.updateWithTime(Timer.getFPGATimestamp(), imu.getRotation2d(), measuredModuleStates);

    }
    public void stopDrive (){
        
    }
    public void drive(double xVelocity, double yVelocity, double angularVelocity, boolean isFieldRelative){
        SwerveModuleState[] moduleStates;
        ChassisSpeeds robotSpeed;
        
        if(isFieldRelative) robotSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, angularVelocity, imu.getRotation2d());
        else robotSpeed = new ChassisSpeeds(xVelocity, yVelocity, angularVelocity);
        
        //SmartDashboard.putNumber("yaw", imu.getRotation2d().getDegrees());
        //SmartDashboard.putNumber("xVelocity", robotSpeed.vxMetersPerSecond);
        //SmartDashboard.putNumber("yVelocity", robotSpeed.vxMetersPerSecond);
        //SmartDashboard.putNumber("angularVelocity", robotSpeed.vxMetersPerSecond);

        moduleStates = driveKinematics.toSwerveModuleStates(robotSpeed);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_SPEED_METRES_PER_SECOND);

        for(int x = 0; x < moduleStates.length; x++){
            swerveModules[x].setDesiredModuleState(moduleStates[x]);
        }
        
    }


public void turnLFmodule (double angle){
    for(SwerveModule swerveModule : swerveModules){
        swerveModule.setDesiredModuleState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(angle)));
    }
    
}    

public static void resetGyro(){
    imu.reset();
}

public void setGyroPosition(Double angle) {
    imu.setYaw(-angle);
}

public Pose2d getRobotPosition() {
    return driveOdometry.getPoseMeters();
}

public void setRobotPosition(Pose2d pose2d , Rotation2d gyroAngle) {
    imu.setYaw(-gyroAngle.getDegrees());
    driveOdometry.resetPosition(pose2d, gyroAngle);
}

public static double deadZone (double input){
    if(Math.abs(input) < 0.2){
        return 0;
    }else if(input > 0.2){
        //SmartDashboard.putNumber("Input pos deadzone", 1.25 * (input + 0.25));
        return 1.25 * (input - 0.2);
    } else {
        //SmartDashboard.putNumber("Input neg deadzone", 1.25 * (input - 0.25));
        return 1.25* (input + 0.2);
    }

}    

public void setDriveMotorClosedLoopRamp(double rampTime) {
    for (SwerveModule swerveModule : swerveModules) {
        swerveModule.setClosedLoopRamp(rampTime);
    }
}
}

