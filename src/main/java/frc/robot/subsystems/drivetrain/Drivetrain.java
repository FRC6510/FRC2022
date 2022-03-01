package frc.robot.subsystems.drivetrain;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.*;

import javax.imageio.spi.ImageOutputStreamSpi;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;

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
    static WPI_PigeonIMU imu = new WPI_PigeonIMU(00);
    

   // SwerveModule[] swerveModules; //0 is LF, 1 is LB, 2 is RF etc

    public Drivetrain(){ //constructor method
        swerveModules = new SwerveModule[]{
            new SwerveModule(LF_DRIVE_MOTOR_PORT, LF_STEER_MOTOR_PORT , LF_ANGLE_ENCODER_PORT, 13.184, true),
            new SwerveModule(LB_DRIVE_MOTOR_PORT, LB_STEER_MOTOR_PORT , LB_ANGLE_ENCODER_PORT, 60.156, true),
            new SwerveModule(RF_DRIVE_MOTOR_PORT, RF_STEER_MOTOR_PORT , RF_ANGLE_ENCODER_PORT, 104.238, false),
            new SwerveModule(RB_DRIVE_MOTOR_PORT, RB_STEER_MOTOR_PORT , RB_ANGLE_ENCODER_PORT, 147.304, false) //TODO simplify later
        };

        driveKinematics = new SwerveDriveKinematics(moduleOffset);

        driveOdometry = new SwerveDriveOdometry(driveKinematics, imu.getRotation2d());

    }
    @Override 
    public void periodic(){
        SwerveModuleState[] measuredModuleStates = new SwerveModuleState[4];
        for (int x = 0; x < measuredModuleStates.length; x++){
            measuredModuleStates[x] = swerveModules[x].getModuleState();
        }
     SmartDashboard.putNumber("frontLeftError: ",  swerveModules[0].getSteerError());
     SmartDashboard.putNumber("backLeftError: ",  swerveModules[1].getSteerError());
     SmartDashboard.putNumber("frontRightError: ",  swerveModules[2].getSteerError());
     SmartDashboard.putNumber("backRightaError: ",  swerveModules[3].getSteerError());
     SmartDashboard.putNumber("yaw",imu.getYaw());
     SmartDashboard.putNumber("converted yaw",imu.getRotation2d().getDegrees());
     //SmartDashboard.putNumber("status frame", swerveModules[0].STATU );
     
        driveOdometry.updateWithTime(Timer.getFPGATimestamp(), imu.getRotation2d(), measuredModuleStates);
    
    }
    public void drive(double xVelocity, double yVelocity, double angularVelocity, boolean isFieldRelative){
        SwerveModuleState[] moduleStates;
        ChassisSpeeds robotSpeed;
        SmartDashboard.putNumber("yaw", imu.getRotation2d().getDegrees());
        if(isFieldRelative) robotSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, angularVelocity, imu.getRotation2d());
        else robotSpeed = new ChassisSpeeds(xVelocity, yVelocity, angularVelocity);
        SmartDashboard.putNumber("xVelocity", robotSpeed.vxMetersPerSecond);
        SmartDashboard.putNumber("yVelocity", robotSpeed.vxMetersPerSecond);
        SmartDashboard.putNumber("angularVelocity", robotSpeed.vxMetersPerSecond);

        
        moduleStates = driveKinematics.toSwerveModuleStates(robotSpeed);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_SPEED_METRES_PER_SECOND);
        SmartDashboard.putNumber("LF ANGLE", moduleStates[0].angle.getDegrees());

        for(int x = 0; x < moduleStates.length; x++){
            swerveModules[x].setDesiredModuleState(moduleStates[x]);
        }

    }
public void turnLFmodule (double angle){
    for(SwerveModule swerveModule : swerveModules){
        swerveModule.setDesiredModuleState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(angle)));

    }
    
}    

public void resetGyro(){
    imu.reset();
}

public static double deadZone (double input){

     if(Math.abs(input) < 0.2){ // <0.15
         return 0;
     } else if(input<0) {
         return -((1.0/0.64)*(0.8*input-0.2)*(0.8*input-0.2));
     } return ((1.0/0.64)*(0.8*input-0.2)*(0.8*input-0.2));
 }    

/*if(Math.abs(input) < 0.2){ // <0.15
    return 0;
} else if(input<0) {
    return -(1.25*input + 0.2);
} return (1.25*input + 0.2);
} */   


}
