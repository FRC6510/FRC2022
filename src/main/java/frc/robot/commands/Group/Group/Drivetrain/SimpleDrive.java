package frc.robot.commands.Group.Group.Drivetrain;

import java.lang.annotation.Target;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class SimpleDrive extends CommandBase{
    //Hardware
    private Drivetrain drivetrain;
    private double xVel, yVel, rotVel, time, startTime;

    /**
     * 
     * @param drivetrain
     * @param xVel
     * @param yVel
     * @param rotVel
     * @param time time command should run for in seconds
     */
    public SimpleDrive(Drivetrain drivetrain, double xVel, double yVel, double rotVel, double time){
        this.drivetrain = drivetrain;
        this.yVel = yVel;
        this.xVel = xVel;
        this.rotVel = rotVel;
        this.time = time;
        // this.linearPID = new ProfiledPIDController(constraints.linearKp, constraints.linearKi, constraints.linearKd, constraints.linearMotionConstraints);
        // this.linearPID.setGoal(new TrapezoidProfile.State(straightLineDistance, 0));
    
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.startTime = Timer.getFPGATimestamp();
        // drivetrain.setRobotPosition(new Pose2d(0, 0, Rotation2d.fromDegrees(0)), Rotation2d.fromDegrees(0));
        System.out.println("Command is being Initialised");
        // this.straightLineDistance = drivetrain.getRobotPosition().getTranslation().getDistance(endPos.getTranslation());
        // this.linearPID.reset(0);
        // this.linearPID.setGoal(new TrapezoidProfile.State(straightLineDistance, 0));
        // this.rotationPID.reset(drivetrain.getRobotPosition().getRotation().getRadians(), 0);
        // this.rotationPID.setGoal(new TrapezoidProfile.State(endPos.getRotation().getRadians(), 0));
        
  
    }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
         drivetrain.drive(xVel, yVel, rotVel, true);
      }
    
     

     // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return isOnTarget(drivetrain.getRobotPosition(), endPos);
    return Timer.getFPGATimestamp() - startTime > time;
  }

// Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0, true);
}






    
}
