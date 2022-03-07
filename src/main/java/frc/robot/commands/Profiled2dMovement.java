package frc.robot.commands;

import java.lang.annotation.Target;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class Profiled2dMovement extends CommandBase{
    //Hardware
    private Drivetrain drivetrain;

    //Control
    // ProfiledPIDController linearPID, rotationPID;
    ProfiledPIDController xPidController, yPidController, rotationPidController;

    private double distanceToTarget;
    private double straightLineDistance;

    private Pose2d endPos, poseTolerance;
    private boolean atTarget;

    public Profiled2dMovement(Drivetrain drivetrain, Profiled2dMovementConfig constraints, Pose2d endPos){
        this.drivetrain = drivetrain;
        // this.linearPID = new ProfiledPIDController(constraints.linearKp, constraints.linearKi, constraints.linearKd, constraints.linearMotionConstraints);
        // this.linearPID.setGoal(new TrapezoidProfile.State(straightLineDistance, 0));
        this.rotationPidController = new ProfiledPIDController(constraints.rotaionalKp, constraints.rotationalKi, constraints.rotationalKd, constraints.rotationalMotionConstraints);
        this.xPidController = new ProfiledPIDController(constraints.linearKp, constraints.linearKi, constraints.linearKd, constraints.linearMotionConstraints);
        this.yPidController = new ProfiledPIDController(constraints.linearKp, constraints.linearKi, constraints.linearKd, constraints.linearMotionConstraints);
        this.endPos = endPos;
        this.poseTolerance = constraints.poseTolerance;
        this.rotationPidController.enableContinuousInput(-Math.PI, Math.PI);
        this.xPidController.setGoal(new TrapezoidProfile.State(endPos.getX(), 0));
        this.yPidController.setGoal(new TrapezoidProfile.State(endPos.getY(), 0));
        this.rotationPidController.setGoal(new TrapezoidProfile.State(endPos.getRotation().getRadians(), 0));
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
     
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
        double xSpeed = xPidController.calculate(drivetrain.getRobotPosition().getX()) + xPidController.getSetpoint().velocity;
        double ySpeed = yPidController.calculate(drivetrain.getRobotPosition().getY()) + yPidController.getSetpoint().velocity;
        double rotationSpeed = rotationPidController.calculate(drivetrain.getRobotPosition().getRotation().getRadians()) + rotationPidController.getSetpoint().velocity;
         drivetrain.drive(xSpeed, ySpeed, rotationSpeed, true);
        // Pose2d currentPos = drivetrain.getRobotPosition();
        // SmartDashboard.putNumber("current Y", currentPos.getY());
        // //Generates values for traveling along straight line
        // // this.straightLineDistance = currentPos.getTranslation().getDistance(endPos.getTranslation());
        // double angleFromCentre = findAngle(currentPos.getTranslation(), endPos.getTranslation());
  
        // final double distanceTravelled = straightLineDistance - currentPos.getTranslation().getDistance(endPos.getTranslation());
        // SmartDashboard.putNumber("distanceTravelled", distanceTravelled); 
        // final double linearSpeed = linearPID.calculate(distanceTravelled) + linearPID.getSetpoint().velocity;
    
        // SmartDashboard.putNumber("linearSpeed", linearSpeed);
        // //Converts straight line to orthogonal components
        // final double frontSpeed = linearSpeed * Math.cos(Math.toRadians(angleFromCentre));
        // final double strafeSpeed = linearSpeed * Math.sin(Math.toRadians(angleFromCentre));
        // final double rotationSpeed = rotationPID.calculate(currentPos.getRotation().getRadians()) + rotationPID.getSetpoint().velocity;
       
        // final double desiredLinearPosition = linearPID.getSetpoint().position;
        // double desiredX = desiredLinearPosition * Math.cos(angleFromCentre);
        // double desiredY = desiredLinearPosition * Math.sin(angleFromCentre);
        // xPidController.setSetpoint(desiredX);
        // double adjustedXVeloicty = xPID.calculate(desiredX) + frontSpeed

        // drivetrain.drive(frontSpeed, strafeSpeed, rotationSpeed, true);
       
    }
    
    public double findAngle(Translation2d startPos,Translation2d endPos){
        double forwardLength = endPos.getX()-startPos.getX();
        double leftLength = endPos.getY() - startPos.getY(); 
        // System.out.println(Math.toDegrees(Math.atan2(leftLength,forwardLength)));
        return  Math.atan2(leftLength,forwardLength);
    }


    public static class Profiled2dMovementConfig{
        public double linearKp, linearKi, linearKd, rotaionalKp, rotationalKi, rotationalKd;

        TrapezoidProfile.Constraints linearMotionConstraints, rotationalMotionConstraints;
        Pose2d poseTolerance;
        
    public Profiled2dMovementConfig(double linearKp, double linearKi, double linearKd, double rotationalKp, double rotationalKi, double rotationalKd, TrapezoidProfile.Constraints forwardMotion, TrapezoidProfile.Constraints rotationalMotion, Pose2d endTolerance){
            this.linearKp = linearKp;
            this.linearKi = linearKi;
            this.linearKd = linearKd;
            this.rotaionalKp = rotationalKp;
            this.rotationalKi = rotationalKi;
            this.rotationalKd = rotationalKd;
            this.linearMotionConstraints = forwardMotion;
            this.rotationalMotionConstraints = rotationalMotion;
            this.poseTolerance = endTolerance;
        }   
        
    }

    public boolean isOnTarget(Pose2d currentPose, Pose2d desiredPose) {
        Pose2d poseError = desiredPose.relativeTo(currentPose);
        final Translation2d translateError = poseError.getTranslation();
        final Rotation2d rotateError = poseError.getRotation();
        final Translation2d translateTolerance = poseTolerance.getTranslation();
        final Rotation2d rotateTolerance = poseTolerance.getRotation();
        SmartDashboard.putBoolean("X on Target", Math.abs(translateError.getX()) < translateTolerance.getX());
        SmartDashboard.putBoolean("Y on Target", Math.abs(translateError.getY()) <translateTolerance.getY());
        SmartDashboard.putBoolean("Rotaton on Target", Math.abs(rotateError.getRadians()) < rotateTolerance.getRadians());
        return Math.abs(translateError.getX()) < translateTolerance.getX()
                    && Math.abs(translateError.getY()) <translateTolerance.getY()
                    && Math.abs(rotateError.getRadians()) < rotateTolerance.getRadians();
        }

     // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isOnTarget(drivetrain.getRobotPosition(), endPos);
  }

// Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {
    drivetrain.drive(0, 0, 0, true);
}






    
}
