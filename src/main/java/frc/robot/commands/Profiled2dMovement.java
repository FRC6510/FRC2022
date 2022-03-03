package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class Profiled2dMovement extends CommandBase{
    //Hardware
    private Drivetrain drivetrain;

    //Control
    ProfiledPIDController linearPID, rotationPID;

    private double distanceToTarget;

    private Pose2d endPos, poseTolerance;
    private boolean atTarget;

    public Profiled2dMovement(Profiled2dMovementConfig constraints, Pose2d endPos){
        this.linearPID = new ProfiledPIDController(constraints.linearKp, constraints.linearKi, constraints.linearKd, constraints.linearMotiConstraints);
        this.rotationPID = new ProfiledPIDController(constraints.rotaionalKp, constraints.rotationalKi, constraints.rotationalKd, constraints.rotationalMotionConstraints);
        this.endPos = endPos;
        this.poseTolerance = poseTolerance;
        this.rotationPID.enableContinuousInput(-Math.PI, Math.PI);
        
    }


    public static class Profiled2dMovementConfig{
        public double linearKp, linearKi, linearKd, rotaionalKp, rotationalKi, rotationalKd;

        TrapezoidProfile.Constraints linearMotiConstraints, rotationalMotionConstraints;
        Pose2d poseTolerance;
        
    public Profiled2dMovementConfig(double linearKp, double linearKi, double linearKd, double rotationalKp, double rotationalKi, double rotationalKd, TrapezoidProfile.Constraints forwardMotion, TrapezoidProfile.Constraints rotationalMotion, Pose2d endTolerance){
            this.linearKp = linearKp;
            this.linearKi = linearKi;
            this.linearKd = linearKd;
            this.rotaionalKp = rotationalKp;
            this.rotationalKi = rotationalKi;
            this.rotationalKd = rotationalKd;
            this.linearMotiConstraints = forwardMotion;
            this.rotationalMotionConstraints = rotationalMotion;
            this.poseTolerance = endTolerance;
        }
        
    }

    
}
