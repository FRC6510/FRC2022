package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.commands.Profiled2dMovement.Profiled2dMovementConfig;

public class DrivetrainConstants {
    public static final int 
    LF_DRIVE_MOTOR_PORT = 11,
    LB_DRIVE_MOTOR_PORT = 12,
    RF_DRIVE_MOTOR_PORT = 13,
    RB_DRIVE_MOTOR_PORT = 10;

public static final int
    LF_STEER_MOTOR_PORT = 15,
    LB_STEER_MOTOR_PORT = 16, 
    RF_STEER_MOTOR_PORT = 17,
    RB_STEER_MOTOR_PORT = 14; 

public static final int
    LF_ANGLE_ENCODER_PORT = 19,
    LB_ANGLE_ENCODER_PORT = 20,
    RF_ANGLE_ENCODER_PORT = 21,
    RB_ANGLE_ENCODER_PORT = 18;

public static final double 
    //x is forwards y is right )
    X_LENGTH_BETWEEN_MODULES = 1,
    Y_WIDTH_BETWEEN_MODULES = 1,
    MODULE_DISTANCE_FROM_CENTRE_LENGTH = X_LENGTH_BETWEEN_MODULES / 2,
    MODULE_DISTANCE_FROM_CENTRE_WIDTH = Y_WIDTH_BETWEEN_MODULES / 2;

public static final Translation2d[] moduleOffset = {
    // positive x values represent moving towards front of robot 
    // positive y values represent moving towards left of robot
    new Translation2d(MODULE_DISTANCE_FROM_CENTRE_LENGTH,MODULE_DISTANCE_FROM_CENTRE_WIDTH), // front left
    new Translation2d(-MODULE_DISTANCE_FROM_CENTRE_LENGTH, MODULE_DISTANCE_FROM_CENTRE_WIDTH), //back left
    new Translation2d(MODULE_DISTANCE_FROM_CENTRE_LENGTH,-MODULE_DISTANCE_FROM_CENTRE_WIDTH), // front right
    new Translation2d(-MODULE_DISTANCE_FROM_CENTRE_LENGTH,-MODULE_DISTANCE_FROM_CENTRE_WIDTH) //back right
};

    public static final double angleGearRatio = 12.8;

    public static final double 
        MAX_SPEED_METRES_PER_SECOND = 3, //0.8
        MAX_ACCELERATION_METRES_PER_SECOND_SQUARED = MAX_SPEED_METRES_PER_SECOND,
        MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 2*Math.PI,
        MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND_SQUARED = MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,

        GEAR_RATIO = 8.14, //one rotation of wheel from 8.14 shaft rotations
        CANCODER_RAW_TICKS_PER_180 = 2048,
        TEN = 10,
        WHEEL_RADIUS = 0.0508,
        HALF_ROTATION = CANCODER_RAW_TICKS_PER_180 / 180.0,
        TALONFX_RESOLUTION = 2048,
        TICKS_PER_WHEEL_ROTATION = TALONFX_RESOLUTION * GEAR_RATIO,
        OMEGA = TEN*((2*Math.PI)/TICKS_PER_WHEEL_ROTATION), //conversion from radians per 100ms to 1000ms (1s)
        VELOCITY_MULTIPLIER = OMEGA * WHEEL_RADIUS;

        /**
         * To determine new values:
         * Align until straight. Then get the absolute position value. Add/subtract (can't remember) that value to the current offset value.
         */
        public static final double
            LF_OFFSET = 13.184-1.143, //Need to test again
            LB_OFFSET = 60.156-0.527, // Should we change PID for turning?
            RF_OFFSET = 104.238+0.112,
            RB_OFFSET = 147.304-2.549;
                    
        public static final double
            linearKp = 2, 
            linearKi = 0,
            linearKd =  0,
            rotationalKp = 1.8,
            rotationalKi = 0,
            rotationalKd = 0;

        
        public static final double 
            A1X = 0.0, 
            A1Y = 0.0, 
            A1R = 0.0, 
            A2X = 0, 
            A2Y = 1, 
            A2R = 0;

        public static final double
            DEADZONEMULTIPLIER = 3.6,
            SLOWDEADZONEMULTIPLIER = 1.25;      
    
        public static final TrapezoidProfile.Constraints
            linearMotionConstraints = new TrapezoidProfile.Constraints(MAX_SPEED_METRES_PER_SECOND, MAX_ACCELERATION_METRES_PER_SECOND_SQUARED),
            angularMotionConstraints = new TrapezoidProfile.Constraints(MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND_SQUARED);

        public static final Pose2d
            endTolerance  = new Pose2d(0.05, 0.05, Rotation2d.fromDegrees(3));

        public static Profiled2dMovementConfig movementParameters = new Profiled2dMovementConfig(linearKp, linearKi, linearKd, rotationalKp, rotationalKi, rotationalKd, linearMotionConstraints, angularMotionConstraints, endTolerance);


}





