package frc.robot.subsystems.drivetrain;

import edu.wpi.first.math.geometry.Translation2d;

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

    public static final double 
        MAX_SPEED_METRES_PER_SECOND = 4, //0.8
        GEAR_RATIO = 8.14, //one rotation of wheel from 8.14 shaft rotations
        CANCODER_RAW_TICKS_PER_180 = 2048,
        TEN = 10,
        WHEEL_RADIUS = 0.0508,
        HALF_ROTATION = CANCODER_RAW_TICKS_PER_180 / 180.0,
        TALONFX_RESOLUTION = 2048,
        TICKS_PER_WHEEL_ROTATION = TALONFX_RESOLUTION * GEAR_RATIO,
        OMEGA = TEN*((2*Math.PI)/TICKS_PER_WHEEL_ROTATION), //conversion from radians per 100ms to 1000ms (1s)
        VELOCITY_MULTIPLIER = OMEGA * WHEEL_RADIUS;
        
}





