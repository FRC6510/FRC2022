package frc.robot.subsystems.drivetrain;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {

    private final WPI_TalonFX driveMotor;
    private final WPI_TalonFX steeringMotor;
    double encoderOffset;

    private final PIDController steerPID;

    private final CANCoder angleEncoder;

    public SwerveModule(int driveMotorPort, int steeringMotorPort , int angleEncoderPort, double encoderOffset, boolean isDriveMotorInverted) {
        this.driveMotor = new WPI_TalonFX(driveMotorPort);
        this.steeringMotor = new WPI_TalonFX(steeringMotorPort);
        configSwerveMotor(driveMotor, FeedbackDevice.IntegratedSensor, 0.1023, 0, 0.6, 0.0488);
        configSwerveMotor(steeringMotor, FeedbackDevice.IntegratedSensor, 1.8, 0, 18, 0);  //kI 0.004 kP 1.5 /1.8 -> kp 1.8

        this.steerPID = new PIDController(0.0055,0,0);
        this.steerPID.enableContinuousInput(-180,180);
        this.steerPID.setTolerance(0);

        this.encoderOffset = encoderOffset;

        this.angleEncoder = new CANCoder(angleEncoderPort);
        CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
        canCoderConfiguration.initializationStrategy=SensorInitializationStrategy.BootToAbsolutePosition;
        canCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        canCoderConfiguration.magnetOffsetDegrees = encoderOffset;
        this.angleEncoder.configAllSettings(canCoderConfiguration);
        steeringMotor.configRemoteFeedbackFilter(angleEncoder,0);
        steeringMotor.configClosedLoopPeakOutput(0, 1);
        driveMotor.setInverted(isDriveMotorInverted);
        steeringMotor.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
        resetToAbsolute();

    }
    protected void configSwerveMotor(WPI_TalonFX motor,FeedbackDevice FeedbackSensor, double kP, double kI, double kD, double kF){
        motor.configFactoryDefault();
        motor.setNeutralMode(NeutralMode.Brake);
        motor.configSelectedFeedbackSensor(FeedbackSensor);
        motor.selectProfileSlot(0, 0);
        motor.config_kP(0, kP, 0);
        motor.config_kI(0, kI, 0);
        motor.config_kD(0, kD, 0);
        motor.config_kF(0, kF, 0);
        motor.config_IntegralZone(0, 33);
        motor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_50Ms);
        motor.configVelocityMeasurementWindow(8,50); //might need to change this
        motor.setStatusFramePeriod(1, 50); /// ????(1, 20);
        motor.setStatusFramePeriod(2, 80); /// ????(2, 30);


    }

    public static double degreesToFalcon(double degrees, double gearRatio){
        double ticks = degrees / (360.0 / (gearRatio * 2048));
        return ticks;
    }

    public static double falconToDegrees(double counts, double gearRatio){
        return counts * (360/(gearRatio * 2048.0));
    }

private void resetToAbsolute(){
    double absolutePosition = degreesToFalcon(angleEncoder.getAbsolutePosition() - encoderOffset, DrivetrainConstants.angleGearRatio);
    steeringMotor.setSelectedSensorPosition(absolutePosition);
}

    public void setDesiredModuleState(SwerveModuleState swerveModuleState){
        swerveModuleState = SwerveModuleState.optimize(swerveModuleState, Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition()));
        double angleOutput = steerPID.calculate(angleEncoder.getAbsolutePosition(), swerveModuleState.angle.getDegrees());
        // SwerveModuleState state = SwerveModuleState.optimize(swerveModuleState, new Rotation2d(angleEncoder.getAbsolutePosition()));
        // SwerveModuleState state = CTREModuleStates.optimise(swerveModuleState, new Rotation2d(angleEncoder.getAbsolutePosition()));
        //SwerveModuleState state = swerveModuleState;
        driveMotor.set(ControlMode.Velocity, convertToWheelEncoderTicks(swerveModuleState.speedMetersPerSecond));
        steeringMotor.set(ControlMode.PercentOutput,angleOutput);
        // SmartDashboard.putNumber("target", DrivetrainConstants.HALF_ROTATION * state.angle.getDegrees());
        // steeringMotor.set(ControlMode.Position,degreesToFalcon(state.angle.getDegrees(), DrivetrainConstants.angleGearRatio));
        // steeringMotor.set(ControlMode.Position, DrivetrainConstants.HALF_ROTATION * state.angle.getDegrees());
        // steeringMotor.configFeedbackNotContinuous(false, 0);
    }

    //public SwerveModuleState directionOptimisation(){
    //    return null;
    //}

    protected SwerveModuleState getModuleState(){
        return new SwerveModuleState(getWheelVelocity(), new Rotation2d(angleEncoder.getAbsolutePosition()));
    }

    public double getSteerError(){
        return steeringMotor.getClosedLoopError(0);
    }

    public double getWheelVelocity(){
        return DrivetrainConstants.VELOCITY_MULTIPLIER * driveMotor.getSelectedSensorVelocity();
    }

    public double convertToWheelEncoderTicks(double wheelVelocity){
        return wheelVelocity/DrivetrainConstants.VELOCITY_MULTIPLIER;
    }

}


