/**
 * Phoenix Software License Agreement
 *
 * Copyright (C) Cross The Road Electronics.  All rights
 * reserved.
 * 
 * Cross The Road Electronics (CTRE) licenses to you the right to 
 * use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and 
 * Phoenix Software API Libraries ONLY when in use with CTR Electronics hardware products
 * as well as the FRC roboRIO when in use in FRC Competition.
 * 
 * THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 * WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 * LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL, 
 * INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 * PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 * BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 * THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 * SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 * (INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */

/**
 * Description:
 * The MotionMagic example demonstrates the motion magic control mode.
 * Tested with Logitech F710 USB Gamepad inserted into Driver Station.
 * 
 * Be sure to select the correct feedback sensor using configSelectedFeedbackSensor() below.
 *
 * After deploying/debugging this to your RIO, first use the left Y-stick 
 * to throttle the Talon manually. This will confirm your hardware setup/sensors
 * and will allow you to take initial measurements.
 * 
 * Be sure to confirm that when the Talon is driving forward (green) the 
 * position sensor is moving in a positive direction. If this is not the 
 * cause, flip the boolean input to the setSensorPhase() call below.
 *
 * Ensure your feedback device is in-phase with the motor,
 * and you have followed the walk-through in the Talon Software Reference Manual.
 * 
 * Controls:
 * Button 1(Button A): When held, put Talon in Motion Magic mode and allow Talon to drive [-10, 10] 
 * 	rotations.
 * Button 2(Button B): When pushed, the selected feedback sensor gets zero'd
 * POV 180(Dpad Down): When pushed, will decrement the smoothing of the motion magic down to 0
 * POV 0(Dpad Up): When pushed, will increment the smoothing of the motion magic up to 8
 * Left Joystick Y-Axis:
 * 	+ Percent Output: Throttle Talon FX forward and reverse, use to confirm hardware setup.
 * Right Joystick Y-Axis:
 * 	+ Motion Maigic: Servo Talon FX forward and reverse, [-10, 10] rotations.
 * 
 * Gains for Motion Magic may need to be adjusted in Constants.java
 * 
 * Supported Version:
 * - Talon FX: 20.2.3.0
 */
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import frc.robot.sim.PhysicsSim;

public class Robot extends TimedRobot {

	/* Tuning - set points */
	double m_targetMin = 10;
	double m_targetMax = 600;

	/* Hardware */
	WPI_TalonFX _talon = new WPI_TalonFX(1); // Use the faster CanFD bus with a CANivore to reduce bandwidth utilization
	/* Tuning - There can be follower motors defined here */
	Joystick _joy = new Joystick(0);

	/* Used to build string throughout loop */
	StringBuilder _sb = new StringBuilder();

	/** How much smoothing [0,8] to use during MotionMagic */
	int _smoothing = 0;

	/** save the last Point Of View / D-pad value */
	int _pov = -1;

	public void simulationInit() {
		PhysicsSim.getInstance().addTalonFX(_talon, 0.5, 5100);
	}
	public void simulationPeriodic() {
		PhysicsSim.getInstance().run();
	}

	public void robotInit() {
		/* Factory default hardware to prevent unexpected behavior */
		_talon.configFactoryDefault();

		/* Configure Sensor Source for Pirmary PID */
		_talon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx,
				Constants.kTimeoutMs);

		/* set deadband to super small 0.001 (0.1 %).
			The default deadband is 0.04 (4 %) */
		_talon.configNeutralDeadband(0.001, Constants.kTimeoutMs);

		/**
		 * Configure Talon FX Output and Sensor direction accordingly Invert Motor to
		 * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
		 * sensor to have positive increment when driving Talon Forward (Green LED)
		 */
		/* Tuning - Check the sensor phase, flip it if it is backwards */
		_talon.setSensorPhase(false);
		_talon.setInverted(false);
		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
		 */
        // _talon.setSensorPhase(true);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

		/* Set the peak and nominal outputs */
		_talon.configNominalOutputForward(0, Constants.kTimeoutMs);
		_talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
		_talon.configPeakOutputForward(1, Constants.kTimeoutMs);
		_talon.configPeakOutputReverse(-1, Constants.kTimeoutMs);

		/* Set Motion Magic gains in slot0 - see documentation */
		_talon.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		/* Tuning - zero out constant values here for step tuning */
		_talon.config_kF(Constants.kSlotIdx, Constants.kGains.kF, Constants.kTimeoutMs);
		_talon.config_kP(Constants.kSlotIdx, Constants.kGains.kP, Constants.kTimeoutMs);
		_talon.config_kI(Constants.kSlotIdx, Constants.kGains.kI, Constants.kTimeoutMs);
		_talon.config_kD(Constants.kSlotIdx, Constants.kGains.kD, Constants.kTimeoutMs);

		/* Set acceleration and vcruise velocity - see documentation */
		/* Tuning - zero out following values here to start with fresh numbers relevant to our robot */
		_talon.configMotionCruiseVelocity(15000, Constants.kTimeoutMs);
		_talon.configMotionAcceleration(6000, Constants.kTimeoutMs);

		/* Zero the sensor once on robot boot up */
		_talon.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

		_talon.configFeedbackNotContinuous(true, Constants.kTimeoutMs);

		/* config current limits */
		_talon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 20, 1));

		/* config intergral zone */
		_talon.config_IntegralZone(Constants.kSlotIdx, 3);
	}

	/**
	 * !! Important !! -- Safety must for any robotic arms that rotate
	 * Tuning -- prevent arm to go back the postion if the arm positon moved manually during robot being disabled.
	 */
	@Override
	public void teleopInit() {
		_talon.set(ControlMode.PercentOutput, 0);
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		/* Get gamepad axis - forward stick is positive */
		double leftYstick = -1.0 * _joy.getY(); /* left-side Y for Xbox360Gamepad */
		double rghtYstick = -1.0 * _joy.getRawAxis(5); /* right-side Y for Xbox360Gamepad */
		if (Math.abs(leftYstick) < 0.10) { leftYstick = 0; } /* deadband 10% */
		if (Math.abs(rghtYstick) < 0.10) { rghtYstick = 0; } /* deadband 10% */

		/* Get current Talon FX motor output */
		double motorOutput = _talon.getMotorOutputPercent();

		/* Prepare line to print */
		_sb.append("\tOut%:");
		_sb.append(motorOutput);
		_sb.append("\tVel:");
		_sb.append(_talon.getSelectedSensorVelocity(Constants.kPIDLoopIdx));

		/* Tuning - add more data points*/
		_sb.append("\tPosition:");
		_sb.append(_talon.getSelectedSensorPosition());
		SmartDashboard.putNumber("Left Position", _talon.getSelectedSensorPosition());

		/**
		 * Tuning - Use arbitrary feed foward
		 */
		// double horizontalHoldOutput = .13;
		// double arbfeedFwdTerm = getFeedForward(horizontalHoldOutput);

		/**
		 * Perform Motion Magic when Button 1 is held, else run Percent Output, which can
		 * be used to confirm hardware setup.
		 */
		if (_joy.getRawButton(1)) {
			/* Motion Magic */

			/* 2048 ticks/rev * 10 Rotations in either direction */
			/* Tuning - setting target position */
			//double targetPos = rghtYstick * 2048 * 10.0;
			double targetPos = m_targetMin;
			_talon.set(TalonFXControlMode.MotionMagic, targetPos);				//, DemandType.ArbitraryFeedForward, horizongtalHoldOutput);
																				//, DemandType.ArbitraryFeedForward, arbfeedFwdTerm);

			/* Append more signals to print when in speed mode */
			_sb.append("\terr:");
			_sb.append(_talon.getClosedLoopError(Constants.kPIDLoopIdx));
			_sb.append("\ttrg:");
			_sb.append(targetPos);
		} else if (_joy.getRawButton(4)) {
			/* Motion Magic */

			/* Tuning - setting target position */
			double targetPos = m_targetMax;
			_talon.set(TalonFXControlMode.MotionMagic, targetPos);

			/* Append more signals to print when in speed mode */
			_sb.append("\terr:");
			_sb.append(_talon.getClosedLoopError(Constants.kPIDLoopIdx));
			_sb.append("\ttrg:");
			_sb.append(targetPos);
		} else {
			/* Percent Output */

			_talon.set(TalonFXControlMode.PercentOutput, leftYstick);
		}
		if (_joy.getRawButton(6)) {
			/* Zero sensor positions */
			_talon.setSelectedSensorPosition(0);
		}

		int pov = _joy.getPOV();
		if (_pov == pov) {
			/* no change */
		} else if (_pov == 180) { // D-Pad down
			/* Decrease smoothing */
			_smoothing--;
			if (_smoothing < 0)
				_smoothing = 0;
			_talon.configMotionSCurveStrength(_smoothing);

			System.out.println("Smoothing is set to: " + _smoothing);
		} else if (_pov == 0) { // D-Pad up
			/* Increase smoothing */
			_smoothing++;
			if (_smoothing > 8)
				_smoothing = 8;
			_talon.configMotionSCurveStrength(_smoothing);

			System.out.println("Smoothing is set to: " + _smoothing);
		}
		_pov = pov; /* save the pov value for next time */

		/* Instrumentation */
		Instrum.Process(_talon, _sb);
	}
	
	/** 
	 * Turning - example of calculating arb-feedforward 
	 * 
	 * @param horizontalHoldOutput 
	 * @return
	 * 
	 * */ 
	private double getFeedForward(double horizontalHoldOutput) {

		// Get the radians of the arm
		// my_getCurrentArmAnge() returns degree
		double theta = Math.toRadians(90 - my_getCurrentArmAnge());

		// get a range of 0 to 1 to multiply by feedforward
		// when in horizontal position, value should be 1
		// when in vertical position, value should be 0
		double gravityCompensation = Math.cos(theta);

		// HorizontalHoldOutput is the minimum power required to hold the arm horizontal,
		// it shold be in the range of 0 - 1, for example: it requires .125 throttle to keep the arm up
		double arb_feedForward = gravityCompensation * horizontalHoldOutput;

		return arb_feedForward;

	}
	
	private double my_getCurrentArmAnge() {

		double my_fullRange = 150;
		double my_offset = -2.3;
		double my_sensorPos = _talon.getSelectedSensorPosition(0);

		double my_angle = (my_sensorPos / 1024) * my_fullRange + my_offset;

		return my_angle;
	}
	
	

}
