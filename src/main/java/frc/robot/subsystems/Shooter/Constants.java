/**
 * Simple class containing constants used throughout project
 */
package frc.robot.subsystems.Shooter;

class Constants {
	/**
	 * Which PID slot to pull gains from. Starting 2018, you can choose from
	 * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
	 * configuration.
	 */
	public static final int kSlotIdx = 0;

	/**
	 * Talon FX supports multiple (cascaded) PID loops. For
	 * now we just want the primary one.
	 */
	public static final int kPIDLoopIdx = 0;

	/**
	 * Set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
    public static final int kTimeoutMs = 30;

	/**
	 * PID Gains may have to be adjusted based on the responsiveness of control loop.
     * kF: 1023 represents output value to Talon at 100%, 20660 represents Velocity units at 100% output
     * 
	 * 	                                    			  kP kI kD      kF          Iz    PeakOut */
   public final static Gains kGains_Velocit  = new Gains( 0.18, 0, 5.4, (0.60*1023)/12204,  300,  1.00); //kp 5, kD 20 on 75%
   public final static Gains k2Gains_Velocit = new Gains( 0.04, 0, 1.46, (0.4*1023)/8552,  300,  1.00);

   
}
//currently p 0.5, d 20 and going rly zigzaggy (was less zigzaggy when it was d10) -> maybe increase p?
 //make it 2046
 //(0.75*2046)/20300
 //kp should be 0.8?? but its super spiky dk if im doing it right