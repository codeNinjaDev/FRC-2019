package frc.robot.hardware;

/*** Abstract class for all input for robot functionality ***/
public abstract class RemoteControl {
	/*** Whose controllers (driver or operator)
	 * 
	 * @author peter
	 *
	 */
	public enum Joysticks {
		kDriverJoy, kOperatorJoy
	};
	/*** What joystick axes (left x or right y) ***/
	public enum Axes {
		kLX, kLY, kRX, kRY
	};
	/*** Update Values ***/
	public abstract void readControls();
	/*** Update button values ***/
	public abstract void readAllButtons();
	/*** Get value of joystick axis ***/
	public abstract double getJoystickValue(Joysticks j, Axes a);

	/*** Brake 1 (Slow down a little) --Right Trigger ***/
	public abstract boolean getSlowDriveTier1Desired();
	/*** Brake 2 (Slow down a lot) --Left Trigger ***/
	public abstract boolean getSlowDriveTier2Desired();

	
	/*** ARM Manual Override  --Back Button ***/
	public abstract boolean toggleManualArmDesired();

	/*** Go to intake cargo (PID) --Driver Green button ***/
	public abstract boolean intakeCargo();
	/*** Go to shoot high cargo (PID) -- START Button***/
	public abstract boolean shootHighCargo();
	/*** Go to shoot low (PID) --Operator RIGHT BUMPER***/
	public abstract boolean shootLowCargo();
	/*** Go to shoot mid (PID) --Operator LEFT BUMPER***/
	public abstract boolean shootMidCargo();
	/*** Go to outtake cargo  --Operator Right Bumper Button***/
	/*** Go to punch Cargo  --Driver Blue Button***/
	public abstract boolean outtakeCargo();

	/*** Go to shoot high cargo (PID) -- Operator Yellow (Y) Button***/
	public abstract boolean aimRocketMid();
	/*** Go to shoot low (PID) --Operator Green (A) Button***/
	public abstract boolean aimCargoBay();
	/*** Go to shoot mid (PID) --Operator Blue (X) Button***/
	public abstract boolean aimBackwardsCargoBay();
	/*** Go to shoot mid (PID) --Operator Red (B) Button***/
	public abstract boolean customWristPosition();
	

	/*** Punch Ball  --**/
	public abstract boolean punchBall();

	/*** Go to push hatch --Operator Left trigger Button***/
	public abstract boolean pushHatch();

	/*** Align with cargo Driver --Right Bumper***/
	public abstract boolean getCargoVisionDesired();
	/*** Align with tape Driver --Left Bumper***/
	public abstract boolean getTapeVisionDesired();
	/*** XBox controller button - Allow Climb */
	public abstract boolean climbDesired();

	

}
