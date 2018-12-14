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
	/*** OUTTAKE WHEELS  --Back Button ***/
	public abstract boolean outtakeWheels();
	/*** INTAKE WHEELS  --Back Button ***/
	public abstract boolean intakeWheels();
	/*** Go to Switch Position (PID) --Green (A) Button ***/
	public abstract boolean getSwitchArmDesired();
	/*** Go to Scale Position (PID) --Yellow (Y) Button***/
	public abstract boolean getScaleArmDesired();
	/*** Go to Vault Position (PID) --Blue (X) Button***/
	public abstract boolean getFeedArmDesired();
	
	/*** Intake block --Right Bumper***/
	public abstract boolean getIntakeDesired();
	/*** Outtake block --Left Bumper***/
	public abstract boolean getOuttakeDesired();

}
