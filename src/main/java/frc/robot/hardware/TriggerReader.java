package frc.robot.hardware;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/*** Reads Trigger Input *****/
public class TriggerReader {
	private Joystick joystick;
	private int triggerAxis;
	private boolean lastState;
	private boolean currState;
	private String triggerName;
	public TriggerReader(Joystick joystick, int triggerAxis, String triggerName) {
		this.joystick = joystick;
		this.triggerAxis = triggerAxis;
		this.triggerName = triggerName + "_TRIGGER";
		if(joystick.getRawAxis(triggerAxis) > 0.8) {
			currState = true;
		} else {
			currState = false;
		}
		lastState = currState;
	}
	
	public void readValue() {
		lastState = currState;
		SmartDashboard.putNumber(triggerName + "_AXIS", joystick.getRawAxis(triggerAxis));
		if(joystick.getRawAxis(triggerAxis) > 0.8) {
			currState = true;
		} else {
			currState = false;
		}
	}
	
	public boolean wasJustPressed() {
		return (lastState == false && currState == true);
	}
	
	public boolean wasJustReleased() {
		return (lastState == true && currState == false);
	}
	
	public boolean stateJustChanged() {
		return (lastState != currState);
	}
	
	public boolean isDown() {
		SmartDashboard.putBoolean(triggerName, currState);
		return currState;
	}

}
