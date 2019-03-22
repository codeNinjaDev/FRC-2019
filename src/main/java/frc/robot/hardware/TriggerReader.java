package frc.robot.hardware;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/*** Reads Trigger Input *****/
public class TriggerReader {
	private Joystick joystick;
	private int triggerAxis;
	private boolean lastState;
	private boolean currState;
	private String triggerName;
	private ShuffleboardTab controllerTab; 
	private NetworkTableEntry triggerStatus;
	private NetworkTableEntry triggerValue;
	private double triggerNumber;
	public TriggerReader(Joystick joystick, int triggerAxis, String triggerName) {
		controllerTab = Shuffleboard.getTab("Controllers");

		this.joystick = joystick;
		this.triggerAxis = triggerAxis;
		this.triggerName = triggerName + "_TRIGGER";
		triggerNumber = joystick.getRawAxis(triggerAxis);
		if(triggerNumber > 0.8) {
			currState = true;
		} else {
			currState = false;
		}
		lastState = currState;
		triggerStatus = controllerTab.add(triggerName, currState).getEntry();
		triggerValue = controllerTab.add(triggerName + "_AXIS", triggerNumber).getEntry();


	}
	
	public void readValue() {
		lastState = currState;
		triggerNumber = joystick.getRawAxis(triggerAxis);
		triggerValue.setDouble(triggerNumber);
		if(triggerNumber > 0.8) {
			currState = true;
		} else {
			currState = false;
		}
		triggerStatus.setBoolean(currState);

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
		return currState;
	}

}
