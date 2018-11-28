package frc.robot.hardware;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/*** Creates a toggle button on gamepad ***/
public class ToggleButtonReader extends ButtonReader {
	private boolean currToggleState;
	private String toggleName;
	public ToggleButtonReader(Joystick joy, int buttonNum, String toggleName) {
		super(joy, buttonNum, toggleName);
		this.toggleName = toggleName + "_TOGGLE";
		currToggleState = false;
	}
	public boolean getState() {
		if (wasJustReleased()) {
			currToggleState = !currToggleState;
		}
		SmartDashboard.putBoolean(toggleName, currToggleState);
		return (currToggleState);
	}
}