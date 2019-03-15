package frc.robot.hardware;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/*** Creates a toggle button on gamepad ***/
public class ToggleButtonReader extends ButtonReader {
	private boolean currToggleState;
	private String toggleName;
	private ShuffleboardTab controllerTab; 
	private NetworkTableEntry toggleStatus;

	public ToggleButtonReader(Joystick joy, int buttonNum, String toggleName) {
		super(joy, buttonNum, toggleName);
		controllerTab = Shuffleboard.getTab("Controllers");

		this.toggleName = toggleName + "_TOGGLE";
		currToggleState = false;
		toggleStatus = controllerTab.add(this.toggleName, currToggleState).getEntry();

	}
	public boolean getState() {
		if (wasJustReleased()) {
			currToggleState = !currToggleState;
		}
		toggleStatus.setBoolean(currToggleState);
		return (currToggleState);
	}
}