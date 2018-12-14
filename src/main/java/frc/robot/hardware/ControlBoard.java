
package frc.robot.hardware;

import edu.wpi.first.wpilibj.Joystick;
/** Handles inputs from controller 
 * @category hardware
 * @author Peter I. Chacko, Eric Warner, John Sullivan Jr, Jake Boothby, Elliot
 *         Friedlander
 * 
 * **/
public class ControlBoard extends RemoteControl {
	//Operator Buttons
/** Operator Buttons **/
	public ButtonReader armSwitchButton, armScaleButton, armFeedButton, intakeOperatorButton, outtakeOperatorButton, intakeDriverButton, outtakeDriverButton;
	/*** Arm Override Trigger ***/
	public ToggleButtonReader armManualButton;
/** Driver Triggers **/
	public TriggerReader slowDriveTier1Button, slowDriveTier2Button, outtakeWheelsButton, intakeWheelsButton;
	/*** Booleans for relax wrist TODO for offseason streamline ***/
	private boolean slowDriveTier1Desired, slowDriveTier2Desired, toggleArmManualDesired, armSwitchDesired, armScaleDesired, armFeedDesired, intakeDesired, outtakeDesired, armShifterDesired, outtakeWheelsDesired, intakeWheelsDesired;

	/** Driver joystick axes **/
	private double driverLeftJoyX, driverLeftJoyY, driverRightJoyX, driverRightJoyY;
	/** Operator joystick axes **/
	private double operatorLeftJoyX, operatorLeftJoyY, operatorRightJoyX, operatorRightJoyY;
	/** Joystick **/
	private Joystick driverJoy, operatorJoy;
	private boolean intakeManualDesired;

	/** Initializes all controller inputs **/
	public ControlBoard() {
		driverJoy = new Joystick(Ports.DRIVER_JOY_USB_PORT);
		operatorJoy = new Joystick(Ports.OPERATOR_JOY_USB_PORT);

		if (Ports.USING_WIN_DRIVER_STATION) {
			//Driver Controls
			
			intakeDriverButton = new ButtonReader(driverJoy, XInput.XINPUT_WIN_LEFT_BUMPER, "INTAKE_DRIVER");
			outtakeDriverButton = new ButtonReader(driverJoy, XInput.XINPUT_WIN_RIGHT_BUMPER, "OUTTAKE_DRIVER");
			
			slowDriveTier1Button = new TriggerReader(driverJoy, XInput.XINPUT_WIN_RIGHT_TRIGGER_AXIS, "BRAKE_1");
			slowDriveTier2Button = new TriggerReader(driverJoy, XInput.XINPUT_WIN_LEFT_TRIGGER_AXIS, "BRAKE_2");
			
			//Operator Controls
			armScaleButton = new ButtonReader(operatorJoy, XInput.XINPUT_WIN_YELLOW_BUTTON, "SCALE");
			armSwitchButton = new ButtonReader(operatorJoy, XInput.XINPUT_WIN_GREEN_BUTTON, "SWITCH");
			armFeedButton = new ButtonReader(operatorJoy, XInput.XINPUT_WIN_BLUE_BUTTON, "FEED");
			armManualButton = new ToggleButtonReader(operatorJoy, XInput.XINPUT_WIN_BACK_BUTTON, "ARM_MANUAL");
			
			intakeWheelsButton = new TriggerReader(operatorJoy, XInput.XINPUT_WIN_LEFT_TRIGGER_AXIS, "IN_WHEELS");
			outtakeWheelsButton = new TriggerReader(operatorJoy, XInput.XINPUT_WIN_RIGHT_TRIGGER_AXIS, "OUT_WHEELS");
			intakeOperatorButton = new ButtonReader(operatorJoy, XInput.XINPUT_WIN_RIGHT_BUMPER, "INTAKE_OP");
			outtakeOperatorButton = new ButtonReader(operatorJoy, XInput.XINPUT_WIN_LEFT_BUMPER, "OUTTAKE_OP");
			
			
		}

		driverLeftJoyX = 0;
		driverLeftJoyY = 0;
		driverRightJoyX = 0;
		driverRightJoyY = 0;

		// Driver variableS		
		slowDriveTier1Desired = false;
		slowDriveTier2Desired = false;
		
		
		//Operator Vars
		armSwitchDesired = false;
		armScaleDesired = false;
		armFeedDesired = false;
		toggleArmManualDesired = false;
		
		intakeDesired = false;
		outtakeDesired = false;
		outtakeWheelsDesired = false;
		intakeWheelsDesired = false;
	}
	/** Reads all controller inputs **/
	public void readControls() {
		readAllButtons();
		if (Ports.USING_WIN_DRIVER_STATION) {
			driverLeftJoyX = driverJoy.getRawAxis(XInput.XINPUT_WIN_LEFT_X_AXIS);
			driverLeftJoyY = -driverJoy.getRawAxis(XInput.XINPUT_WIN_LEFT_Y_AXIS);
			driverRightJoyX = driverJoy.getRawAxis(XInput.XINPUT_WIN_RIGHT_X_AXIS);
			driverRightJoyY = -driverJoy.getRawAxis(XInput.XINPUT_WIN_RIGHT_Y_AXIS);

			operatorLeftJoyX = operatorJoy.getRawAxis(XInput.XINPUT_WIN_LEFT_X_AXIS);
			operatorLeftJoyY = -operatorJoy.getRawAxis(XInput.XINPUT_WIN_LEFT_Y_AXIS);
			operatorRightJoyX = operatorJoy.getRawAxis(XInput.XINPUT_WIN_RIGHT_X_AXIS);
			operatorRightJoyY = -operatorJoy.getRawAxis(XInput.XINPUT_WIN_RIGHT_Y_AXIS);
		}

		// Driver Variables

		slowDriveTier1Desired = slowDriveTier1Button.isDown();
		slowDriveTier2Desired = slowDriveTier2Button.isDown();
	
		//Operator Vars
		armSwitchDesired = armSwitchButton.isDown();
		armScaleDesired = armScaleButton.isDown();
		armFeedDesired = armFeedButton.isDown();
		
		toggleArmManualDesired = armManualButton.getState();
		
		intakeDesired = intakeDriverButton.isDown() || intakeOperatorButton.isDown();
		outtakeDesired = outtakeDriverButton.isDown() || outtakeOperatorButton.isDown();
		
		outtakeWheelsDesired = outtakeWheelsButton.isDown();
		intakeWheelsDesired = intakeWheelsButton.isDown();

	}
	/** Reads all controller buttons **/
	public void readAllButtons() {
		//Driver
		slowDriveTier1Button.readValue();
		slowDriveTier2Button.readValue();
		
		//Operator 
		
		armSwitchButton.readValue();
		armScaleButton.readValue();
		armFeedButton.readValue();
		armManualButton.readValue();
		
		intakeDriverButton.readValue();
		outtakeDriverButton.readValue();
		intakeOperatorButton.readValue();
		outtakeOperatorButton.readValue();
		
		outtakeWheelsButton.readValue();
		intakeWheelsButton.readValue();

	}

	/** Gets joystick value given joystick  and axe 
	 * 
	 * @param j A Joystick
	 * @param a An Axis
	 * 
	 * **/
	public double getJoystickValue(Joysticks j, Axes a) {
		switch (j) {
		case kDriverJoy:
			if (a == Axes.kLX) {
				return driverLeftJoyX;
			} else if (a == Axes.kLY) {
				return driverLeftJoyY;
			} else if (a == Axes.kRX) {
				return driverRightJoyX;
			} else if (a == Axes.kRY) {
				return driverRightJoyY;
			}
			break;
		case kOperatorJoy:
			if (a == Axes.kLX) {
				return operatorLeftJoyX;
			} else if (a == Axes.kLY) {
				return operatorLeftJoyY;
			} else if (a == Axes.kRX) {
				return operatorRightJoyX;
			} else if (a == Axes.kRY) {
				return operatorRightJoyY;
			}
			break;
		default:
			return 0.0;
		}
		return 0.0;
	}
	
	/*
	 * Commented in RemoteControl.java
	 * (non-Javadoc)
	 * 
	 */
	
	
	@Override
	public boolean getSlowDriveTier1Desired() {
		return slowDriveTier1Desired;
	}
	@Override
	public boolean getSlowDriveTier2Desired() {
		return slowDriveTier2Desired;
	}	
	
	@Override
	public boolean toggleManualArmDesired() {
		return toggleArmManualDesired;
	}

	@Override
	public boolean getSwitchArmDesired() {

		return armSwitchDesired;
	}

	@Override
	public boolean getScaleArmDesired() {
		return armScaleDesired;
	}

	@Override
	public boolean getFeedArmDesired() {

		return armFeedDesired;
	}


	@Override
	public boolean getIntakeDesired() {

		return intakeDesired;
	}

	@Override
	public boolean getOuttakeDesired() {

		return outtakeDesired;
	}
	
	
	@Override
	public boolean outtakeWheels() {
		return outtakeWheelsDesired;
	}
	@Override
	public boolean intakeWheels() {
		return intakeWheelsDesired;
	}



}
