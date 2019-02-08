
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
	public ButtonReader intakeCargoButton, shootHighCargoButton, shootLowCargoButton, shootMidCargo, scoreHatchButton, autoAlignCargoButton, autoAlignTapeButton, loadHatchButton, floorHatchButton;
	/*** Arm Override Trigger ***/
	public ToggleButtonReader armManualButton;
/** Driver Triggers **/
	public TriggerReader slowDriveTier1Button, slowDriveTier2Button, outtakePistonsButton;

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
			
			autoAlignCargoButton = new ButtonReader(driverJoy, XInput.XINPUT_WIN_LEFT_BUMPER, "CARGO_ALIGN");
			autoAlignTapeButton = new ButtonReader(driverJoy, XInput.XINPUT_WIN_RIGHT_BUMPER, "TAPE_ALIGN");
			
			slowDriveTier1Button = new TriggerReader(driverJoy, XInput.XINPUT_WIN_RIGHT_TRIGGER_AXIS, "BRAKE_1");
			slowDriveTier2Button = new TriggerReader(driverJoy, XInput.XINPUT_WIN_LEFT_TRIGGER_AXIS, "BRAKE_2");
			intakeCargoButton = new ButtonReader(driverJoy, XInput.XINPUT_WIN_LEFT_STICK_CLICK, "INTAKE_CARGO");

			//Operator Controls
			shootHighCargoButton = new ButtonReader(operatorJoy, XInput.XINPUT_WIN_YELLOW_BUTTON, "HIGH_CARGO");
			shootMidCargo = new ButtonReader(operatorJoy, XInput.XINPUT_WIN_BLUE_BUTTON, "MID_CARGO");
			shootLowCargoButton = new ButtonReader(operatorJoy, XInput.XINPUT_WIN_GREEN_BUTTON, "LOW_CARGO");
			armManualButton = new ToggleButtonReader(operatorJoy, XInput.XINPUT_WIN_BACK_BUTTON, "ARM_MANUAL");
			
			outtakePistonsButton = new TriggerReader(operatorJoy, XInput.XINPUT_WIN_RIGHT_TRIGGER_AXIS, "PUSH PISTON");
			scoreHatchButton = new ButtonReader(operatorJoy, XInput.XINPUT_WIN_RIGHT_BUMPER, "SCORE_HATCH");
			floorHatchButton = new ButtonReader(operatorJoy, XInput.XINPUT_WIN_LEFT_BUMPER, "FLOOR_HATCH");
			loadHatchButton = new ButtonReader(operatorJoy, XInput.XINPUT_WIN_RED_BUTTON, "LOAD_HATCH");

			
			
		}

		driverLeftJoyX = 0;
		driverLeftJoyY = 0;
		driverRightJoyX = 0;
		driverRightJoyY = 0;

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


	}
	/** Reads all controller buttons **/
	public void readAllButtons() {
		//Driver
		slowDriveTier1Button.readValue();
		slowDriveTier2Button.readValue();
		
		//Operator 
		
		intakeCargoButton.readValue();
		shootHighCargoButton.readValue();
		shootLowCargoButton.readValue();
		armManualButton.readValue();
		
		autoAlignCargoButton.readValue();
		autoAlignTapeButton.readValue();
		shootMidCargo.readValue();
		scoreHatchButton.readValue();
		
		outtakePistonsButton.readValue();

		floorHatchButton.readValue();
		loadHatchButton.readValue();

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
		return slowDriveTier1Button.isDown();
	}
	@Override
	public boolean getSlowDriveTier2Desired() {
		return slowDriveTier2Button.isDown();
	}	
	
	@Override
	public boolean toggleManualArmDesired() {
		return armManualButton.getState();
	}

	

	@Override
	public boolean intakeCargo() {
		return intakeCargoButton.isDown();
	}

	@Override
	public boolean shootHighCargo() {
		return shootHighCargoButton.isDown();
	}

	@Override
	public boolean shootLowCargo() {
		return shootLowCargoButton.isDown();
	}

	@Override
	public boolean shootMidCargo() {
		return shootMidCargo.isDown();
	}

	@Override
	public boolean floorHatch() {
		return floorHatchButton.isDown();
	}

	@Override
	public boolean scoreHatch() {
		return scoreHatchButton.isDown();
	}

	@Override
	public boolean loadHatch() {
		return loadHatchButton.isDown();
	}

	@Override
	public boolean outtakePistons() {
		return outtakePistonsButton.isDown();
	}

	@Override
	public boolean getCargoVisionDesired() {
		return autoAlignCargoButton.isDown();
	}

	@Override
	public boolean getTapeVisionDesired() {
		return autoAlignTapeButton.isDown();
	}



}
