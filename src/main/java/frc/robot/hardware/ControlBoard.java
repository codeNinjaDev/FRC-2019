
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
	public ButtonReader miscWristPosition, cargoBayButton, backCargoBayButton, rocketMidButton, intakeCargoButton, driverOuttakeCargoButton, shootHighCargoButton, shootLowCargoButton, shootMidCargo, operatorOuttakeCargoButton, autoAlignCargoButton, autoAlignTapeButton;
	/*** Arm Override Trigger ***/
	public ToggleButtonReader climbButton, armManualButton;
/** Driver Triggers **/
	public TriggerReader slowDriveTier1Button, slowDriveTier2Button, punchBallButton, pushHatchButton;

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
			intakeCargoButton = new ButtonReader(driverJoy, XInput.XINPUT_WIN_GREEN_BUTTON, "INTAKE_CARGO");
			climbButton = new ToggleButtonReader(driverJoy, XInput.XINPUT_WIN_X_BUTTON, "CLIMB");
			//Operator Controls
			shootHighCargoButton = new ButtonReader(operatorJoy, XInput.XINPUT_WIN_START_BUTTON, "FAST_CARGO");
			shootMidCargo = new ButtonReader(operatorJoy, XInput.XINPUT_WIN_LEFT_BUMPER, "MID_CARGO");
			shootLowCargoButton = new ButtonReader(operatorJoy, XInput.XINPUT_WIN_RIGHT_BUMPER, "SLOWER_CARGO");

			cargoBayButton = new ButtonReader(operatorJoy, XInput.XINPUT_WIN_GREEN_BUTTON, "CARGO_BAY_POSITION");
			rocketMidButton = new ButtonReader(operatorJoy, XInput.XINPUT_WIN_YELLOW_BUTTON, "ROCKET_MID_POSITION");
			backCargoBayButton = new ButtonReader(operatorJoy, XInput.XINPUT_WIN_BLUE_BUTTON, "SHOOTING_BACK_POSITION");
			miscWristPosition = new ButtonReader(operatorJoy, XInput.XINPUT_WIN_RED_BUTTON, "CUSTOM_WRIST_POSITION");

			armManualButton = new ToggleButtonReader(operatorJoy, XInput.XINPUT_WIN_BACK_BUTTON, "ARM_MANUAL");
			driverOuttakeCargoButton = new ButtonReader(driverJoy, XInput.XINPUT_WIN_BLUE_BUTTON, "OUTTAKE_CARGO");
			punchBallButton = new TriggerReader(operatorJoy, XInput.XINPUT_WIN_RIGHT_TRIGGER_AXIS, "PUNCH PISTON");
			pushHatchButton = new TriggerReader(operatorJoy, XInput.XINPUT_WIN_RIGHT_TRIGGER_AXIS, "PUSH PISTON");


			operatorOuttakeCargoButton = new ButtonReader(operatorJoy, XInput.XINPUT_WIN_RIGHT_BUMPER, "Outtake Cargo");
			
			
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
		climbButton.readValue();
		intakeCargoButton.readValue();

		//Operator 
		
		shootHighCargoButton.readValue();
		shootMidCargo.readValue();
		shootLowCargoButton.readValue();
		armManualButton.readValue();

		cargoBayButton.readValue();
		backCargoBayButton.readValue();
		rocketMidButton.readValue();
		miscWristPosition.readValue();
		
		autoAlignCargoButton.readValue();
		autoAlignTapeButton.readValue();
		operatorOuttakeCargoButton.readValue();
		
		punchBallButton.readValue();
		pushHatchButton.readValue();
		driverOuttakeCargoButton.readValue();
		

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
	public boolean outtakeCargo() {
		return driverOuttakeCargoButton.isDown() || operatorOuttakeCargoButton.isDown();
	}



	@Override
	public boolean punchBall() {
		return punchBallButton.isDown();
	}

	@Override
	public boolean getCargoVisionDesired() {
		return autoAlignCargoButton.isDown();
	}

	@Override
	public boolean getTapeVisionDesired() {
		return autoAlignTapeButton.isDown();
	}

	@Override
	public boolean climbDesired() {
		return climbButton.isDown();
	}

	@Override
	public boolean pushHatch() {
		return pushHatchButton.isDown();
	}

	@Override
	public boolean aimRocketMid() {
		return rocketMidButton.isDown();
	}

	@Override
	public boolean aimCargoBay() {
		return cargoBayButton.isDown();
	}

	@Override
	public boolean aimBackwardsCargoBay() {
		return backCargoBayButton.isDown();
	}

	@Override
	public boolean customWristPosition() {
		return miscWristPosition.isDown();
	}



}
