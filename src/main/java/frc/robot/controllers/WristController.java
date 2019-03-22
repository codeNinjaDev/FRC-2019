package frc.robot.controllers;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.pid.*;
import frc.robot.Params;
import frc.robot.hardware.*;
import frc.robot.hardware.RemoteControl.Joysticks;

/*** Controls mechanism for both cargo and hatch */
public class WristController extends Subsystem {
	private VictorSP leftIntakeMotor, rightIntakeMotor;
	private SpeedControllerGroup intakeMotors;

	private RemoteControl humanControl;

	private VictorSP armMotorA, armMotorB;
	private SpeedControllerGroup armMotors;

	private Solenoid shooterPiston;
	public Solenoid outtakePiston;
	public Encoder wristPotentiometer;

	private PIDOutput armPIDOutput;
	public PIDController armPID;

	/*** Current state of robot; e.g init or teleop ***/
	private ArmState m_stateVal;
	/*** Next state of robot; e.g init or teleop ***/
	private ArmState nextState;
	/*** Boolean for override arm control ***/
	private boolean toggleArmManual;
	private double feedforward;

	/*** Enum that lets us shift our status from init to teleop and vice versa ***/
	public enum ArmState {
		kInitialize, kTeleop
	};

	// IS going down positive or negative?
	public WristController(RemoteControl humanControl) {
		this.humanControl = humanControl;

		wristPotentiometer = new Encoder(Ports.WRIST_POT[0], Ports.WRIST_POT[1]);
		wristPotentiometer.setDistancePerPulse(360 / Params.PULSES_PER_ROTATION);
		leftIntakeMotor = new VictorSP(Ports.LEFT_INTAKE_WHEEL_PORT);
		rightIntakeMotor = new VictorSP(Ports.RIGHT_INTAKE_WHEEL_PORT);
		//Physically inverted
		rightIntakeMotor.setInverted(false);

		intakeMotors = new SpeedControllerGroup(leftIntakeMotor, rightIntakeMotor);
		intakeMotors.setInverted(true);

		armMotorA = new VictorSP(Ports.WRIST_ARM_A_PORT);
		armMotorB = new VictorSP(Ports.WRIST_ARM_B_PORT);
		//Physically inverted
		armMotorA.setInverted(false);
		armMotors = new SpeedControllerGroup(armMotorA, armMotorB);
		armMotors.setInverted(true);
		outtakePiston = new Solenoid(Ports.OUTTAKE_SOLENOIDS[0]);

		shooterPiston = new Solenoid(Ports.SHOOTER_SOLENOIDS[0]);

		armPIDOutput = new WristPIDOutput(armMotors);
		armPID = new PIDController(0, 0, 0, wristPotentiometer, armPIDOutput);
		m_stateVal = ArmState.kInitialize;
		nextState = ArmState.kInitialize;


	}


	public void stop() {
		armMotors.set(0);
		armPID.disable();
	}

	public void reset() {
		m_stateVal = ArmState.kInitialize;
		wristPotentiometer.reset();
	}

	public void update() {
		feedforward = Params.arm_f; //* (120 - wristPotentiometer.getDistance())
		System.out.println("running");

		// Switches based off of current state
		switch (m_stateVal) {
		// If initializing
		case kInitialize:
			System.out.println("init");

			// Intitalize Variables and PId
			armPID.setPID(Params.arm_p, Params.arm_i, Params.arm_d, feedforward);
			armPID.setOutputRange(-1, 1);
			armPID.setSetpoint(Params.ARM_STOW_SETPOINT);
			armPID.disable();
			// Sets next state to teleop
			nextState = ArmState.kTeleop;
			m_stateVal = ArmState.kTeleop;
			break;
		case kTeleop:
			System.out.println("teleoping");

			// *** TOGGLE ZONE ***//

			// If armManualDesired, Toggle the arm override
			toggleArmManual = humanControl.toggleManualArmDesired();
			//Control wheel speed
			intakeWheels();
			//Push ball into wheel on trigger press
			if(humanControl.punchBall()) 
				punchCargo();
			 else 
				shooterPiston.set(false);
			//Push possible hatch mechanism off
			if(humanControl.pushHatch()) 
				outtakePiston.set(true);
			else 
				outtakePiston.set(false);

			SmartDashboard.putData(wristPotentiometer);
			// Arm Behavior. If manual, control arm with joystick, else control with PID
			if (toggleArmManual) {
				// Disable PID if in manual
				if (armPID.isEnabled()) {
					armPID.disable();
				}
				// Move arm based off of the Right Y of Operator Joystick
				armMotors.set((humanControl.getJoystickValue(Joysticks.kOperatorJoy, RemoteControl.Axes.kRY)));

				
			} else {

				moveArmPID();

			}

		}
	}

	public void punchCargo() {
		shooterPiston.set(true);
	}
	/*** Set PID Setpoints for arm based on human input. NEW Different buttons for moving arm than spinning wheels, with the exception of intake*/
	public void moveArmPID() {

		if (humanControl.intakeCargo()) {
			startArmPID(Params.ARM_INTAKE_CARGO_SETPOINT);
		} else if (humanControl.aimCargoBay()) {
			startArmPID(Params.ARM_LOW_CARGO_SETPOINT);
		} else if(humanControl.aimBackwardsCargoBay()) {
			startArmPID(Params.ARM_BACK_CARGO_SETPOINT);
		} else if (humanControl.customWristPosition()) {
			startArmPID(Params.ARM_MISC_CARGO_SETPOINT);
		} else if (humanControl.aimRocketMid()) {
			startArmPID(Params.ARM_HIGH_CARGO_SETPOINT);
		} else {
			startArmPID(Params.ARM_STOW_SETPOINT);
		}

		

	}

	public void startArmPID(double setpoint) {
		armPID.setPID(Params.arm_p, Params.arm_i, Params.arm_d, feedforward);
		armPID.setOutputRange(-Params.MAX_ARM_PID_OUT, Params.MAX_ARM_PID_OUT);
		armPID.setAbsoluteTolerance(3);
		armPID.setSetpoint(setpoint);
		armPID.enable();
	}
	/***Changes wheel speed based on human input */
	public void intakeWheels() {
		if(humanControl.outtakeCargo()) {
			intakeMotors.set(0.3);
		} else if(humanControl.shootMidCargo()) {
			intakeMotors.set(0.45);
		} else if(humanControl.shootHighCargo()) {
			intakeMotors.set(1);
		} else if(humanControl.intakeCargo()) {
			intakeMotors.set(-.35);
		} else {
			intakeMotors.set(0);

		}
	}
	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub

	}

}
