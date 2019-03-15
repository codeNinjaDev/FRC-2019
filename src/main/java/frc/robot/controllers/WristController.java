package frc.robot.controllers;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
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
import frc.robot.auto.actions.OuttakePistons;
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
	public TenTurnPotentiometer wristPotentiometer;

	private PIDOutput armPIDOutput;
	private PIDController armPID;

	/*** Current state of robot; e.g init or teleop ***/
	private ArmState m_stateVal;
	/*** Next state of robot; e.g init or teleop ***/
	private ArmState nextState;
	private ArmPosition desiredArmPosition;
	/*** Boolean for override arm control ***/
	private boolean toggleArmManual;

	/*** Enum that lets us shift our status from init to teleop and vice versa ***/
	public enum ArmState {
		kInitialize, kTeleop
	};

	public enum ArmPosition {
		stow, intakeCargo, shootLowCargo, shootMidCargo, shootHighCargo
	};
	// IS going down positive or negative?
	public WristController(RemoteControl humanControl) {
		this.humanControl = humanControl;

		wristPotentiometer = new TenTurnPotentiometer(Ports.WRIST_POT);
		wristPotentiometer.setGearRatio(210);
		leftIntakeMotor = new VictorSP(Ports.LEFT_INTAKE_WHEEL_PORT);
		rightIntakeMotor = new VictorSP(Ports.RIGHT_INTAKE_WHEEL_PORT);
		rightIntakeMotor.setInverted(true);

		intakeMotors = new SpeedControllerGroup(leftIntakeMotor, rightIntakeMotor);
		intakeMotors.setInverted(false);

		armMotorA = new VictorSP(Ports.WRIST_ARM_A_PORT);
		armMotorB = new VictorSP(Ports.WRIST_ARM_B_PORT);


		armMotorA.setInverted(true);
		armMotors = new SpeedControllerGroup(armMotorA, armMotorB);
		outtakePiston = new Solenoid(Ports.OUTTAKE_SOLENOIDS[0]);

		shooterPiston = new Solenoid(Ports.SHOOTER_SOLENOIDS[0]);

		armPIDOutput = new WristPIDOutput(armMotors);
		armPID = new PIDController(0, 0, 0, wristPotentiometer, armPIDOutput);
		m_stateVal = ArmState.kInitialize;
		nextState = ArmState.kInitialize;

		desiredArmPosition = ArmPosition.stow;

	}

	public void showAngle() {
		SmartDashboard.putNumber("TEN_TURN_POT", wristPotentiometer.getAngle());
	}

	public void stop() {
		armMotors.set(0);

	}

	public void reset() {
		m_stateVal = ArmState.kInitialize;
	}

	public void update() {
		System.out.println("running");

		// Switches based off of current state
		switch (m_stateVal) {
		// If initializing
		case kInitialize:
			System.out.println("init");

			// Intitalize Variables and PId
			armPID.setPID(Params.arm_p, Params.arm_i, Params.arm_d, Params.arm_f);
			armPID.setOutputRange(-1, 1);
			armPID.setSetpoint(Params.ARM_STOW_SETPOINT);

			// Sets next state to teleop
			nextState = ArmState.kTeleop;
			m_stateVal = ArmState.kTeleop;
			break;
		case kTeleop:
			System.out.println("teleoping");

			// *** TOGGLE ZONE ***//

			// If armManualDesired, Toggle the arm override
			toggleArmManual = humanControl.toggleManualArmDesired();

			// Arm Behavior
			if (toggleArmManual) {
				// Disable PID if in manual
				if (armPID.isEnabled()) {
					armPID.disable();
				}
				// Move arm based off of the Right Y of Operator Joystick
				armMotors.set((humanControl.getJoystickValue(Joysticks.kOperatorJoy, RemoteControl.Axes.kLY)));
				// Intake normally
				
				if(humanControl.shootLowCargo()) {
					shooterPiston.set(true);
					intakeMotors.set(0.75);
				} else if(humanControl.shootMidCargo()) {
					shooterPiston.set(true);
					intakeMotors.set(0.9);
				} else if(humanControl.shootHighCargo()) {
					shooterPiston.set(true);
					intakeMotors.set(1);
				} else if(humanControl.intakeCargo()) {
					shooterPiston.set(false);
					intakeMotors.set(-1);
				} else {
					shooterPiston.set(false);
				}

				if(humanControl.outtakePistons()) {
					System.out.println("OUTAKING");
										SmartDashboard.putString("Outtaking", "OUtaking");

					outtakePistons();

					
				} else {
					outtakePiston.set(false);
				}
			} else {

				try {
					scoring();
				} catch(Exception e) {
					System.out.println(e);
				}
				if(humanControl.outtakePistons()) {
					System.out.println("OUTAKING");
					SmartDashboard.putString("Outtaking", "OUtaking");
					outtakePistons();
					
				} else {
					outtakePiston.set(false);
				}
			}

		}
	}
	public void outtakePistons() {
		outtakePiston.set(true);
	}
	public void scoring() {

		if(humanControl.intakeCargo()) {
			desiredArmPosition = ArmPosition.intakeCargo;
			intakeMotors.set(-1);
			shooterPiston.set(false);
		} else if(humanControl.shootLowCargo()) {
			intakeMotors.set(0.75);
			desiredArmPosition = ArmPosition.shootLowCargo;

		} else if(humanControl.shootMidCargo()) {
			intakeMotors.set(0.9);
			desiredArmPosition = ArmPosition.shootMidCargo;
		} else if(humanControl.shootHighCargo()) {
			intakeMotors.set(1);
			desiredArmPosition = ArmPosition.shootHighCargo;
		} else {
			shooterPiston.set(false);
			desiredArmPosition = ArmPosition.stow;
			intakeMotors.set(0);
		}

		switch (desiredArmPosition) {
			case stow:
				startArmPID(Params.ARM_STOW_SETPOINT);
				break;
			case intakeCargo:
				startArmPID(Params.ARM_INTAKE_CARGO_SETPOINT);
				break;
			case shootLowCargo:
				startArmPID(Params.ARM_LOW_CARGO_SETPOINT);
				if(armPID.onTarget()) {
					shooterPiston.set(true);
				}
				break;
			case shootMidCargo:
				startArmPID(Params.ARM_MID_CARGO_SETPOINT);
				if(armPID.onTarget()) {
					shooterPiston.set(true);
				}
				break;
			case shootHighCargo:
				startArmPID(Params.ARM_HIGH_CARGO_SETPOINT);
				if(armPID.onTarget()) {
					shooterPiston.set(true);
				}
				break;
			default: 
				desiredArmPosition = ArmPosition.stow;
				break;

		}



	}

	public void startArmPID(double setpoint) {
		armPID.setPID(Params.arm_p, Params.arm_i, Params.arm_d, Params.arm_f);
		armPID.setOutputRange(-Params.MAX_ARM_PID_OUT, Params.MAX_ARM_PID_OUT);
		armPID.setSetpoint(setpoint);
		armPID.enable();
	}

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub

	}

}
