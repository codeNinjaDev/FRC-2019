package frc.robot.controllers;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.pid.*;
import frc.robot.Params;
import frc.robot.hardware.*;
import frc.robot.hardware.RemoteControl.Joysticks;

/*** Controls mechanism for both cargo and hatch */
public class WristController extends Subsystem {
	private Spark leftIntakeMotor, rightIntakeMotor;
	private SpeedControllerGroup intakeMotors;

	private RemoteControl humanControl;

	private Spark armAMotor, armBMotor;
	private SpeedControllerGroup armMotors;

	private DoubleSolenoid outtakePistons, shooterPiston;
	private TenTurnPotentiometer wristPotentiometer;

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
		stow, intakeCargo, shootLowCargo, shootMidCargo, shootHighCargo, floorHatch, loadHatch, scoreHatch
	};

	public WristController(RemoteControl humanControl) {
		this.humanControl = humanControl;

		wristPotentiometer = new TenTurnPotentiometer(Ports.WRIST_POT);
		wristPotentiometer.setGearRatio(4);

		leftIntakeMotor = new Spark(Ports.LEFT_INTAKE_WHEEL_PORT);
		rightIntakeMotor = new Spark(Ports.RIGHT_INTAKE_WHEEL_PORT);
		rightIntakeMotor.setInverted(true);

		intakeMotors = new SpeedControllerGroup(leftIntakeMotor, rightIntakeMotor);
		intakeMotors.setInverted(false);

		armAMotor = new Spark(Ports.WRIST_ARM_A_PORT);
		armBMotor = new Spark(Ports.WRIST_ARM_B_PORT);

		armMotors = new SpeedControllerGroup(armAMotor, armBMotor);

		armMotors.setInverted(false);

		outtakePistons = new DoubleSolenoid(Ports.OUTTAKE_SOLENOIDS[0], Ports.OUTTAKE_SOLENOIDS[1]);
		shooterPiston = new DoubleSolenoid(Ports.SHOOTER_SOLENOIDS[0], Ports.SHOOTER_SOLENOIDS[1]);

		armPIDOutput = new WristPIDOutput(armMotors);
		armPID = new PIDController(0, 0, 0, wristPotentiometer, armPIDOutput);
		m_stateVal = ArmState.kInitialize;
		nextState = ArmState.kInitialize;

		desiredArmPosition = ArmPosition.stow;

	}

	public void stop() {
		armMotors.set(0);

	}

	public void reset() {
		m_stateVal = ArmState.kInitialize;
	}

	public void update() {
		// Switches based off of current state
		switch (m_stateVal) {
		// If initializing
		case kInitialize:

			// Intitalize Variables and PId
			armPID.setPID(Params.arm_p, Params.arm_i, Params.arm_d, Params.arm_f);
			armPID.setOutputRange(-1, 1);
			armPID.setSetpoint(Params.ARM_STOW_SETPOINT);

			// Sets next state to teleop
			nextState = ArmState.kTeleop;
			break;
		case kTeleop:
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
				armMotors.set((humanControl.getJoystickValue(Joysticks.kOperatorJoy, RemoteControl.Axes.kRY) * .75));
				// Intake normally
				
				if(humanControl.intakeCargo()) {
					intakeMotors.set(-1);
				} else if(humanControl.shootLowCargo()) {
					intakeMotors.set(0.4);
				} else if(humanControl.shootMidCargo()) {
					intakeMotors.set(0.75);
				} else if(humanControl.shootHighCargo()) {
					intakeMotors.set(1);
				}

				if(humanControl.outtakeHatch()) {
					outtakePistons.set(Value.kForward);
					outtakePistons.set(Value.kReverse);
				}
			} else {


				
				if(humanControl.outtakeHatch()) {
					outtakePistons.set(Value.kForward);
					outtakePistons.set(Value.kReverse);
				}
			}

			

		}
	}

	public void scoring() {
		if(humanControl.intakeCargo()) {
			desiredArmPosition = ArmPosition.intakeCargo;
			intakeMotors.set(-1);
			shooterPiston.set(Value.kReverse);
		} else if(humanControl.shootLowCargo()) {
			desiredArmPosition = ArmPosition.shootLowCargo;

		} else if(humanControl.shootMidCargo()) {

			desiredArmPosition = ArmPosition.shootMidCargo;
		} else if(humanControl.shootHighCargo()) {

			desiredArmPosition = ArmPosition.shootHighCargo;
		} else if(humanControl.floorHatch()) {
			shooterPiston.set(Value.kReverse);
			desiredArmPosition = ArmPosition.floorHatch;
		} else if(humanControl.loadHatch()) {
			shooterPiston.set(Value.kReverse);
			desiredArmPosition = ArmPosition.loadHatch;
		} else if(humanControl.scoreHatch()) {
			shooterPiston.set(Value.kReverse);
			desiredArmPosition = ArmPosition.scoreHatch;
		} else {
			shooterPiston.set(Value.kReverse);
			desiredArmPosition = ArmPosition.stow;
			intakeMotors.set(0);

		}

		switch (desiredArmPosition) {
			case stow:
				armPID.setPID(Params.arm_p, Params.arm_i, Params.arm_d, Params.arm_f);
				armPID.setOutputRange(-1, 1);
				armPID.setSetpoint(Params.ARM_STOW_SETPOINT);
				armPID.enable();

				break;
			case intakeCargo:
				armPID.setPID(Params.arm_p, Params.arm_i, Params.arm_d, Params.arm_f);
				armPID.setOutputRange(-1, 1);
				armPID.setSetpoint(Params.ARM_INTAKE_CARGO_SETPOINT);
				armPID.enable();


				break;
			case shootLowCargo:
				armPID.setPID(Params.arm_p, Params.arm_i, Params.arm_d, Params.arm_f);
				armPID.setOutputRange(-1, 1);
				armPID.setSetpoint(Params.ARM_LOW_CARGO_SETPOINT);
				armPID.enable();
				if(armPID.onTarget()) {
					shooterPiston.set(Value.kForward);
					intakeMotors.set(0.4);
				}
				break;
			case shootMidCargo:
				armPID.setPID(Params.arm_p, Params.arm_i, Params.arm_d, Params.arm_f);
				armPID.setOutputRange(-1, 1);
				armPID.setSetpoint(Params.ARM_MID_CARGO_SETPOINT);
				armPID.enable();
				if(armPID.onTarget()) {
					shooterPiston.set(Value.kForward);
					intakeMotors.set(0.75);
				}
				break;
			case shootHighCargo:
				armPID.setPID(Params.arm_p, Params.arm_i, Params.arm_d, Params.arm_f);
				armPID.setOutputRange(-1, 1);
				armPID.setSetpoint(Params.ARM_HIGH_CARGO_SETPOINT);
				armPID.enable();
				if(armPID.onTarget()) {
					shooterPiston.set(Value.kForward);
					intakeMotors.set(1);
				}
				break;
			case floorHatch:
				armPID.setPID(Params.arm_p, Params.arm_i, Params.arm_d, Params.arm_f);
				armPID.setOutputRange(-1, 1);
				armPID.setSetpoint(Params.ARM_FLOOR_HATCH_SETPOINT);
				armPID.enable();
				break;
			case scoreHatch:
				armPID.setPID(Params.arm_p, Params.arm_i, Params.arm_d, Params.arm_f);
				armPID.setOutputRange(-1, 1);
				armPID.setSetpoint(Params.ARM_SCORE_HATCH_SETPOINT);
				armPID.enable();
				break;
			case loadHatch:
				armPID.setPID(Params.arm_p, Params.arm_i, Params.arm_d, Params.arm_f);
				armPID.setOutputRange(-1, 1);
				armPID.setSetpoint(Params.ARM_LOAD_HATCH_SETPOINT);
				armPID.enable();
				break;
		}



	}

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub

	}

}
