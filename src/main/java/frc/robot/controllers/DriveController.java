package frc.robot.controllers;

import frc.robot.Params;
import frc.robot.hardware.RemoteControl;
import frc.robot.hardware.RobotModel;
import frc.robot.pid.ArcadeStraightPIDOutput;
import frc.robot.pid.DriveEncodersPIDSource;
import frc.robot.pid.WheelsPIDOutput;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * TODO make this comment better: Handles both teleoperated and autonomus
 * driving
 * 
 * @category controllers
 * @author Peter I. Chacko, Eric Warner, John Sullivan Jr, Jake Boothby, Elliot
 *         Friedlander
 * 
 **/
public class DriveController extends Subsystem {

	private RobotModel robot;
	// Handles the math for arcade, curvature, and tank drive
	private DifferentialDrive drive;
	private RemoteControl humanControl;
	/** Current drive state **/
	private DriveState m_stateVal;
	/** Next drive state **/
	private DriveState nextState;
	/**
	 * Left PID Output <i>Uses {@link WheelsPIDOutput}</i>
	 **/
	private PIDOutput leftPIDOutput;

	private PIDController leftPID;
	/**
	 * Left Right PID Output <i>Uses {@link WheelsPIDOutput}</i>
	 **/
	private PIDOutput rightPIDOutput;
	private PIDController rightPID;


	/**
	 * Staight PID Output <i>Uses {@link ArcadeStraightPIDOutput}</i>
	 **/
	private PIDOutput straightPIDOutput;
	private PIDController straightPID;
	/**
	 * Averages Encoder values PID Source <i>Uses {@link DriveEncodersPIDSource}</i>
	 **/
	private PIDSource avgEncodersPIDSource;

	/**
	 * Different types of drive state
	 * 
	 * @param kInitialize
	 *            Initial DriveState
	 * @param kTeleopDrive
	 *            Teleop DriveState
	 **/
	enum DriveState {
		kInitialize, kTeleopDrive
	};
	private final double adjustedGlobalMaxSpeed;

	/**
	 * Initalizes all drive variables
	 * 
	 * @param robot
	 *            Robot Model to get encoders and motors
	 * @param humanControl
	 *            Get inputs from controllers
	 **/
	public DriveController(RobotModel robot, RemoteControl humanControl) {
		//Initialize dependencies
		this.robot = robot;
		this.humanControl = humanControl;
		//Create a differential drive that allows us to easily control robot
		drive = new DifferentialDrive(this.robot.getLeftDriveMotors(), this.robot.getRightDriveMotors());
		drive.setSafetyEnabled(false);
		
		/* Left and right PID Output */

		leftPIDOutput = new WheelsPIDOutput(RobotModel.Wheels.LeftWheels, this.robot);
		leftPID = new PIDController(Params.drive_p, Params.drive_i, Params.drive_d, this.robot.getLeftDriveEncoder(),
				leftPIDOutput);
		// TODO Might change this to max power variable
		leftPID.setOutputRange(-1.0, 1.0);
		leftPID.setAbsoluteTolerance(0.25);
		leftPID.disable();

		rightPIDOutput = new WheelsPIDOutput(RobotModel.Wheels.RightWheels, this.robot);

		rightPID = new PIDController(Params.drive_p, Params.drive_i, Params.drive_d, robot.getLeftDriveEncoder(),
				rightPIDOutput);
		// TODO Might change this to max power variable
		rightPID.setOutputRange(-1.0, 1.0);
		rightPID.setAbsoluteTolerance(0.25);
		rightPID.disable();

		avgEncodersPIDSource = new DriveEncodersPIDSource(this.robot);

		straightPIDOutput = new ArcadeStraightPIDOutput(drive, this.robot);
		straightPID = new PIDController(Params.drive_p, Params.drive_i, Params.drive_d, avgEncodersPIDSource,
				straightPIDOutput);
		// TODO might change this to max power variable
		straightPID.setOutputRange(-1.0, 1.0);
		straightPID.setAbsoluteTolerance(1);
		straightPID.disable();

		m_stateVal = DriveState.kInitialize;
		nextState = DriveState.kInitialize;

		adjustedGlobalMaxSpeed = Math.pow(Params.MAX_SPEED, .5); 
	}


	/**
	 * Updates inputs and drive in teleopPeriodic
	 * 
	 * @param currTimeSec
	 *            Does absolutely nothing
	 * @param deltaTimeSec
	 *            Does absolutely nothing
	 **/
	public void update() {
		switch (m_stateVal) {
		case kInitialize:
			leftPID.disable();
			rightPID.disable();
			nextState = DriveState.kTeleopDrive;
			break;
		case kTeleopDrive:
			double driverLeftX = humanControl.getJoystickValue(RemoteControl.Joysticks.kDriverJoy,
					RemoteControl.Axes.kLX);
			double driverLeftY = humanControl.getJoystickValue(RemoteControl.Joysticks.kDriverJoy,
					RemoteControl.Axes.kLY);
			double driverRightX = humanControl.getJoystickValue(RemoteControl.Joysticks.kDriverJoy,
					RemoteControl.Axes.kRX);
			double driverRightY = humanControl.getJoystickValue(RemoteControl.Joysticks.kDriverJoy,
					RemoteControl.Axes.kRY);

			if (leftPID.isEnabled() || rightPID.isEnabled()) {
				leftPID.disable();
				rightPID.disable();
			}

			arcadeDrive(driverLeftY, driverRightX, true);

			nextState = DriveState.kTeleopDrive;
			break;
		}
		m_stateVal = nextState;
	}

	/**
	 * Arcade Drive function, adds brake functionality
	 * 
	 * @param myY
	 *            moveValue
	 * @param myX
	 *            rotateValue
	 * @param teleOp
	 *            boolean inTeleop?
	 **/
	public void arcadeDrive(double myY, double myX, boolean teleOp) {
		if (teleOp) {
			// Brake 1
			if ((humanControl.getSlowDriveTier1Desired() && !humanControl.getSlowDriveTier2Desired())
					|| (!humanControl.getSlowDriveTier1Desired() && humanControl.getSlowDriveTier2Desired())) {
				Params.GLOBAL_Y_DRIVE_SPEED_MULTIPLIER = 0.35;
				Params.GLOBAL_X_DRIVE_SPEED_MULTIPLIER = 0.35;
				Params.SQUARE_DRIVE_AXIS_INPUT = false;
				// Brake 2
			} else if ((humanControl.getSlowDriveTier1Desired() && humanControl.getSlowDriveTier2Desired())) {
				Params.GLOBAL_Y_DRIVE_SPEED_MULTIPLIER = 0.2;
				Params.GLOBAL_X_DRIVE_SPEED_MULTIPLIER = 0.2;
				Params.SQUARE_DRIVE_AXIS_INPUT = false;
			} else {
				Params.GLOBAL_Y_DRIVE_SPEED_MULTIPLIER = 1.0;
				Params.GLOBAL_X_DRIVE_SPEED_MULTIPLIER = 1.0;
				Params.SQUARE_DRIVE_AXIS_INPUT = true;
			}
			/*** Adjusted because it has to offset the squared inputs */
			// Speed * BrakeSpeed * Max Speed from prefs
			drive.arcadeDrive(myY * Params.GLOBAL_Y_DRIVE_SPEED_MULTIPLIER * adjustedGlobalMaxSpeed,
					myX * Params.GLOBAL_X_DRIVE_SPEED_MULTIPLIER * adjustedGlobalMaxSpeed,
					Params.SQUARE_DRIVE_AXIS_INPUT);
			SmartDashboard.putNumber("Prefs MAXSPEED", Params.MAX_SPEED);

		} else {
			drive.arcadeDrive(myY * Params.MAX_SPEED, myX * Params.MAX_SPEED, false);
		}
	}

	/** Tank Drive Function with multipliers **/
	public void tankDrive(double myLeft, double myRight) {
		if ((humanControl.getSlowDriveTier1Desired() && !humanControl.getSlowDriveTier2Desired())
				|| (!humanControl.getSlowDriveTier1Desired() && humanControl.getSlowDriveTier2Desired())) {
			Params.GLOBAL_Y_DRIVE_SPEED_MULTIPLIER = 0.65;
			Params.GLOBAL_X_DRIVE_SPEED_MULTIPLIER = 0.65;
			Params.SQUARE_DRIVE_AXIS_INPUT = false;
		} else if ((humanControl.getSlowDriveTier1Desired() && humanControl.getSlowDriveTier2Desired())) {
			Params.GLOBAL_Y_DRIVE_SPEED_MULTIPLIER = 0.35;
			Params.GLOBAL_X_DRIVE_SPEED_MULTIPLIER = 0.35;
			Params.SQUARE_DRIVE_AXIS_INPUT = false;
		} else {
			Params.GLOBAL_Y_DRIVE_SPEED_MULTIPLIER = 1.0;
			Params.GLOBAL_X_DRIVE_SPEED_MULTIPLIER = 1.0;
			Params.SQUARE_DRIVE_AXIS_INPUT = true;
		}
		drive.tankDrive(myLeft * Params.GLOBAL_Y_DRIVE_SPEED_MULTIPLIER,
				myRight * Params.GLOBAL_Y_DRIVE_SPEED_MULTIPLIER, Params.SQUARE_DRIVE_AXIS_INPUT);
	}

	/** Changes current DriveState to initalize **/
	public void reset() {
		m_stateVal = DriveState.kInitialize;
	}

	/** Stops driveTrain **/
	public void stop() {
		drive.arcadeDrive(0, 0, false);
	}

	/* LEFT PID CONTROLLER */


	/***
	 * Configures the left PID Controller
	 *
	 * @param maxOutput The power range allowed for the pid controller
	 * @param P The P coefficient of the PID Controller
	 * @param I The I coefficient of the PID Controller
	 * @param D The D coefficient of the PID Controller
	 * @param setpoint The target that the PID Controller should reach
	 */
	public void configureLeftPID(double maxOutput, double P, double I, double D, double setpoint) {
		leftPID.setOutputRange(-maxOutput, maxOutput);
		leftPID.setPID(P, I, D);
		leftPID.setSetpoint(setpoint);
	}

	/***
	 * Enables the PID Controller
	 */
	public void startLeftPID() {
		leftPID.enable();
	}

	/***
	 * Disables the PID Controller
	 */
	public void stopLeftPID() {
		leftPID.disable();
	}

	/***
	 * Returns whether the PID Controller has reached target setpoint
	 */
	public boolean leftPIDReachedTarget() {
		return leftPID.onTarget();
	}

	/* RIGHT PID CONTROLLER */
	/***
	 * Enables the PID Controller
	 */
	public void startRightPID() {
		rightPID.enable();
	}

	/***
	 * Disables the PID Controller
	 */
	public void stopRightPID() {
		rightPID.disable();
	}

	/***
	 * Returns whether the PID Controller has reached target setpoint
	 */
	public boolean rightPIDReachedTarget() {
		return rightPID.onTarget();
	}


	/***
	 * Configures the right PID Controller
	 *
	 * @param maxOutput The power range allowed for the pid controller
	 * @param P The P coefficient of the PID Controller
	 * @param I The I coefficient of the PID Controller
	 * @param D The D coefficient of the PID Controller
	 * @param setpoint The target that the PID Controller should reach
	 */
	public void configureRightPID(double maxOutput, double P, double I, double D, double setpoint) {
		rightPID.setOutputRange(-maxOutput, maxOutput);
		rightPID.setPID(P, I, D);
		rightPID.setSetpoint(setpoint);
	}

	/* STRAIGHT PID CONTROLLER */


	/***
	 * Configures the straight PID Controller
	 *
	 * @param maxOutput The power range allowed for the pid controller
	 * @param P The P coefficient of the PID Controller
	 * @param I The I coefficient of the PID Controller
	 * @param D The D coefficient of the PID Controller
	 * @param setpoint The target that the PID Controller should reach
	 */
	public void configureStraightPID(double maxOutput, double P, double I, double D, double setpoint) {
		straightPID.setOutputRange(-maxOutput, maxOutput);
		straightPID.setPID(P, I, D);
		straightPID.setSetpoint(setpoint);
	}

	/***
	 * Enables the Straight PID Controller
	 */
	public void startStraightPID() {
		straightPID.enable();
	}

	/***
	 * Disables the Straight PID Controller
	 */
	public void stopStraightPID() {
		straightPID.disable();
	}

	/***
	 * Returns whether the Straight PID Controller has reached target setpoint
	 */
	public boolean straightPIDReachedTarget() {
		return straightPID.onTarget();
	}

	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}
}
