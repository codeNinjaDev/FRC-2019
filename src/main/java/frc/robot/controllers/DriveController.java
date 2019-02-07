package frc.robot.controllers;

import frc.robot.Params;
import frc.robot.pid.*;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.*;
import frc.robot.hardware.*;
import frc.robot.hardware.SuperEncoder;
import frc.robot.pid.*;
import frc.robot.controllers.*;
import com.kauailabs.navx.frc.AHRS;

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
	/*** Drive Motors ***/
	private VictorSP leftDriveMotorA, leftDriveMotorB, rightDriveMotorA, rightDriveMotorB;
	/*** Motor Groups for Drive ***/
	public SpeedControllerGroup leftDriveMotors, rightDriveMotors;
	/*** Drive Encoder ***/
	public SuperEncoder leftDriveEncoder, rightDriveEncoder;

	private SuperGyro gyro;
	// Handles the math for arcade, curvature, and tank drive
	private DifferentialDrive drive;
	private RemoteControl humanControl;
	/** Current drive state **/
	private DriveState m_stateVal;
	/** Next drive state **/
	private DriveState nextState;

	/**
	 * Staight PID Output <i>Uses {@link ArcadeStraightPIDOutput}</i>
	 **/
	private ArcadeStraightPIDOutput straightPIDOutput;
	private PIDController straightPID;
	/**
	 * Averages Encoder values PID Source <i>Uses {@link DriveEncodersPIDSource}</i>
	 **/
	private DriveEncodersPIDSource avgEncodersPIDSource;

	/*** Left PID Output */
	private LeftPIDOutput leftPIDOutput;
	private PIDController leftPID;

	private RightPIDOutput rightPIDOutput;
	private PIDController rightPID; 

	private VisionPIDSource visionSource;
	private DriveRotateMotorsPIDOutput visionOutput;
	private PIDController visionPID;

	private VisionController vision;

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
	public DriveController(RemoteControl humanControl, VisionController vision) {
		this.vision = vision;

		gyro = new SuperGyro(SPI.Port.kMXP);
		gyro.hardReset();
		// Init drive motors
		leftDriveMotorA = new VictorSP(Ports.LEFT_DRIVE_MOTOR_A_PWM_PORT);
		leftDriveMotorB = new VictorSP(Ports.LEFT_DRIVE_MOTOR_B_PWM_PORT);
		rightDriveMotorA = new VictorSP(Ports.RIGHT_DRIVE_MOTOR_A_PWM_PORT);
		rightDriveMotorB = new VictorSP(Ports.RIGHT_DRIVE_MOTOR_B_PWM_PORT);
		// Make a Speed Controller group for Drive
		leftDriveMotors = new SpeedControllerGroup(leftDriveMotorA, leftDriveMotorB);
		rightDriveMotors = new SpeedControllerGroup(rightDriveMotorA, rightDriveMotorB);
		
		// Initialize drive encoders
		leftDriveEncoder = new SuperEncoder(Ports.LEFT_DRIVE_ENCODER_PORTS[0], Ports.LEFT_DRIVE_ENCODER_PORTS[1]);
		rightDriveEncoder = new SuperEncoder(Ports.RIGHT_DRIVE_ENCODER_PORTS[0], Ports.RIGHT_DRIVE_ENCODER_PORTS[1]);

		// Encoder setup
		leftDriveEncoder.setReverseDirection(true);
		leftDriveEncoder.setDistancePerPulse(((1.0) / Params.PULSES_PER_ROTATION) * (Params.WHEEL_CIRCUMFERENCE));
		leftDriveEncoder.setSamplesToAverage(1);
		rightDriveEncoder.setReverseDirection(true);
		rightDriveEncoder.setDistancePerPulse(((1.0) / Params.PULSES_PER_ROTATION) * (Params.WHEEL_CIRCUMFERENCE));
		rightDriveEncoder.setSamplesToAverage(1);

		leftDriveMotorA.setSafetyEnabled(false);
		leftDriveMotorB.setSafetyEnabled(false);
		rightDriveMotorA.setSafetyEnabled(false);
		rightDriveMotorB.setSafetyEnabled(false);

		leftDriveEncoder.setPIDSourceType(PIDSourceType.kDisplacement);
		leftDriveEncoder.setSamplesToAverage(Params.DRIVE_Y_PID_SAMPLES_AVERAGE);
		rightDriveEncoder.setPIDSourceType(PIDSourceType.kDisplacement);
		rightDriveEncoder.setSamplesToAverage(Params.DRIVE_Y_PID_SAMPLES_AVERAGE);

		//Initialize dependencies
		this.humanControl = humanControl;
		//Create a differential drive that allows us to easily control robot
		drive = new DifferentialDrive(leftDriveMotors, rightDriveMotors);
		drive.setSafetyEnabled(false);
		
		/* Left and right PID Output */


		avgEncodersPIDSource = new DriveEncodersPIDSource(leftDriveEncoder, rightDriveEncoder);

		straightPIDOutput = new ArcadeStraightPIDOutput(drive, leftDriveEncoder, rightDriveEncoder);
		straightPID = new PIDController(Params.drive_p, Params.drive_i, Params.drive_d, avgEncodersPIDSource,
				straightPIDOutput);
		// TODO might change this to max power variable
		straightPID.setOutputRange(-1.0, 1.0);
		straightPID.setAbsoluteTolerance(1);
		straightPID.disable();

		leftPIDOutput = new LeftPIDOutput(leftDriveMotors);
		leftPID = new PIDController(Params.drive_p, Params.drive_i, Params.drive_d, leftDriveEncoder, leftPIDOutput);
	
		leftPID.setOutputRange(-1.0, 1.0);
		leftPID.setAbsoluteTolerance(1);
		leftPID.disable();

		rightPIDOutput = new RightPIDOutput(rightDriveMotors);
		rightPID = new PIDController(Params.drive_p, Params.drive_i, Params.drive_d, rightDriveEncoder, rightPIDOutput);

		rightPID.setOutputRange(-1.0, 1.0);
		rightPID.setAbsoluteTolerance(1);
		rightPID.disable();

		visionSource = new VisionPIDSource(vision, gyro);
		visionOutput = new DriveRotateMotorsPIDOutput(drive);
		visionPID = new PIDController(.011, 0.00012, 0.008, visionSource, visionOutput);

		visionPID.setOutputRange(-0.22, 0.22);
		visionPID.setAbsoluteTolerance(.25);
		visionPID.setSetpoint(0);

		visionPID.disable();



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

			/*if(humanControl.getOuttakeDesired()) {
				visionPID.setSetpoint(vision.targetYaw());
				gyro.reset();
			}*/
			SmartDashboard.putBoolean("GYRO?", gyro.isConnected());
			if(humanControl.getCargoVisionDesired()) {
				visionPID.setPID(.05, 0.0001, 0.08);
				visionPID.enable();
				SmartDashboard.putNumber("GYRO VISION ANGLE", gyro.getAngle());
				if(vision.targetYaw() != 0) {
					arcadeDrive(driverLeftY, -visionPID.get(), false);
				} else {
					arcadeDrive(driverLeftY,driverRightX, false);
				}
				
			} else if(humanControl.getTapeVisionDesired()) {
				visionPID.setPID(.05, 0.00016, 0.008);
				visionPID.enable();
				SmartDashboard.putNumber("GYRO VISION ANGLE", gyro.getAngle());
				if(vision.targetYaw() != 0) {
					arcadeDrive(driverLeftY, -visionPID.get(), false);
				} else {
					arcadeDrive(driverLeftY,driverRightX, false);
				}
				
			} else {
				visionPID.disable();
				arcadeDrive(driverLeftY, driverRightX, true);
			}

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

	public double getAverageTotalDistance() {
		return ((leftDriveEncoder.getTotalDistance() + rightDriveEncoder.getTotalDistance()) / 2);
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
		leftPID.setAbsoluteTolerance(1);
		leftPID.setOutputRange(-maxOutput, maxOutput);
		leftPID.setPID(P, I, D);
		leftPID.setSetpoint(setpoint);
	}

	/***
	 * Enables the Straight PID Controller
	 */
	public void startLeftPID() {
		leftPID.enable();
	}

	/***
	 * Disables the left PID Controller
	 */
	public void stopLeftPID() {
		leftPID.disable();
	}

	/***
	 * Returns whether the left PID Controller has reached target setpoint
	 */
	public boolean leftPIDReachedTarget() {
		return leftPID.onTarget();
	}

	/* RIGHT PID CONTROLLER */


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
		rightPID.setAbsoluteTolerance(1);
		rightPID.setOutputRange(-maxOutput, maxOutput);
		rightPID.setPID(P, I, D);
		rightPID.setSetpoint(setpoint);
	}

	/***
	 * Enables the right PID Controller
	 */
	public void startRightPID() {
		rightPID.enable();
	}

	/***
	 * Disables the right PID Controller
	 */
	public void stopRightPID() {
		rightPID.disable();
	}

	/***
	 * Returns whether the right PID Controller has reached target setpoint
	 */
	public boolean rightPIDReachedTarget() {
		return rightPID.onTarget();
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
		straightPID.setAbsoluteTolerance(1);
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

	/*** Resets encoders ***/
	public void resetEncoders() {
		leftDriveEncoder.reset();
		rightDriveEncoder.reset();

	}

	/*** Sets Left Drive output ***/
	public void setLeftMotors(double output) {
		leftDriveMotors.set(output);

	}

	/*** Sets Right Drive output ***/
	public void setRightMotors(double output) {
		rightDriveMotors.set(-output);
	}

	public double getGyroAngle() {
		return gyro.getAngle();
	}

	public double getTotalGyroAngle() {
		return gyro.getTotalAngle();
	}
	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}
}
