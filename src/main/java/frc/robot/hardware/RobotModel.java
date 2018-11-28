package frc.robot.hardware;

import frc.robot.Params;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.can.CANStatus;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

/*** Hardware (motors, pneumatics, sensors) for robot ***/
public class RobotModel extends Subsystem {
	/*** Drive Motors ***/
	private VictorSP leftDriveMotorA, leftDriveMotorB, rightDriveMotorA, rightDriveMotorB;
	/*** Motor Groups for Drive ***/
	private SpeedControllerGroup leftDriveMotors, rightDriveMotors;
	/*** Drive Encoder ***/
	private Encoder leftDriveEncoder, rightDriveEncoder;
	/*** Built-in accelerometer */
	private Accelerometer accel;

	/*** Timers for robot status ***/
	private Timer timer, logTimer, teleopTimer;
	/*** Power Distribution Panel (Gives electricity to all motors) ***/
	private PowerDistributionPanel pdp;

	CANStatus canStatus;


	/*** Initalizes all hardware ***/
	public RobotModel() {
		pdp = new PowerDistributionPanel();
		
		// Init drive motors
		leftDriveMotorA = new VictorSP(Ports.LEFT_DRIVE_MOTOR_A_PWM_PORT);
		leftDriveMotorB = new VictorSP(Ports.LEFT_DRIVE_MOTOR_B_PWM_PORT);
		rightDriveMotorA = new VictorSP(Ports.RIGHT_DRIVE_MOTOR_A_PWM_PORT);
		rightDriveMotorB = new VictorSP(Ports.RIGHT_DRIVE_MOTOR_B_PWM_PORT);
		// Make a Speed Controller group for Drive
		leftDriveMotors = new SpeedControllerGroup(leftDriveMotorA, leftDriveMotorB);
		rightDriveMotors = new SpeedControllerGroup(rightDriveMotorA, rightDriveMotorB);
		rightDriveMotors.setInverted(true); // negative value since wheels are inverted on one side
		
		// Initialize drive encoders
		leftDriveEncoder = new Encoder(Ports.LEFT_DRIVE_ENCODER_PORTS[0], Ports.LEFT_DRIVE_ENCODER_PORTS[1]);
		rightDriveEncoder = new Encoder(Ports.RIGHT_DRIVE_ENCODER_PORTS[0], Ports.RIGHT_DRIVE_ENCODER_PORTS[1]);

		// Encoder setup
		leftDriveEncoder.setReverseDirection(false);
		leftDriveEncoder.setDistancePerPulse(((1.0) / Params.PULSES_PER_ROTATION) * (Params.WHEEL_CIRCUMFERENCE));
		leftDriveEncoder.setSamplesToAverage(1);
		rightDriveEncoder.setReverseDirection(false);
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

		accel = new BuiltInAccelerometer();
		timer = new Timer();
		timer.start();

		logTimer = new Timer();

	}

	public enum Wheels {
		LeftWheels, RightWheels, AllWheels
	};

	/*** sets the speed for a given wheel(s) ***/
	public void setWheelSpeed(Wheels w, double speed) {
		switch (w) {
		case LeftWheels:
			leftDriveMotorA.set(speed);
			leftDriveMotorB.set(speed);
			break;
		case RightWheels:
			rightDriveMotorA.set(speed);

			rightDriveMotorB.set(speed);

			break;
		case AllWheels:
			leftDriveMotorA.set(speed);
			leftDriveMotorB.set(speed);
			rightDriveMotorA.set(speed);
			rightDriveMotorB.set(speed);
			break;
		}
	}

	/*** returns the speed of a given wheel ***/
	public double getWheelSpeed(Wheels w) {
		switch (w) {
		case LeftWheels:
			return leftDriveMotorA.get();
		case RightWheels:
			return rightDriveMotorA.get();
		default:
			return 0;
		}
	}

	/*** resets variables and objects ***/
	public void reset() {
		resetEncoders();
	}

	/*** returns the voltage from PDP ***/
	public double getVoltage() {
		return pdp.getVoltage();
	}

	/*** returns the total energy of the PDP ***/
	public double getTotalEnergy() {
		return pdp.getTotalEnergy();
	}

	/*** returns the total current of the PDP ***/
	public double getTotalCurrent() {
		return pdp.getTotalCurrent();
	}

	/*** returns the total power of the PDP ***/
	public double getTotalPower() {
		return pdp.getTotalPower();
	}

	/*** resets the timer ***/
	public void resetTimer() {
		timer.reset();
	}

	/*** Gets current system clock ***/
	@SuppressWarnings("static-access")
	public double getTimestamp() {
		return timer.getFPGATimestamp();
	}

	/*** returns the time ***/
	public double getTime() {
		return timer.get();
	}

	/*** Resets encoders ***/
	public void resetEncoders() {
		leftDriveEncoder.reset();
		rightDriveEncoder.reset();

	}

	/*** Returns difference between left and right encoder ***/
	public double getEncoderError() {
		return leftDriveEncoder.getDistance() - rightDriveEncoder.getDistance();
	}

	/*** Sets Left Drive output ***/
	public void setLeftMotors(double output) {
		leftDriveMotors.set(output);

	}

	/*** Sets Right Drive output ***/
	public void setRightMotors(double output) {
		rightDriveMotors.set(output);
	}

	/** Gets the horizontal acceleration */
	public double getAccelX() {
		return accel.getX();
	}
	
	/** Gets the forward acceleration */
	public double getAccelY() {
		return accel.getY();
	}
	
	/** Gets the vertical acceleration */
	public double getAccelZ() {
		return accel.getZ();
	}

	/*** Gets raw pulses of right drive encoder */
	public int getRightDriveEncoderRaw() {
		return rightDriveEncoder.get();
	}

	/*** Gets raw pulses of left drive encoder */
	public int getLeftDriveEncoderRaw() {
		return leftDriveEncoder.get();
	}

	/*** Gets distance (in) of left drive encoder */
	public double getLeftDriveEncoderDistance() {
		return leftDriveEncoder.getDistance();
	}

	/*** Gets distance (in) of right drive encoder */
	public double getRightDriveEncoderDistance() {
		return rightDriveEncoder.getDistance();
	}
	
	/*** Gets speed (in/s) of left drive encoder */
	public double getLeftDriveEncoderVelocity() {
		return leftDriveEncoder.getRate();
	}

	/*** Gets speed (in/s) of right drive encoder */
	public double getRightDriveEncoderVelocity() {
		return rightDriveEncoder.getRate();
	}

	/**Set the PID source type of the drive encoders */
	public void setDriveEncoderPIDSourceType(PIDSourceType pidSource) {
		leftDriveEncoder.setPIDSourceType(pidSource);
		rightDriveEncoder.setPIDSourceType(pidSource);
	}

	/*** Returns the left drive encoder object */
	public Encoder getLeftDriveEncoder() {
		return leftDriveEncoder;
	}

	/*** Returns the right drive encoder object */
	public Encoder getRightDriveEncoder() {
		return rightDriveEncoder;
	}

	/*** Returns left drive motor speed controller group */
	public SpeedControllerGroup getLeftDriveMotors() {
		return leftDriveMotors;
	}

	/*** Returns right drive motor speed controller group */
	public SpeedControllerGroup getRightDriveMotors() {
		return rightDriveMotors;
	}

	public void startLogTimer() {
		logTimer.start();
	}

	public double getLogTimer() {
		return logTimer.get();
	}

	public void stopLogTimer() {
		logTimer.stop();
	}

	/*** Runs in loop ***/
	public void update() {
	}

	@Override
	protected void initDefaultCommand() {
		
	}

}
