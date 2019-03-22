package frc.robot.auto.actions;

import frc.robot.Params;
import frc.robot.controllers.DriveController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*** Drive Forward action that uses arcade drive **/
public class ArcadeStraightCommand extends Command {
	private DriveController driveTrain;
	/*** Instance variable for desired distance ***/
	private double distance;
	/*** Instance variable or allowed timeout ***/
	private double timeout, start_time;
	/*** Instance variable for max speed set ***/
	private double maxSpeed;
	/*** Instance variables for P,I,D of straight PID controller ***/
	private double P, I, D;
	/*** Residual distance of left and right encoder ***/
	private double leftEncoderStartDistance, rightEncoderStartDistance;
	// Not sure
	private double afterSetpointTime, timeAfterHit;
	/*** Boolean checks if reached distance ***/
	private boolean reachedSetpoint;
	// Not sure
	private int target_pass;

	/***
	 * <h1>ArcadeStraightAction Constructor</h1>
	 * 
	 * <h2>Activity:</h2>
	 * <ul>
	 * <li>Initializes DriveController and RobotModel object from
	 * MasterController</li>
	 * <li>Initializes distance, timeout, maxSpeed from Constructor params</li>
	 * 
	 * 
	 * 
	 * </ul>
	 * 
	 * @param controllers
	 *            Gets all robot systems
	 * @param distance
	 *            Gets desired distance setpoint
	 * @param maxSpeed
	 *            Sets max speed during action
	 * @param timeout
	 *            How much time allowed to reach setpoint
	 * @param timeAfterHit
	 *            How much cushion time after reaching setpoint before ending action
	 *            (Doesn't do anything currently)
	 * 
	 */
	public ArcadeStraightCommand(DriveController driveController, double distance, double maxSpeed, double timeout, double timeAfterHit) {
		requires(driveController);

		this.driveTrain = driveController;
		this.distance = distance;
		this.timeout = timeout;
		this.maxSpeed = maxSpeed;
		this.timeAfterHit = timeAfterHit;
		start_time = 0;
		reachedSetpoint = false;
		leftEncoderStartDistance = 0.0;
		rightEncoderStartDistance = 0.0;

		P = Params.drive_p;
		I = Params.drive_i;
		D = Params.drive_d;

		SmartDashboard.putNumber("DRIVE_PID_P", P);
		SmartDashboard.putNumber("DRIVE_PID_I", I);
		SmartDashboard.putNumber("DRIVE_PID_D", D);

	}

	protected void initialize() {
		// Starts the timer
		start_time = Timer.getFPGATimestamp();

		// Reset Encoders before startng
		driveTrain.resetEncoders();

		leftEncoderStartDistance = driveTrain.leftDriveEncoder.getDistance();
		rightEncoderStartDistance = driveTrain.leftDriveEncoder.getDistance();
		// Set Output range and PID Constants
		driveTrain.configureStraightPID(maxSpeed, P, I, D, distance);
		// Starts arcade PID staright
		driveTrain.startStraightPID();
	}

	protected void execute() {
		boolean reachedTimeout = Timer.getFPGATimestamp() >= start_time + timeout;

		if (driveTrain.straightPIDReachedTarget() || reachedTimeout) {
			reachedSetpoint = true;
		} else {
			reachedSetpoint = false;
		}
	}

	/*** Checks if timeout is finished or reached setpoint ***/
	@Override
	protected boolean isFinished() {
		return reachedSetpoint;
	}

	protected void end() {
		driveTrain.stopStraightPID();
	}

	protected void interrupted() {
		end();
	}
}
