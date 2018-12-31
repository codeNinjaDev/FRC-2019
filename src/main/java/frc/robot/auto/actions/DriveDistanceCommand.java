package frc.robot.auto.actions;

import frc.robot.controllers.DriveController;
import frc.robot.Params;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*** Drives Left and Right Side PID independently but to a certain distance (in)***/
public class DriveDistanceCommand extends Command {

	private DriveController driveTrain;
	
	/*** Distance Setpoint ***/
	private double distance;
	/*** Max time for action to complete ***/
	private double timeout, start_time;
	/*** Max speed action can run ***/
	private double maxSpeed;
	//P I D constants
	/** PID coefficients 
	 */
	private double P, I, D;
	/*** The residual distance of encoders (error basically) ***/
	private double leftEncoderStartDistance, rightEncoderStartDistance;
	/*** Checks if reached desired setpoint ***/
	private boolean reachedSetpoint;
	/*** If true, it waits until timeout is complete, even if PID has hit the setpoint ***/
	private boolean waitForTimeout;
	/*** Drives Forward with independent left and right PID controllers
	 * 
	 * @param controllers ALl classes that control robot functionality
	 * @param distance Desired distance setpoint
	 * @param maxSpeed Max output of PID controller
	 * @param timeout Allowed time for action to take place
	 * @param waitForTimeout Whether to wait the full timeout, even if the setpoint is reached
	 */
	public DriveDistanceCommand(DriveController driveController,double distance, double maxSpeed, double timeout, boolean waitForTimeout) {
		requires(driveController);
		
		this.driveTrain = driveController;
		this.distance = distance;
		this.timeout = timeout;
		this.maxSpeed = maxSpeed;
		this.waitForTimeout = waitForTimeout;
		
		reachedSetpoint = false;
		leftEncoderStartDistance = 0.0; 
		rightEncoderStartDistance = 0.0;
		
		P = Params.drive_p;
		I = Params.drive_i;
		D = Params.drive_d;
		
		

	}
	@Override
	/*** Checks if action is finished ***/
	public boolean isFinished() {
		//Check if we want to only wait for Timeout
		if(waitForTimeout) 
			return (Timer.getFPGATimestamp() >= start_time + timeout);
		else //If not, end if we reached the setpoint
			return (Timer.getFPGATimestamp() >= start_time + timeout) || reachedSetpoint;
	}

	@Override
	/*** Updates action code in a loop ***/
	public void execute() {
		//Checks if pid controllers are reached setpoint
		if(driveTrain.leftPIDReachedTarget() && driveTrain.rightPIDReachedTarget()) {
			reachedSetpoint = true;
		} else {

			reachedSetpoint = false;
		}
	}

	@Override
	/*** Code that runs when action ends ***/
	public void end() {
		//Disables PID and stops drive
		driveTrain.stopLeftPID();
		driveTrain.stopRightPID();
		driveTrain.stop();
	}

	@Override
	/*** Starts action ***/
	protected void initialize() {
		//Starts Timer
		start_time = Timer.getFPGATimestamp();
		//Configures encoders to measuring magnitude, not rate
		//Resets encoders
		driveTrain.resetEncoders();
		
		leftEncoderStartDistance = driveTrain.leftDriveEncoder.getDistance();
		rightEncoderStartDistance = driveTrain.leftDriveEncoder.getDistance();
		
		//Sets PID Outputrange, constants, and setpoints
		
		driveTrain.configureLeftPID(maxSpeed, P, I, D, distance);
		
		driveTrain.configureRightPID(maxSpeed, P, I, D, distance);
		
		//Starts PID
		driveTrain.startLeftPID();
		driveTrain.startRightPID();
	}
	
	protected void interrupted() {
		end();
	}
}
