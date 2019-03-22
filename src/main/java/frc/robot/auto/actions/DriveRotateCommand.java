package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Params;
import frc.robot.controllers.DriveController;
/*** Rotate to a specified angle with encoders as sensor ***/
public class DriveRotateCommand extends Command{
	private DriveController driveTrain;
	/*** Distance Setpoint (Converted from angle in degrees to inches) ***/
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
	
	/***
	 * Converts angle to degrees and initalizes all variables
	 * @param controllers ALl classes that control robot functionality
	 * @param angle Angle Setpoint (e.g 90 degrees)
	 * @param maxSpeed Max Speed of rotation (-1 to 1)
	 * @param timeout Max allowed time action runs for
	 * @param waitForTimeout Whether to wait the full timeout, even if the setpoint is reached
	 */ 
	public DriveRotateCommand(DriveController driveController, double angle, double maxSpeed, double timeout, boolean waitForTimeout) {
		
		requires(driveController);
		
		this.driveTrain = driveController;
		//It takes 20 inches on the left side and -20 inches on the right side to turn 90 degrees
		this.distance = (angle * 20.0) / (90.0);
		this.timeout = timeout;
		this.maxSpeed = maxSpeed;
		this.waitForTimeout = waitForTimeout;
		
		reachedSetpoint = false;
		leftEncoderStartDistance = 0.0; 
		rightEncoderStartDistance = 0.0;
		
		P = Params.drive_p;
		I = Params.drive_i;
		D =  Params.drive_d;
		
		SmartDashboard.putNumber("DRIVE_PID_P", P);
		SmartDashboard.putNumber("DRIVE_PID_I", I);
		SmartDashboard.putNumber("DRIVE_PID_D", D);

	}
	@Override
	public boolean isFinished() {
		if((Timer.getFPGATimestamp() >= start_time + timeout) && !(reachedSetpoint)) {
			
		}
		if(waitForTimeout) 
			return (Timer.getFPGATimestamp() >= start_time + timeout);
		else
			return (Timer.getFPGATimestamp() >= start_time + timeout) || reachedSetpoint;
	}

	@Override
	public void execute() {
		if(driveTrain.leftPIDReachedTarget() && driveTrain.rightPIDReachedTarget()) {
			reachedSetpoint = true;
		} else {
			reachedSetpoint = false;
		}
	}

	@Override
	public void end() {
		driveTrain.stopLeftPID();
		driveTrain.stopRightPID();
		driveTrain.stop();
	}

	public void interrupt() {
		end();
	}
	
	@Override
	public void start() {
		start_time = Timer.getFPGATimestamp();
		
		driveTrain.resetEncoders();
		
		leftEncoderStartDistance = driveTrain.leftDriveEncoder.getDistance();
		rightEncoderStartDistance = driveTrain.leftDriveEncoder.getDistance();
		
		driveTrain.configureLeftPID(maxSpeed, P, I, D, (distance - leftEncoderStartDistance));
		
		driveTrain.configureRightPID(maxSpeed, P, I, D, -(distance - rightEncoderStartDistance));
		
		driveTrain.startLeftPID();
		driveTrain.startRightPID();
	}
	
	protected void interrupted() {
		end();
	}

}
