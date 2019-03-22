package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Params;
import frc.robot.controllers.DriveController;
/*** Rotate to a specified angle with encoders as sensor ***/
public class GyroRotateCommand extends Command{
	private DriveController driveTrain;

	/*** Max time for action to complete ***/
	private double timeout, start_time;
	/*** Max speed action can run ***/
	private double maxSpeed;
	//P I D constants
	/** PID coefficients 
	 */
	private double P, I, D;
	private double angle;
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
	public GyroRotateCommand(DriveController driveController, double angle, double timeout, boolean waitForTimeout) {
		
		requires(driveController);
		this.angle = angle;
		this.driveTrain = driveController;
		this.timeout = timeout;
		this.waitForTimeout = waitForTimeout;
		
		reachedSetpoint = false;

		
		P = Params.new_drive_p;
		I = Params.new_drive_i;
		D =  Params.new_drive_d;
		
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
        P = Params.new_drive_p;
		I = Params.new_drive_i;
		D =  Params.new_drive_d;
		if(driveTrain.gyroPID.onTarget()) {
			reachedSetpoint = true;
		} else {
			reachedSetpoint = false;
		}
	}

	@Override
	public void end() {
		driveTrain.gyroPID.disable();
		driveTrain.stop();
	}

	public void interrupt() {
		end();
	}
	
	@Override
	public void start() {
		start_time = Timer.getFPGATimestamp();
		
		driveTrain.resetEncoders();
		
		driveTrain.gyro.reset();
        driveTrain.gyroPID.setPID(P, I, D);
        driveTrain.gyroPID.setSetpoint(angle);


		driveTrain.gyroPID.enable();
	}
	
	protected void interrupted() {
		end();
	}

}
