package frc.robot.auto.actions;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Params;
import frc.robot.controllers.DriveController;
import frc.robot.controllers.VisionController;
/*** Rotate to a specified angle with encoders as sensor ***/
public class VisionAlignCommand extends Command{
	private DriveController driveTrain;

	/*** Max time for action to complete ***/
	private double timeout, start_time;
	/*** Max speed action can run ***/
	private double maxSpeed;
	//P I D constants
	/** PID coefficients 
	 */
	private double P, I, D;
	/*** Checks if reached desired setpoint ***/
	private boolean reachedSetpoint;
	/*** If true, it waits until timeout is complete, even if PID has hit the setpoint ***/
	private boolean waitForTimeout;
	private VisionController vision;
	/***
	 * Converts angle to degrees and initalizes all variables
	 * @param controllers ALl classes that control robot functionality
	 * @param maxSpeed Max Speed of rotation (-1 to 1)
	 * @param timeout Max allowed time action runs for
	 * @param waitForTimeout Whether to wait the full timeout, even if the setpoint is reached
	 */ 
	public VisionAlignCommand(DriveController driveController, VisionController vision, double timeout, boolean waitForTimeout) {
		
		requires(driveController);
		this.driveTrain = driveController;
		this.timeout = timeout;
		this.waitForTimeout = waitForTimeout;
		this.vision = vision;
		reachedSetpoint = false;

		
		P = Params.vision_p;
		I = Params.vision_i;
		D =  Params.vision_d;
		
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
		P = Params.vision_p;
		I = Params.vision_i;
		D =  Params.vision_d;
		
		if(driveTrain.visionPID.onTarget()) {
			reachedSetpoint = true;
		} else {
			reachedSetpoint = false;
        }
        driveTrain.arcadeDrive(0, -driveTrain.visionPID.get(), false);
	}

	@Override
	public void end() {
		driveTrain.visionPID.disable();
		driveTrain.stop();
	}

	public void interrupt() {
		end();
	}
	
	@Override
	public void start() {
        if(!vision.tapeDetected.getBoolean(false)) {
            end();
        }
		start_time = Timer.getFPGATimestamp();
		
		driveTrain.resetEncoders();
		
        driveTrain.visionPID.setPID(P, I, D);
        driveTrain.visionPID.setSetpoint(0);


		driveTrain.visionPID.enable();
	}
	
	protected void interrupted() {
		end();
	}

}
