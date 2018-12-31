/**
 * 
 */
package frc.robot.auto.actions;

import frc.robot.controllers.DriveController;
import frc.robot.feed.DataWriter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * @author peter
 *
 * Drives for a duration of time
 */
public class DriveIntervalCommand extends Command {
	private DriveController kDrive;
	
	double goal_time, x_drive, y_drive, start_time;
	DataWriter<double[]> positionVsTimeCSV;
	public DriveIntervalCommand(DriveController driveController, double seconds, double y, double x) {
		
		requires(driveController);
		
		goal_time = seconds;
		x_drive = x;
		y_drive = y;
		this.kDrive = driveController;
		
	}
	
	public boolean isFinished() {
		return (Timer.getFPGATimestamp() >= start_time + goal_time);
	}

	@Override
	public void execute() {
		SmartDashboard.putNumber("reachedUPDATE", Timer.getFPGATimestamp());
		kDrive.arcadeDrive(y_drive, x_drive, false);
		
	}

	@Override
	public void end() {
		kDrive.stop();
		
	}

	@Override
	public void initialize() {
		
	}
	
	public void interrupt() {
		end();
	}
	
	protected void interrupted() {
		end();
	}
}
