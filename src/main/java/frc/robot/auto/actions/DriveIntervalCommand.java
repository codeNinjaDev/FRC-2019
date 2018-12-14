/**
 * 
 */
package frc.robot.auto.actions;

import frc.robot.controllers.DriveController;
import frc.robot.feed.DataWriter;
import frc.robot.hardware.RobotModel;
import frc.robot.MasterController;

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
	private RobotModel robot;
	
	double goal_time, x_drive, y_drive, start_time;
	DataWriter<double[]> positionVsTimeCSV;
	public DriveIntervalCommand(MasterController controllers, double seconds, double y, double x) {
		
		requires(controllers.getDriveController());
		requires(controllers.getRobotModel());
		
		goal_time = seconds;
		x_drive = x;
		y_drive = y;
		this.kDrive = controllers.getDriveController();
		this.robot = controllers.getRobotModel();
		
		positionVsTimeCSV = new DataWriter<double[]>("/home/lvuser/PositionTime.csv", double[].class);
		System.out.println("Action Drive ");
	}
	
	public boolean isFinished() {
		return (Timer.getFPGATimestamp() >= start_time + goal_time);
	}

	@Override
	public void execute() {
		SmartDashboard.putNumber("reachedUPDATE", Timer.getFPGATimestamp());
		kDrive.arcadeDrive(y_drive, x_drive, false);
		double[] currentPos = {robot.getLeftDriveEncoderDistance(), robot.getLogTimer()};
		positionVsTimeCSV.add(currentPos);
		System.out.println("UPDATING");
		SmartDashboard.putString("AUTON", "UPDATING");
	}

	@Override
	public void end() {
		kDrive.stop();
		double[] finalPos = {robot.getLeftDriveEncoderDistance(), robot.getLogTimer()};
		positionVsTimeCSV.add(finalPos);
		positionVsTimeCSV.flush();
		robot.stopLogTimer();
	}

	@Override
	public void initialize() {
		robot.resetEncoders();
		robot.startLogTimer();
		double[] startPos = {robot.getLeftDriveEncoderDistance(), robot.getLogTimer()};
		positionVsTimeCSV.add(startPos);
		SmartDashboard.putNumber("reachedSTART", Timer.getFPGATimestamp());
		start_time = Timer.getFPGATimestamp();
	}
	
	public void interrupt() {
		end();
	}
	
	protected void interrupted() {
		end();
	}
}
