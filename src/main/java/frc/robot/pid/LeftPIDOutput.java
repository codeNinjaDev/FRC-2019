package frc.robot.pid;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Spark;

/*** PID Output for Driving left side ***/
public class LeftPIDOutput implements PIDOutput {
	private Spark driveSide;
	/*** Averages encoder values and returns them to PID Controller ***/
	public LeftPIDOutput(Spark leftDriveside) {
		this.driveSide = leftDriveside;
	}
	@Override
	public void pidWrite(double output) {
		driveSide.set(output);
	}
	
	
}
