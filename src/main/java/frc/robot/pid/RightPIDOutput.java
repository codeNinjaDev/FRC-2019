package frc.robot.pid;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Spark;

/*** PID Output for Driving left side ***/
public class RightPIDOutput implements PIDOutput {
	private Spark driveSide;
	/*** Averages encoder values and returns them to PID Controller ***/
	public RightPIDOutput(Spark rightDriveside) {
		this.driveSide = rightDriveside;
	}
	@Override
	public void pidWrite(double output) {
		driveSide.set(output);

	}
	
	
}
