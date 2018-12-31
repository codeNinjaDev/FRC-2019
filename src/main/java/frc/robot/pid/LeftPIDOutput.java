package frc.robot.pid;



import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.PIDOutput;

/*** PID Output for Driving left side ***/
public class LeftPIDOutput implements PIDOutput {
	private SpeedControllerGroup driveSide;
	/*** Averages encoder values and returns them to PID Controller ***/
	public LeftPIDOutput(SpeedControllerGroup leftDriveside) {
		this.driveSide = leftDriveside;
	}
	@Override
	public void pidWrite(double output) {
		driveSide.set(output);
	}
	
	
}
