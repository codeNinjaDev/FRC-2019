package frc.robot.pid;



import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.PIDOutput;

/*** PID Output for Driving left side ***/
public class RightPIDOutput implements PIDOutput {
	private SpeedControllerGroup driveSide;
	/*** Averages encoder values and returns them to PID Controller ***/
	public RightPIDOutput(SpeedControllerGroup rightDriveside) {
		this.driveSide = rightDriveside;
	}
	@Override
	public void pidWrite(double output) {
		if(driveSide.getInverted())
			driveSide.set(output);
		else
			driveSide.set(-output);
	}
	
	
}
