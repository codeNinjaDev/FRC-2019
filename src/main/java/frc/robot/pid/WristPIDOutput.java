package frc.robot.pid;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class WristPIDOutput implements PIDOutput{
	
	SpeedControllerGroup wristMotors;
	
	public WristPIDOutput(SpeedControllerGroup wristMotors) {
		this.wristMotors = wristMotors;
	}

	@Override
	public void pidWrite(double output) {
		wristMotors.set(output);
	}

}