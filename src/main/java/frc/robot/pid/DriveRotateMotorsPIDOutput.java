package frc.robot.pid;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
/*** Controls Drive Rotation ***/
public class DriveRotateMotorsPIDOutput implements PIDOutput{
	private DifferentialDrive drive;
	private double pidOutput;
	public DriveRotateMotorsPIDOutput(DifferentialDrive drive) {
		this.drive = drive;
		this.pidOutput = 0;
	}
	@Override
	public void pidWrite(double output) {
		pidOutput = output;
	}

	public double getPIDOutput() {
		return pidOutput;
	}
	
	
}
