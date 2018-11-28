package frc.robot.pid;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
/*** Controls Drive Rotation ***/
public class DriveRotateMotorsPIDOutput implements PIDOutput{
	private DifferentialDrive drive;
	public DriveRotateMotorsPIDOutput(DifferentialDrive drive) {
		this.drive = drive;
	}
	@Override
	public void pidWrite(double output) {
		drive.arcadeDrive(0.0, output, false);
	}
	
	
}
