package frc.robot.pid;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
/*** Drives Straight based on PIDOutput ***/
public class DriveYMotorsPIDOutput implements PIDOutput {

	private DifferentialDrive drive;
	public DriveYMotorsPIDOutput(DifferentialDrive drive) {
		this.drive = drive;
	}
	@Override
	public void pidWrite(double output) {
		drive.arcadeDrive(output, 0, false);
	}
	
}
