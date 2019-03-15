package frc.robot.pid;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
/*** Controls Drive Rotation ***/
public class DriveRotateMotorsPIDOutput implements PIDOutput{
	private DifferentialDrive drive;
	private double pidOutput;
	private boolean act;
	public DriveRotateMotorsPIDOutput(DifferentialDrive drive, boolean act) {
		this.drive = drive;
		this.act = act;
		this.pidOutput = 0;
	}
	@Override
	public void pidWrite(double output) {
		if(act){
			drive.arcadeDrive(0, output);
		} else {
			pidOutput = output;
		}
	}

	public double getPIDOutput() {
		return pidOutput;
	}
	
	
}
