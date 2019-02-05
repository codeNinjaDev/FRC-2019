package frc.robot.pid;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.hardware.*;

/*** Controls how a PID controller outputs to the robot in arcade drive ***/
public class ArcadeStraightPIDOutput implements PIDOutput {
	private DifferentialDrive drive;
	private SuperEncoder leftDriveEncoder, rightDriveEncoder;

	private double loopOutput;
	//P for rotation
	private double kPencoder; //0.625
	public ArcadeStraightPIDOutput(DifferentialDrive drive, SuperEncoder leftDriveEncoder, SuperEncoder rightDriveEncoder) {
		this.drive = drive;
		kPencoder = 0.625;
		this.leftDriveEncoder = leftDriveEncoder;
		this.rightDriveEncoder = rightDriveEncoder;
	}
	@Override
	public void pidWrite(double output) {
		loopOutput = output;
		//Drives straight based on output
		drive.arcadeDrive(output, (getEncoderError() * kPencoder), false);
	    
	}
	
	double getPIDLoopOutput() {
		return loopOutput;
	}

	private double getEncoderError() {
		return leftDriveEncoder.getDistance() - rightDriveEncoder.getDistance();
	}
}
