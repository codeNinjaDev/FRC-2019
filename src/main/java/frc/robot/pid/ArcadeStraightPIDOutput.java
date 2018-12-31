package frc.robot.pid;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;

/*** Controls how a PID controller outputs to the robot in arcade drive ***/
public class ArcadeStraightPIDOutput implements PIDOutput {
	private DifferentialDrive drive;
	private Encoder leftDriveEncoder, rightDriveEncoder;

	private double loopOutput;
	//P for rotation
	private double kPencoder; //0.625
	public ArcadeStraightPIDOutput(DifferentialDrive drive, Encoder leftDriveEncoder, Encoder rightDriveEncoder) {
		this.drive = drive;
		kPencoder = 0.625;
		leftDriveEncoder = this.leftDriveEncoder;
		rightDriveEncoder = this.rightDriveEncoder;
	}
	@Override
	public void pidWrite(double output) {
		loopOutput = output;
		//Drives straight based on output
		drive.arcadeDrive(output, getEncoderError() * kPencoder, false);
	    SmartDashboard.putNumber("ArcadeCORRECTION", getEncoderError() * kPencoder);
	    SmartDashboard.putNumber("auton_EncoderError", getEncoderError());
	}
	
	double getPIDLoopOutput() {
		return loopOutput;
	}

	private double getEncoderError() {
		return leftDriveEncoder.getDistance() - rightDriveEncoder.getDistance();
	}
}
