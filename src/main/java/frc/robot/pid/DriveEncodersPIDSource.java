package frc.robot.pid;

import frc.robot.hardware.RobotModel;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
/*** PID Source for Driving with Encoders ***/
public class DriveEncodersPIDSource implements PIDSource {
	private RobotModel robot;
	/*** Averages encoder values and returns them to PID Controller ***/
	public DriveEncodersPIDSource(RobotModel robot) {
		this.robot = robot;
	}
	
	@Override
	public PIDSourceType getPIDSourceType() {
		return robot.getLeftDriveEncoder().getPIDSourceType();
		//TODO Add right encoder somehow
	}
	@Override
	public double pidGet() {
		switch (getPIDSourceType()) {
	    case kDisplacement:
	      return getAverageDistance();
	    case kRate:
	      return getAverageRate();
	    default:
	      return 0.0;
	  }
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		robot.getLeftDriveEncoder().setPIDSourceType(pidSource);
		robot.getRightDriveEncoder().setPIDSourceType(pidSource);		
	}
	
	public double getAverageDistance() {
		return ((robot.getLeftDriveEncoderDistance()) + (robot.getRightDriveEncoderDistance())) / 2.0;	
	}
	public double getAverageRate() {
		return ((robot.getLeftDriveEncoderVelocity()) + (robot.getRightDriveEncoderVelocity())) / 2.0;	
	}

}
