package frc.robot.pid;


import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Encoder;

/*** PID Source for Driving with Encoders ***/
public class DriveEncodersPIDSource implements PIDSource {
	private Encoder leftDriveEncoder, rightDriveEncoder;
	/*** Averages encoder values and returns them to PID Controller ***/
	public DriveEncodersPIDSource(Encoder leftDriveEncoder, Encoder rightDriveEncoder) {
		leftDriveEncoder = this.leftDriveEncoder;
		rightDriveEncoder = this.rightDriveEncoder;
	}
	
	@Override
	public PIDSourceType getPIDSourceType() {
		return leftDriveEncoder.getPIDSourceType();
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
		leftDriveEncoder.setPIDSourceType(pidSource);
		rightDriveEncoder.setPIDSourceType(pidSource);		
	}
	
	public double getAverageDistance() {
		return (leftDriveEncoder.getDistance() + rightDriveEncoder.getDistance()) / 2.0;	
	}
	public double getAverageRate() {
		return (leftDriveEncoder.getRate() + rightDriveEncoder.getRate()) / 2.0;	
	}

}
