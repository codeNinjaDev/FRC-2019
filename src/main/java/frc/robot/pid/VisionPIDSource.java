package frc.robot.pid;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import frc.robot.controllers.VisionController;
/*
 * TODO Learn how to make camera a pid source
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;*/
import frc.robot.hardware.SuperGyro;

public class VisionPIDSource implements PIDSource {
	private VisionController vision;
	private SuperGyro gyro;
	public VisionPIDSource(VisionController vision, SuperGyro gyro) {
		this.gyro = gyro;
		this.vision = vision;
	}
	
	@Override
	public PIDSourceType getPIDSourceType() {
		return PIDSourceType.kDisplacement;
		//TODO Add right encoder somehow
	}
	@Override
	public double pidGet() {
		switch (getPIDSourceType()) {
	    case kDisplacement:
	      return vision.targetYaw();
	    case kRate:
	      return 0.0;
	    default:
	      return 0.0;
	  }
	}



	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		gyro.setPIDSourceType(pidSource);
	}

}
