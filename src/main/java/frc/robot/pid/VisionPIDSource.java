package frc.robot.pid;

import frc.robot.controllers.VisionController;
/*
 * TODO Learn how to make camera a pid source
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;*/

public class VisionPIDSource {
	private VisionController vision;
	public VisionPIDSource(VisionController vision) {
		this.vision = vision;
	}
	
/*	@Override
	public PIDSourceType getPIDSourceType() {
		return robot.camera.pu;
		//TODO Add right encoder somehow
	}
	@Override
	public double pidGet() {
		switch (getPIDSourceType()) {
	    case kDisplacement:
	      return getAverageDistance();
	    case kRate:
	      return 0.0;
	    default:
	      return 0.0;
	  }
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		robot.leftDriveEncoder.setPIDSourceType(PIDSourceType.kRate);
		robot.rightDriveEncoder.setPIDSourceType(PIDSourceType.kRate);		
	}*/

}
