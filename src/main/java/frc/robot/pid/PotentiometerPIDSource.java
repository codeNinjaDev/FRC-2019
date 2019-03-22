package frc.robot.pid;

import frc.robot.hardware.TenTurnPotentiometer;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Timer;
/*** Sets 10 turn Potentiometer as a PID Source ***/
public class PotentiometerPIDSource implements PIDSource {
	/*** Ten Turn Potentiometer ***/
	TenTurnPotentiometer pot;
	/*** Current Angle from pot***/
	double currentAngle;
	/*** Past Angle from pot ***/
	double pastAngle;
	/*** Current Time ***/
	double currentTime;
	/*** Past Time ***/
	double pastTime;
	/*** Timer to calculate arm rate ***/
	Timer armTimer;

	public PotentiometerPIDSource(TenTurnPotentiometer pot) {
		
		this.pot = pot;
		pastAngle = pot.getAngle();
		currentAngle = pot.getAngle();
		armTimer = new Timer();
		armTimer.start();
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		pot.setPIDSourceType(pidSource);
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		return pot.getPIDSourceType();
	}

	@Override
	public double pidGet() {
		switch (getPIDSourceType()) {
		case kDisplacement:
			return pot.getAngle();
		case kRate:
			currentTime = armTimer.get();
			currentAngle = pot.getAngle();
			double deltaAngle = currentAngle - pastAngle;
			double deltaTime = currentTime - pastTime;
			
			double anglePerSecond =  deltaAngle/deltaTime;
			
			pastTime = currentTime;
			pastAngle = currentAngle;
			return anglePerSecond;
		default:
			return 0;
		}
	}

}