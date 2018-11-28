package frc.robot.pid;

import frc.robot.hardware.RobotModel;
import edu.wpi.first.wpilibj.PIDOutput;
/*** Sets both wheel sides as a PID output***/
public class WheelsPIDOutput implements PIDOutput {

	private RobotModel.Wheels wheels;
	private RobotModel robot;
	private double loopOutput;
	public WheelsPIDOutput(RobotModel.Wheels wheels, RobotModel robot) {
		this.wheels = wheels;
		this.robot = robot;
	}
	@Override
	public void pidWrite(double output) {
		loopOutput = output;
		robot.setWheelSpeed(wheels, output);
	}
	
	double getPIDLoopOutput() {
		return loopOutput;
	}


}
