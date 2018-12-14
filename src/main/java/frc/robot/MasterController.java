
package frc.robot;
import frc.robot.controllers.*;
import frc.robot.hardware.RobotModel;
/**
 * Getter and setter class that packages important classes for autonomous. e.g
 * {@code master.getRobotModel()} returns the RobotModel class passed in
 * Robot.java
 * 
 * @author Peter I. Chacko, Eric Warner, John Sullivan Jr, Jake Boothby, Elliot
 *         Friedlander
 * 
 * @param driveTrain
 *            Gets a DriveController object
 * @param robot
 *            Gets a RobotModel object
 * @param motion
 *            Gets a MotionController object
 * @param vision
 *            Gets a VisionController object
 * @param lights
 *            Gets a LightConroller object
 * 
 */
public class MasterController {
	/*** Vision Controller ***/
	private VisionController vision;
	/*** Drive Controller ***/
	private DriveController driveTrain;
	/*** Light Controller ***/
	private LightController lights;
	/*** Robot Model ***/
	private RobotModel robot;
	/*** Motion Profiling Controller ***/
	private MotionController motion;
	

	/**
	 * 
	 * Takes private classes and sets to the ones assigned in Robot.java
	 * 
	 * @param driveTrain
	 *            Gets a DriveController object
	 * @param robot
	 *            Gets a RobotModel object
	 * @param motion
	 *            Gets a MotionController object
	 * @param vision
	 *            Gets a VisionController object
	 * @param lights
	 *            Gets a LightConroller object
	 */
	public MasterController(DriveController driveTrain, RobotModel robot, MotionController motion,
			VisionController vision, LightController lights) {

		this.vision = vision;
		this.driveTrain = driveTrain;
		this.robot = robot;
		this.motion = motion;
		this.lights = lights;
	}

	/**
	 * Returns VisionController object from Robot.java
	 *
	 **/
	public VisionController getVisionController() {
		return vision;
	}

	/**
	 * Returns RobotModel object from Robot.java
	 *
	 **/
	public RobotModel getRobotModel() {
		return robot;
	}

	/**
	 * Returns DriveController object from Robot.java
	 *
	 **/
	public DriveController getDriveController() {
		return driveTrain;
	}

	/**
	 * Returns LightController object from Robot.java
	 *
	 **/
	public LightController getLightController() {
		return lights;
	}

	/**
	 * Returns MotionController object from Robot.java
	 *
	 **/
	public MotionController getMotionController() {
		return motion;
	}


}