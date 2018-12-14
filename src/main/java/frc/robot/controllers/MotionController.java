package frc.robot.controllers;

import java.io.File;


import frc.robot.hardware.RobotModel;
import frc.robot.Params;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

/**
 * Handles path planning and following
 * 
 * @category controllers
 * @author Peter I. Chacko, Eric Warner, John Sullivan Jr, Jake Boothby, Elliot
 *         Friedlander
 * 
 **/
public class MotionController extends Subsystem {
	private RobotModel robot;
	boolean isProfileFinished = false;

	/**
	 * A single waypoint used for Trajectory Generation.
	 *
	 * A Waypoint is a 'setpoint' that you wish for your trajectory to intersect.
	 * Waypoints are given an X, Y coordinate stating their location in space, and
	 * an exit angle that defines the heading the trajectory should point towards
	 * once this waypoint is reached. The angle is given in Radians
	 *
	 * @author Jaci
	 */
	private Waypoint[] points;
	/**
	 * The Trajectory Configuration outlines the rules to follow while generating
	 * the trajectory. This includes the method used for 'fitting' the spline, the
	 * amount of samples to use, the time difference and maximum values for the
	 * velocity, acceleration and jerk of the trajectory.
	 * 
	 * @author Jaci
	 */
	public Trajectory.Config config;
	/**
	 * The Trajectory object contains an array of Segments that represent the
	 * location, velocity, acceleration, jerk and heading of a particular point in
	 * the trajectory.
	 *
	 * Trajectories can be generated with the Pathfinder class
	 *
	 * @author Jaci
	 */
	public Trajectory trajectory;
	/**
	 * The Tank Modifier will take in a Source Trajectory and a Wheelbase Width and
	 * spit out a Trajectory for each side of the wheelbase. This is commonly used
	 * in robotics for robots which have a drive system similar to a 'tank', where
	 * individual parallel sides are driven independently
	 *
	 * The Source Trajectory is measured from the centre of the drive base. The
	 * modification will not modify the central trajectory
	 *
	 * @author Jaci
	 */
	public TankModifier modifier;
	/**
	 * The EncoderFollower is an object designed to follow a trajectory based on
	 * encoder input. This class can be used for Tank or Swerve drive
	 * implementations.
	 *
	 * @author Jaci
	 */
	public EncoderFollower left;
	/**
	 * The EncoderFollower is an object designed to follow a trajectory based on
	 * encoder input. This class can be used for Tank or Swerve drive
	 * implementations.
	 *
	 * @author Jaci
	 */
	public EncoderFollower right;
	private boolean isEnabled;
	
	/** Gets RobotModel object and sets boolean isEnabled to false **/
	public MotionController(RobotModel robot) {
		this.robot = robot;
		isEnabled = false;
		
	}
	/** Sets up config, trajectory, tank modifier, and encoderFollowers using an array of Waypoints **/
	public void setUp(Waypoint[] points) {
		//Takes in points
		this.points = points;
		//Configures Trajectory based off of Time Step and Kinematics
		config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, Params.dt,
				Params.maximum_velocity, Params.maximum_acceleration, Params.maximum_jerk);
		//Generates trajectory
		trajectory = Pathfinder.generate(points, config);

		// Sets up Modifier and encoder followers from @jaci
		modifier = new TankModifier(trajectory).modify(Params.wheel_base_width);
		left = new EncoderFollower(modifier.getLeftTrajectory());
		right = new EncoderFollower(modifier.getRightTrajectory());
	}

	/** Sets up tank modifier, and encoderFollowers using an already made trajectory **/

	public void setUp(Trajectory trajectoryInput) {
		SmartDashboard.putString("MOTION_PROFILING", "SETUPSTART");

		trajectory = trajectoryInput;
		SmartDashboard.putString("MOTION_PROFILING", "trajectory");
		SmartDashboard.putString("Modifier", "STARTING");

		// TODO find distance between front and rear axles of a vehicle
		modifier = new TankModifier(trajectory).modify(Params.wheel_base_width);
		SmartDashboard.putString("Modifier", "Done");

		left = new EncoderFollower(modifier.getLeftTrajectory());
		SmartDashboard.putString("Modifier", "Left");

		right = new EncoderFollower(modifier.getRightTrajectory());
		SmartDashboard.putString("Modifier", "Right");

	}
	/** Sets up trajectory, tank modifier, and encoderFollowers using a CSV file **/

	public void setUp(File trajectoryCSV) {

		trajectory = Pathfinder.readFromCSV(trajectoryCSV);

		// TODO find distance between front and rear axles of a vehicle
		modifier = new TankModifier(trajectory).modify(Params.wheel_base_width);
		left = new EncoderFollower(modifier.getLeftTrajectory());
		right = new EncoderFollower(modifier.getRightTrajectory());
	}
	/** Static function that returns a trajectory given an array of waypoints **/
	public static Trajectory generateTrajectory(Waypoint[] points) {
		
		SmartDashboard.putString("PATH_GENERATING", "CALCULATING");
		
		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
				Trajectory.Config.SAMPLES_HIGH, Params.dt, Params.maximum_velocity, Params.maximum_acceleration,
				Params.maximum_jerk);
		
		
		Trajectory traj = Pathfinder.generate(points, config);;
		SmartDashboard.putString("PATH_GENERATING", "DONE");

		return traj;
	}
	/** Enables motion profiling **/
	public void enable() {
		//Sets enabled to true
		isEnabled = true;
		//Configures Encoders
		left.configureEncoder(robot.getLeftDriveEncoderRaw(), (int) Math.round(Params.PULSES_PER_ROTATION), Params.WHEEL_DIAMETER);
		right.configureEncoder(robot.getRightDriveEncoderRaw(), (int) Math.round(Params.PULSES_PER_ROTATION), Params.WHEEL_DIAMETER);
		//Configure PIDVA Constants
		left.configurePIDVA(Params.kp, Params.ki, Params.kd, Params.kv, Params.ka);
		right.configurePIDVA(Params.kp, Params.ki, Params.kd, Params.kv, Params.ka);


	}
	
	//Returns true if profile finished
	public boolean isProfileFinished() {
		return isProfileFinished;
	}

	// TODO Put this in control loop
	/** Runs motion profiling **/
	public void update() {
		double deltaTime = Timer.getFPGATimestamp();
		// If we are enabled and the profile isn't finished
		if (isEnabled && !isProfileFinished()) {
			double l = left.calculate(robot.getLeftDriveEncoderRaw());
			double r = right.calculate(robot.getRightDriveEncoderRaw());

			double gyro_heading = 0; //TODO robot.getAngle();

			double desired_heading = Pathfinder.r2d(left.getHeading());
			//Bound Half Degrees and next line just bounds to -180 180
			double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
			double turn = 0.8 * (-1.0 / 80) * angleDifference;
			robot.setLeftMotors(l + turn);
			robot.setRightMotors(r - turn);
	        

	        if (left.isFinished() && right.isFinished()) {
	            isProfileFinished = true;
	        }
		}
		
		deltaTime = Timer.getFPGATimestamp() - deltaTime;
		SmartDashboard.putNumber("Update Interation", deltaTime);

	}
	/** Stops motion profiling **/
	public void disable() {
		isEnabled = false;
		robot.setLeftMotors(0);
		robot.setRightMotors(0);
	}
	@Override
	protected void initDefaultCommand() {
		// TODO Auto-generated method stub
		
	}
	
	

}
