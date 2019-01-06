package frc.robot.auto.actions;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;


import frc.robot.controllers.MotionController;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Trajectory;
/*** Autonomous Action that follows trajectory ***/
public class PathFollowerCommand extends Command{
	private MotionController motion;
	private double timeout, start_time;
	/****
	 * Constructor for PathFollower Action
	 * @param controllers All the controllers for robot functionality
	 * @param trajectory Pathfinder @jaci Trajectory to follow
	 * @param timeout Time allowed to follow path
	 */
	public PathFollowerCommand(MotionController motion, Trajectory trajectory, double timeout) {
		requires(motion);
		
		this.motion = motion;
		this.timeout = timeout;
		SmartDashboard.putString("MOTIONPROFILING", "SETTING_UP");
		//Sets up configuration, modifiers, and encoder followers
		this.motion.setUp(trajectory);
		SmartDashboard.putString("MOTIONPROFILING", "FINISHED_SETTING_UP");

	}
	@Override
	/*** Checks if Trajectory is finished or we went over timeout ***/
	public boolean isFinished() {
		return (motion.isProfileFinished()) || (Timer.getFPGATimestamp() >= start_time + timeout);
	}
	

	@Override
	/*** Runs in loop and updates PIDVA motion pid controller ***/
	public void execute() {
		motion.update();
	}

	@Override
	/*** Disables motion profiling when finished ***/
	public void end() {
		motion.disable();
	}

	@Override
	/*** Starts Profile following ***/
	public void initialize() {
		//Starts timer
		start_time = Timer.getFPGATimestamp();
		
		//Starts following path
		motion.enable();
	}

	protected void interrupted() {
		end();
	}
}
