package frc.robot.auto.routines;

import frc.robot.auto.actions.PathFollowerCommand;
import frc.robot.controllers.MotionController;
import edu.wpi.first.wpilibj.command.CommandGroup;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;

public class MotionRoutine extends CommandGroup{
	MotionController motion;
	Trajectory trajectory;
	
	public MotionRoutine(MotionController motion) {
		Waypoint[] points = new Waypoint[] {
		    new Waypoint(0, 100, 0)                           // Waypoint @ x=0, y=0,   exit angle=0 radians
		};
		trajectory = MotionController.generateTrajectory(points);
		addSequential(new PathFollowerCommand(motion, trajectory, 4));
	}

	

}
