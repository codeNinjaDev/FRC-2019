package frc.robot.auto.routines;

import frc.robot.controllers.MotionController;
import edu.wpi.first.wpilibj.command.CommandGroup;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;

public class MotionRoutine extends CommandGroup{
	MotionController motion;
	Trajectory trajectory;
	Waypoint[] points = new Waypoint[] {
		    new Waypoint(0, 100, 0)                           // Waypoint @ x=0, y=0,   exit angle=0 radians
		};
	public MotionRoutine(MotionController motion) {
		//trajectory = MotionController.generateTrajectory(points);
	}

	

}
