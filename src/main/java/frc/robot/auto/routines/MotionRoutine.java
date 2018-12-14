package frc.robot.auto.routines;

import frc.robot.MasterController;
import frc.robot.controllers.MotionController;


import edu.wpi.first.wpilibj.command.CommandGroup;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;

public class MotionRoutine extends CommandGroup{
	MasterController controllers;
	Trajectory trajectory;
	Waypoint[] points = new Waypoint[] {
		    new Waypoint(0, 100, 0)                           // Waypoint @ x=0, y=0,   exit angle=0 radians
		};
	public MotionRoutine(MasterController controllers) {
		//trajectory = MotionController.generateTrajectory(points);
	}

	

}
