package frc.robot.auto.routines;

import frc.robot.auto.actions.PathFollowerCommand;
import frc.robot.controllers.MotionController;

import java.io.File;

import edu.wpi.first.wpilibj.command.CommandGroup;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;

public class MotionRoutine extends CommandGroup{
	MotionController motion;
	Trajectory trajectory;
	
	public MotionRoutine(MotionController motion) {
		
		File csvFile = new File("/home/lvuser/deploy/path/test.pf1.csv");
		trajectory = Pathfinder.readFromCSV(csvFile);
		addSequential(new PathFollowerCommand(motion, trajectory, 4));
	}

	

}
