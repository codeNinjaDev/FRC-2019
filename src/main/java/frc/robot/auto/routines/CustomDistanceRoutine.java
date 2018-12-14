 package frc.robot.auto.routines;

import frc.robot.MasterController;
import frc.robot.Params;
import frc.robot.auto.actions.DriveIntervalCommand;
import frc.robot.auto.actions.WaitCommand;
import frc.robot.feed.DashboardVariables;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class CustomDistanceRoutine extends CommandGroup {
	public CustomDistanceRoutine(MasterController controllers) {
		addSequential(new WaitCommand(Params.TIME_DELAY));
		addSequential(new DriveIntervalCommand(controllers, DashboardVariables.firstAutoTime, 1, 0));
	}
	

}