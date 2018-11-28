package frc.robot.auto.routines;


import frc.robot.MasterController;
import frc.robot.Params;
import frc.robot.auto.actions.DriveDistanceCommand;
import frc.robot.auto.actions.WaitCommand;
import frc.robot.feed.DashboardVariables;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class PassAutoLineRoutine extends CommandGroup {
	
	public PassAutoLineRoutine(MasterController controllers) {
		addSequential(new DriveDistanceCommand(controllers, 90, .7, 5, true));
	}
}
