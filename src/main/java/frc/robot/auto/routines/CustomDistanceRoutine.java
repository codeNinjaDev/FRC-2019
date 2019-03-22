 package frc.robot.auto.routines;

import frc.robot.Params;
import frc.robot.auto.actions.DriveIntervalCommand;
import frc.robot.auto.actions.WaitCommand;
import frc.robot.controllers.DriveController;
import frc.robot.feed.DashboardVariables;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class CustomDistanceRoutine extends CommandGroup {
	private DriveController driveController;
	public CustomDistanceRoutine(DriveController driveController) {
		this.driveController = driveController;
		addSequential(new WaitCommand(Params.TIME_DELAY));
		addSequential(new DriveIntervalCommand(driveController, DashboardVariables.firstAutoTime, 1, 0));
	}
	

}