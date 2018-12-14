package frc.robot.auto;
import frc.robot.auto.routines.*;
import frc.robot.MasterController;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*** Gets selected autonomous routine ***/
public class AutoSelector {
	/*** Radio buttons on SmartDashboard ***/
	private SendableChooser<CommandGroup> autoChooser;
	private MasterController controllers;

	/*** registers autonomous routines in order ***/
	public AutoSelector(MasterController controllers) {
		this.controllers = controllers;
		autoChooser = new SendableChooser<CommandGroup>();


		
	} 
	/*** Lists the auto routines on SmartDashboard ***/
	public void listOptions() {
		autoChooser.addDefault("Nothing (Default)", new DoNothingRoutine());
		autoChooser.addObject("Pass Auto Line (Drive 100)", new PassAutoLineRoutine(controllers));
		autoChooser.addObject("Custom Routine (check preferences)", new CustomDistanceRoutine(controllers));
		autoChooser.addObject("Motion Profling Routine", new MotionRoutine(controllers));
		SmartDashboard.putData(autoChooser);
	}
	
	/*** Get selected Auto ***/
	public CommandGroup getSelectedAuto() {
		return autoChooser.getSelected();
	}
	

}
