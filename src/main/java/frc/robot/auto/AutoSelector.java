package frc.robot.auto;
import frc.robot.auto.routines.*;
import frc.robot.controllers.DriveController;
import frc.robot.controllers.LightController;
import frc.robot.controllers.MotionController;
import frc.robot.controllers.VisionController;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*** Gets selected autonomous routine ***/
public class AutoSelector {
	/*** Radio buttons on SmartDashboard ***/
	private SendableChooser<CommandGroup> autoChooser;

	private DriveController driveController;
	private MotionController motion;
	private VisionController visionController;
	private LightController lights;

	/*** registers autonomous routines in order ***/
	public AutoSelector(DriveController driveController, MotionController motion, VisionController visionController, LightController lights) {
		autoChooser = new SendableChooser<CommandGroup>();

		this.driveController = driveController;
		this.motion = motion;
		this.visionController = visionController;
		this.lights = lights;
		
	} 
	/*** Lists the auto routines on SmartDashboard ***/
	public void listOptions() {
		autoChooser.addDefault("Nothing (Default)", new DoNothingRoutine());
		autoChooser.addObject("Pass Auto Line (Drive 100)", new PassAutoLineRoutine(driveController));
		autoChooser.addObject("Custom Routine (check preferences)", new CustomDistanceRoutine(driveController));
		autoChooser.addObject("Motion Profling Routine", new MotionRoutine(motion));

		SmartDashboard.putData(autoChooser);
	}
	
	/*** Get selected Auto ***/
	public CommandGroup getSelectedAuto() {
		return autoChooser.getSelected();
	}
	

}
