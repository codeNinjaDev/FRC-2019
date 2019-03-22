package frc.robot.auto;
import frc.robot.auto.routines.*;
import frc.robot.controllers.DriveController;
import frc.robot.controllers.LightController;
import frc.robot.controllers.MotionController;
import frc.robot.controllers.VisionController;
import frc.robot.controllers.WristController;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
	private WristController wrist;

	/*** registers autonomous routines in order ***/
	public AutoSelector(DriveController driveController, MotionController motion, VisionController visionController, WristController wrist, LightController lights) {
		autoChooser = new SendableChooser<CommandGroup>();
		this.wrist = wrist;
		this.driveController = driveController;
		this.motion = motion;
		this.visionController = visionController;
		this.lights = lights;
		
	} 
	/*** Lists the auto routines on SmartDashboard ***/
	public void listOptions() {
		autoChooser.setDefaultOption("Nothing (Default)", new DoNothingRoutine());
		autoChooser.addOption("Pass Auto Line (Drive 100)", new PassAutoLineRoutine(driveController));
		autoChooser.addOption("Custom Routine (check preferences)", new CustomDistanceRoutine(driveController));
		autoChooser.addOption("Motion Profling Routine", new MotionRoutine(motion));
		autoChooser.addOption("Right Side Hatch", new RightSideHatch(driveController, wrist, visionController));
		autoChooser.addOption("Left Side Hatch", new LeftSideHatch(driveController, wrist, visionController));
		autoChooser.addOption("Right Front Hatch", new RightFrontHatch(driveController, wrist, visionController));
		autoChooser.addOption("Left Front Hatch", new LeftFrontHatch(driveController, wrist, visionController));


		Shuffleboard.getTab("Autonomous").add("Autonomous Commands", autoChooser);
	}
	
	/*** Get selected Auto ***/
	public CommandGroup getSelectedAuto() {
		return autoChooser.getSelected();
	}
	public void cancelAuto() {
		getSelectedAuto().cancel();
		Scheduler.getInstance().removeAll();
	}

}
