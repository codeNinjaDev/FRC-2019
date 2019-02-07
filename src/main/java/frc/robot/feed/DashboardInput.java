package frc.robot.feed;

import frc.robot.Params;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/*** Allows input from Robot Preferences in SmartDashboard ***/
public class DashboardInput {
	/*** Preferences object to get input ***/
	Preferences preferences;

	// TODO Check if Dashboard Variables or Params works better
	/*** Get Instance of preferences and Update input ***/
	public DashboardInput() {
		// Get overall Input from preferences
		updateInput();
	}

	/*** Updates input from Dashboard ***/
	public void updateInput() {
		preferences = Preferences.getInstance();

		Params.arm_p = preferences.getDouble("ARM P Value", 0);
		Params.arm_i = preferences.getDouble("ARM I Value", 0);
		Params.arm_d = preferences.getDouble("ARM D Value", 0);
		Params.arm_f = preferences.getDouble("ARM F Value", 0);

		Params.ARM_STOW_SETPOINT = preferences.getDouble("Stow angle", 0);
		Params.ARM_LOW_CARGO_SETPOINT = preferences.getDouble("Low cargo angle", 60);
		Params.ARM_MID_CARGO_SETPOINT = preferences.getDouble("Mid cargo angle", 40);
		Params.ARM_HIGH_CARGO_SETPOINT = preferences.getDouble("High cargo angle", 20);
		Params.ARM_LOAD_HATCH_SETPOINT = preferences.getDouble("Load hatch angle", 10);
		Params.ARM_SCORE_HATCH_SETPOINT = preferences.getDouble("Score hatch angle", 10);
		Params.ARM_FLOOR_HATCH_SETPOINT = preferences.getDouble("Floor hatch angle", 80);


		Params.drive_p = preferences.getDouble("DRIVE_P_VALUE", 0.4);
		Params.drive_i = preferences.getDouble("DRIVE_I_VALUE", 0);
		Params.drive_d = preferences.getDouble("DRIVE_D_VALUE", 0.05);

		
		// Set Max Speed to preferences Max Speed
		DashboardVariables.max_speed = preferences.getDouble("MAX_SPEED", 1);

		DashboardVariables.firstAutoTime = preferences.getDouble("AUTO_TIME", 0);

		Params.TIME_DELAY = preferences.getDouble("AUTO_DELAY", 0);
		Params.MAX_SPEED = preferences.getDouble("MAX_SPEED", 1);

		Params.track_base_width = preferences.getDouble("TRACK_BASE_WIDTH", 12);
		Params.wheel_base_width = preferences.getDouble("WHEEL_BASE_WIDTH", 12);
		Params.dt = preferences.getDouble("DELTA_TIME_MP", .2);

	}

	
}
