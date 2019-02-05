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
